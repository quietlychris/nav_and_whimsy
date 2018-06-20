// cmoran, 2018

mod kinematics_lib;
mod tests;

use kinematics_lib::SMD;
use kinematics_lib::DoF;
use kinematics_lib::State;
use kinematics_lib::Point;
//use kinematics_lib::*;
use std::f32;
//for writing to log file
use std::fs::File;
use std::io::{Write, BufWriter};

pub const DT: f32 = 0.01;

fn main() {

    let f = File::create("run_log.csv").expect("Unable to create file");
    let mut f = BufWriter::new(f);
    let g = File::create("waypoints.csv").expect("Unable to create file");
    let mut g = BufWriter::new(g);
    { // Writes headers to position file
        f.write_all(b"xaxis.var").expect("couldn't write xaxis.var header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"xaxis.vard").expect("couldn't write xaxis.vard header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"xaxis.vardd").expect("couldn't write xaxis.vardd header");
        f.write_all(b",").expect("couldn't write comma");

        f.write_all(b"yaxis.var").expect("couldn't write yaxis.var header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"yaxis.vard").expect("couldn't write yaxis.vard header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"yaxis.vardd").expect("couldn't write yaxis.vardd header");
        f.write_all(b",").expect("couldn't write comma");

        f.write_all(b"zaxis.var").expect("couldn't write zaxis.var header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"zaxis.vard").expect("couldn't write zaxis.vard header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"zaxis.vardd").expect("couldn't write zaxis.vardd header");
        f.write_all(b",").expect("couldn't write comma");

        f.write_all(b"yaw.var").expect("couldn't write yaw.var header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"yaw.vard").expect("couldn't write yaw.vard header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"yaw.vardd").expect("couldn't write yaw.vardd header");
        f.write_all(b"\n").expect("couldn't write comma");
    }

    // Creates an array of points to be searched
    let mut mesh: Vec<Point> = Vec::new();
    /*for z in -1isize..1isize {
        for y in -3isize..4isize {
            for x in -3isize..4isize {
                let mut point = Point::new(x as f32,y as f32,3.0,false);
                mesh.push(point);
            }
        }
    }*/
    //TO_DO: This is a case that throws an error; diectly above or below the current position
    //mesh.push(Point::new(0.0,0.0,1.0,false));

    mesh.push(Point::new(1.0,1.0,1.0,false));
    mesh.push(Point::new(-1.0,4.0,-2.0,false));
    mesh.push(Point::new(1.0,3.0,3.0,false));
    mesh.push(Point::new(0.0,0.0,0.0,false));

    for a in 0..mesh.len() { // Writes waypoints in mesh[] to file
        g.write_all(mesh[a].x.to_string().as_bytes()).expect("couldn't write x position for waypoint");
        g.write_all(b",").expect("couldn't write comma");
        g.write_all(mesh[a].y.to_string().as_bytes()).expect("could write y position for waypoint");
        g.write_all(b",").expect("couldn't write comma");
        g.write_all(mesh[a].z.to_string().as_bytes()).expect("could write z position for waypoint");
        g.write_all(b"\n").expect("couldn't write endline");
    }

    // Sets initial position
    let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0));

    // Creates SMDs for both x,y, and yaw DoF
    let position_smd = SMD::new(1.0,0.7,0.90);
    let yaw_smd = SMD::new(0.1,0.3,0.7);
    let z_smd = SMD::new(1.0,0.8,0.5);

    // Sets physical limit parameters for allowances
    let yaw_allowance: f32 = 0.2;
    let yaw_speed_allowance: f32 = 3.0;
    let max_speed: f32 = 0.5;
    let zero_speed_allowance: f32 = 0.01;
    let time_max: f32 = 600.0;

    let mut time: f32 = 0.0;
    for counter_1 in 0..mesh.len()
    {
        while mesh[counter_1].yon != true
        {
            let desired = State::new(DoF::new(mesh[counter_1].x,0.0,0.0),
                                     DoF::new(mesh[counter_1].y,0.0,0.0),
                                     DoF::new(mesh[counter_1].z,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            let desired_yaw = current.calculate_yaw(desired);
            let total_velocity = (current.xaxis.vard.powf(2.0) + current.yaxis.vard.powf(2.0)).powf(0.5);
            //println!("In main(): desired_yaw vs current: {:.3} {:.3}",desired_yaw,current.yaw.var);
            let mut acceleration: (f32, f32, f32) = (0.0,0.0,0.0);
            let yaw_difference = (desired_yaw - current.yaw.var).abs();

            // NEED THREE CASES HERE:
            // In right direction, proceed to target
            // In wrong direction, slow to zero velocity
            // At zero velocity in wrong direction, turn to target

            if yaw_difference < yaw_allowance && current.yaw.vard.abs() < yaw_speed_allowance && total_velocity < max_speed {
                //println!("CASE 1: Everything's right, b/c yaw {:.3} ~ {:.3} && speed {:.3} < {:.3}", current.yaw.var,desired_yaw,total_velocity,max_speed);
                acceleration = (current.get_xy_acceleration(desired,position_smd,yaw_allowance),                             current.get_z_acceleration(desired,z_smd),current.get_yaw_acceleration(desired_yaw,yaw_smd));
            }
            else if total_velocity < zero_speed_allowance && yaw_difference > yaw_allowance
            {
                //println!("CASE 2: Just updating direction, b/c total_v = {:.3} < {:.3} and yaw {:.3} ~ {:3} -> yaw-delta {:.3} > {:.3} ",total_velocity,zero_speed_allowance,current.yaw.var,desired_yaw,yaw_difference,yaw_allowance);
                acceleration.2 = current.get_yaw_acceleration(desired_yaw,yaw_smd);
            }
            else
            {
                //println!("CASE 3: trending speed towards zero, b/c ({:.3} > {:.3})",total_velocity,zero_speed_allowance);
                // If direction is right, and angular speed isn't too high, update direction and position
                acceleration = current.trend_speed_towards_zero(yaw_smd);
            }

            // Assigns acceleration values for the various degrees of freedom
            let xvardd = acceleration.0*current.yaw.var.to_radians().cos();
            let yvardd = acceleration.0*current.yaw.var.to_radians().sin();
            let zvardd = acceleration.1;
            let yawvardd = acceleration.2;
            //println!("acceleration tuple: {:?},acceleration);
            current.kinematic_update(xvardd,yvardd,zvardd,yawvardd);
            //current.print();
            { // Writes state data to position file
                f.write_all(current.xaxis.var.to_string().as_bytes()).expect("couldn't write x position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(current.xaxis.vard.to_string().as_bytes()).expect("couldn't write x position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(current.xaxis.vardd.to_string().as_bytes()).expect("couldn't write x position");
                f.write_all(b",").expect("couldn't write comma");

                f.write_all(current.yaxis.var.to_string().as_bytes()).expect("could write y position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(current.yaxis.vard.to_string().as_bytes()).expect("could write y position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(current.yaxis.vardd.to_string().as_bytes()).expect("could write y position");
                f.write_all(b",").expect("couldn't write comma");

                f.write_all(current.zaxis.var.to_string().as_bytes()).expect("could write z position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(current.zaxis.vard.to_string().as_bytes()).expect("could write z position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(current.zaxis.vardd.to_string().as_bytes()).expect("could write z position");
                f.write_all(b",").expect("couldn't write comma");

                f.write_all(current.yaw.var.to_string().as_bytes()).expect("could write yaw var");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(current.yaw.vard.to_string().as_bytes()).expect("could write yaw vard");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(current.yaw.vardd.to_string().as_bytes()).expect("could write yaw vardd");
                f.write_all(b"\n").expect("couldn't write endline");
            }

            let min_distance: f32 = 0.1;
            for counter_2 in 0..mesh.len() { // If close enough to a mesh point, marks that point as true
                if (current.xaxis.var - mesh[counter_2].x).abs() < min_distance
                    && (current.yaxis.var - mesh[counter_2].y).abs() < min_distance
                        && (current.zaxis.var - mesh[counter_2].z).abs() < min_distance
                            && mesh[counter_2].yon!=true
                            {
                                mesh[counter_2].make_true();
                            }
            }

            time = time + DT;
            if time > time_max {break;}
        }

    }
    if time > time_max {println!("Went over maximum time");}
    println!("End time is: {:.2}",time);
    for a in 0..mesh.len() { // Prints mesh results
        mesh[a].print();
    }
}
