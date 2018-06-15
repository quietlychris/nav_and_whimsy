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
//use std::fs::write;

//use std::convert::{From, Into};
//use std::string::String;

pub const DT: f32 = 0.01;
//pub const thrust_const: f32 = 1.0;
//pub const length: usize = 9; // used as grid constant

fn main() {

    let f = File::create("run_log.csv").expect("Unable to create file");
    let mut f = BufWriter::new(f);
    let g = File::create("waypoints.csv").expect("Unable to create file");
    let mut g = BufWriter::new(g);

    let mut mesh: Vec<Point> = Vec::new();
    for y in -3isize..4isize {
        for x in -3isize..4isize {
            let mut point = Point::new(x as f32,y as f32,false);
            mesh.push(point);
        }
    }


    let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0));

    /*let desired = State::new(DoF::new(1.0,0.0,0.0),
                             DoF::new(1.0,0.0,0.0),
                             DoF::new(0.0,0.0,0.0));*/
    let position_smd = SMD::new(1.0,0.9,0.90);
    let yaw_smd = SMD::new(0.1,0.1,0.9);
    let yaw_allowance: f32 = 0.50;
    let yaw_speed_allowance: f32 = 3.0;
    let max_speed: f32 = 0.3;
    let zero_speed_allowance: f32 = 0.01;
    let time_max: f32 = 900.0;

    let mut time: f32 = 0.0;
    for counter_1 in 0..mesh.len()
    {

        while mesh[counter_1].yon != true
        {
            let desired = State::new(DoF::new(mesh[counter_1].x,0.0,0.0),
                                     DoF::new(mesh[counter_1].y,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0));

            { // Writes waypoint to file
                 g.write_all(desired.xaxis.var.to_string().as_bytes()).expect("couldn't write x position for waypoint");
                 g.write_all(b",").expect("couldn't write comma");
                 g.write_all(desired.yaxis.var.to_string().as_bytes()).expect("could write y position for waypoint");
                 g.write_all(b"\n").expect("couldn't write endline");
            }

            let desired_yaw = current.calculate_yaw(desired);
            let total_velocity = (current.xaxis.vard.powf(2.0) + current.yaxis.vard.powf(2.0)).powf(0.5);
            //println!("In main(): desired_yaw vs current: {:.3} {:.3}",desired_yaw,current.yaw.var);
            let mut acceleration: (f32, f32) = (0.0,0.0);
            let yaw_difference = (desired_yaw - current.yaw.var).abs();

            // NEED THREE CASES HERE:
            // In right direction, proceed to target
            // In wrong direction, slow to zero velocity
            // At zero velocity in wrong direction, turn to target

            if yaw_difference < yaw_allowance && current.yaw.vard.abs() < yaw_speed_allowance && total_velocity < max_speed {
                println!("CASE 1: Everything's right, b/c yaw {:.2} ~ {:2} && speed {:.2} < {:.2}",
                    current.yaw.var,desired_yaw,total_velocity,max_speed);
                acceleration = (current.update_position(desired,position_smd,yaw_allowance),current.update_direction(desired_yaw,yaw_smd));
            }
            else if total_velocity < zero_speed_allowance && yaw_difference > yaw_allowance
            {
                println!("CASE 2: Just updating direction, b/c total_v = {:.2} < {:.2} and yaw {:.2} ~ {:2} -> yaw-delta {:.2} > {:.2} ",total_velocity,zero_speed_allowance,current.yaw.var,desired_yaw,yaw_difference,yaw_allowance);
                acceleration.1 = current.update_direction(desired_yaw,yaw_smd);
            }
            else
            {
                println!("CASE 3: trending speed towards zero, b/c ({:.2} > {:.2})",total_velocity,max_speed);
                // If direction is right, and angular speed isn't too high, update direction and position
                acceleration = current.trend_speed_towards_zero(yaw_smd);
            }

            let xvardd = acceleration.0*current.yaw.var.to_radians().cos();
            let yvardd = acceleration.0*current.yaw.var.to_radians().sin();
            let yawvardd = acceleration.1;
            current.kinematic_update(xvardd,yvardd,yawvardd);

            { // Writes position to file
                f.write_all(current.xaxis.var.to_string().as_bytes()).expect("couldn't write x position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(current.yaxis.var.to_string().as_bytes()).expect("could write y position");
                f.write_all(b"\n").expect("couldn't write endline");
            }

            let min_distance: f32 = 0.1;
            for counter_2 in 0..mesh.len()
            {

                if (current.xaxis.var - mesh[counter_2].x).abs() < min_distance
                    && (current.yaxis.var - mesh[counter_2].y).abs() < min_distance
                        && mesh[counter_2].yon!=true
                        {
                            mesh[counter_2].make_true();
                        }

            }

            time = time + DT;
            if time > time_max {break;}
        }
    for a in 0..mesh.len() {
        mesh[a].print();
    }
    if time > time_max {break;}


    }
}
