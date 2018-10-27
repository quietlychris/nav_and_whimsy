// cmoran, 2018

mod kinematics_lib;
mod physics_lib;
mod algorithms_lib;
mod tests;

use kinematics_lib::DoF;
use kinematics_lib::Point;
use kinematics_lib::SMD;
use kinematics_lib::State;
use physics_lib::Object;
use physics_lib::Environment;

use std::f32;
//for writing to log file
use std::fs::File;
use std::io::{BufWriter, Write};

pub const DT: f32 = 0.01;
pub const PI: f32 = 3.14159;

fn main() {
    let f = File::create("run_log.csv").expect("Unable to create file");
    let mut f = BufWriter::new(f);
    let g = File::create("waypoints.csv").expect("Unable to create file");
    let mut g = BufWriter::new(g);
    //let path = Path::new("waypoints.csv");
    //let mut writer = Writer::from_file(&path);

    {
        // Writes headers to position file
        f.write_all(b"time")
            .expect("couldn't write xaxis.var header");
        f.write_all(b",").expect("couldn't write comma");

        f.write_all(b"xaxis.var")
            .expect("couldn't write xaxis.var header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"xaxis.vard")
            .expect("couldn't write xaxis.vard header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"xaxis.vardd")
            .expect("couldn't write xaxis.vardd header");
        f.write_all(b",").expect("couldn't write comma");

        f.write_all(b"yaxis.var")
            .expect("couldn't write yaxis.var header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"yaxis.vard")
            .expect("couldn't write yaxis.vard header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"yaxis.vardd")
            .expect("couldn't write yaxis.vardd header");
        f.write_all(b",").expect("couldn't write comma");

        f.write_all(b"zaxis.var")
            .expect("couldn't write zaxis.var header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"zaxis.vard")
            .expect("couldn't write zaxis.vard header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"zaxis.vardd")
            .expect("couldn't write zaxis.vardd header");
        f.write_all(b",").expect("couldn't write comma");

        f.write_all(b"yaw.var")
            .expect("couldn't write yaw.var header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"yaw.vard")
            .expect("couldn't write yaw.vard header");
        f.write_all(b",").expect("couldn't write comma");
        f.write_all(b"yaw.vardd")
            .expect("couldn't write yaw.vardd header");
        f.write_all(b"\n").expect("couldn't write comma");
    }

    /*let mut file_header = Vec::new();
    values.push(time);
    values.push(xaxis.var);
    values.push(xaxis.vard);
    values.push(xaxis.vardd);
    values.push(yaxis.var);
    values.push(yaxis.vard);
    values.push(yaxis.vardd);
    values.push(zaxis.var);
    values.push(zaxis.vard);
    values.push(zaxis.vardd);
    values.push(yaw.var);
    values.push(yaw.vard);
    values.push(yaw.vardd);*/

    // Creates an array of points to be searched
    let mut mesh: Vec<Point> = Vec::new();
    for z in -1isize..2isize {
        for y in -3isize..4isize {
            for x in -3isize..4isize {
                let mut point = Point::new(x as f32, y as f32, z as f32, false);
                mesh.push(point);
            }
        }
    }

    /*mesh.push(Point::new(0.0, 0.0, 1.0, false));
    mesh.push(Point::new(-1.0, 4.0, -2.0, false));
    mesh.push(Point::new(1.0, 3.0, 3.0, false));
    mesh.push(Point::new(0.0, 0.0, 0.0, false));*/

    for a in 0..mesh.len() {
        // Writes waypoints in mesh[] to file
        g.write_all(mesh[a].x.to_string().as_bytes())
            .expect("couldn't write x position for waypoint");
        g.write_all(b",").expect("couldn't write comma");
        g.write_all(mesh[a].y.to_string().as_bytes())
            .expect("could write y position for waypoint");
        g.write_all(b",").expect("couldn't write comma");
        g.write_all(mesh[a].z.to_string().as_bytes())
            .expect("could write z position for waypoint");
        g.write_all(b"\n").expect("couldn't write endline");
    }

    let mut list: Vec<ListPoint> = Vec::new(); // Creates a new vector of points w/scores
    for b in 0..mesh.len() {
        let list_point = calculate_score(mesh[b],desired); // Calulates score of each point and returns ListPoint object
        list.push(list_point); // Adds ListPoint object to the ListPoint Vector
    }

    let mut min = list[0].score; // Sets the minimum value of the ListPoint vector score
    let mut order_vec: Vec<usize> = Vec::new() // Creates a new vector in which to the put the min-max ordered list of scored points
    for c in 0..list.len() {
        if list[c].score < min { // If the score ofa point is below that of the intitial setting
            min = list[c].score; // Set that point as having the minimum value
            order_vec.push(c); // Add that ListPoint's index to the ordered list of scored points
            list.remove(c); // Remove that ListPoint from the list
        }
    }

    // Creates the environment, a spherical object, and sets the intial position to the origin
    let environment = Environment::new(false);
    let sphere_radius: f32 = 0.1;
    let sphere_mass: f32 = 1.0;
    let sphere_surface: f32 = 4.0*PI*(sphere_radius.powf(2.0));
    let mut sphere = Object::new(sphere_mass,sphere_surface,environment,State::default());

    // Creates SMDs for both x,y, and yaw DoF
    let position_smd = SMD::new(1.0, 0.3, 0.90);
    let yaw_smd = SMD::new(0.1, 0.3, 0.7);
    let z_smd = SMD::new(1.0, 0.3, 0.5);

    // Sets physical limit parameters for allowances
    let yaw_allowance: f32 = 0.2;
    let yaw_speed_allowance: f32 = 3.0;
    let max_speed: f32 = 0.5;
    let zero_speed_allowance: f32 = 0.01;
    let time_max: f32 = 1800.0;

    // Starts time for simulation loop
    let mut time: f32 = 0.0;
    for counter_1 in 0..mesh.len() {
        let desired = State::new(
            DoF::new(mesh[counter_1].x, 0.0, 0.0),
            DoF::new(mesh[counter_1].y, 0.0, 0.0),
            DoF::new(mesh[counter_1].z, 0.0, 0.0),
            DoF::new(0.0, 0.0, 0.0),
        );

        while mesh[counter_1].yon != true {
            let desired_yaw = sphere.state.calculate_yaw(desired);
            let total_velocity =
                (sphere.state.xaxis.vard.powf(2.0) + sphere.state.yaxis.vard.powf(2.0)).powf(0.5);
            //println!("In main(): desired_yaw vs sphere.state: {:.3} {:.3}",desired_yaw,sphere.state.yaw.var);
            let mut acceleration: (f32, f32, f32) = (0.0, 0.0, 0.0);
            let yaw_difference = (desired_yaw - sphere.state.yaw.var).abs();

            // NEED THREE CASES HERE:
            // In right direction, proceed to target
            // In wrong direction, slow to zero velocity
            // At zero velocity in wrong direction, turn to target

            if yaw_difference < yaw_allowance && sphere.state.yaw.vard.abs() < yaw_speed_allowance
                && total_velocity < max_speed
            {
                //println!("CASE 1: Everything's right, b/c yaw {:.3} ~ {:.3} && speed {:.3} < {:.3}", sphere.state.yaw.var,desired_yaw,total_velocity,max_speed);
                acceleration = (
                    sphere.state.get_xy_acceleration(desired, position_smd, yaw_allowance),
                    sphere.state.get_z_acceleration(desired, z_smd),
                    sphere.state.get_yaw_acceleration(desired_yaw, yaw_smd),
                );
            } else if total_velocity < zero_speed_allowance && yaw_difference > yaw_allowance {
                //println!("CASE 2: Just updating direction, b/c total_v = {:.3} < {:.3} and yaw {:.3} ~ {:3} -> yaw-delta {:.3} > {:.3} ",total_velocity,zero_speed_allowance,sphere.state.yaw.var,desired_yaw,yaw_difference,yaw_allowance);
                acceleration.1 = sphere.state.get_z_acceleration(desired, z_smd);
                acceleration.2 = sphere.state.get_yaw_acceleration(desired_yaw, yaw_smd);
            } else {
                //println!("CASE 3: trending speed towards zero, b/c ({:.3} > {:.3})",total_velocity,zero_speed_allowance);
                // If direction is right, and angular speed isn't too high, update direction and position
                acceleration = sphere.state.trend_speed_towards_zero(yaw_smd);
                acceleration.1 = sphere.state.get_z_acceleration(desired, z_smd);
            }

            // Assigns acceleration values for the various degrees of freedom
            let xvardd = acceleration.0 * sphere.state.yaw.var.to_radians().cos();
            let yvardd = acceleration.0 * sphere.state.yaw.var.to_radians().sin();
            let zvardd = acceleration.1;
            let yawvardd = acceleration.2;
            //println!("acceleration tuple: {:?},acceleration);
            sphere.state.kinematic_update(xvardd, yvardd, zvardd, yawvardd);
            //sphere.state.print();
            {
                // Writes state data to position file
                f.write_all(time.to_string().as_bytes())
                    .expect("couldn't write x position");
                f.write_all(b",").expect("couldn't write comma");

                f.write_all(sphere.state.xaxis.var.to_string().as_bytes())
                    .expect("couldn't write x position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(sphere.state.xaxis.vard.to_string().as_bytes())
                    .expect("couldn't write x position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(sphere.state.xaxis.vardd.to_string().as_bytes())
                    .expect("couldn't write x position");
                f.write_all(b",").expect("couldn't write comma");

                f.write_all(sphere.state.yaxis.var.to_string().as_bytes())
                    .expect("could write y position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(sphere.state.yaxis.vard.to_string().as_bytes())
                    .expect("could write y position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(sphere.state.yaxis.vardd.to_string().as_bytes())
                    .expect("could write y position");
                f.write_all(b",").expect("couldn't write comma");

                f.write_all(sphere.state.zaxis.var.to_string().as_bytes())
                    .expect("could write z position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(sphere.state.zaxis.vard.to_string().as_bytes())
                    .expect("could write z position");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(sphere.state.zaxis.vardd.to_string().as_bytes())
                    .expect("could write z position");
                f.write_all(b",").expect("couldn't write comma");

                f.write_all(sphere.state.yaw.var.to_string().as_bytes())
                    .expect("could write yaw var");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(sphere.state.yaw.vard.to_string().as_bytes())
                    .expect("could write yaw vard");
                f.write_all(b",").expect("couldn't write comma");
                f.write_all(sphere.state.yaw.vardd.to_string().as_bytes())
                    .expect("could write yaw vardd");
                f.write_all(b"\n").expect("couldn't write endline");
            }

            let min_distance: f32 = 0.1;
            for counter_2 in 0..mesh.len() {
                // If close enough to a mesh point, marks that point as true
                if (sphere.state.xaxis.var - mesh[counter_2].x).abs() < min_distance
                    && (sphere.state.yaxis.var - mesh[counter_2].y).abs() < min_distance
                    && (sphere.state.zaxis.var - mesh[counter_2].z).abs() < min_distance
                    && mesh[counter_2].yon != true
                {
                    mesh[counter_2].make_true();
                }
            }

            time = time + DT;
            if time > time_max {
                break;
            }
        }
    }
    if time > time_max {
        println!("Went over maximum time");
    }
    println!("End time is: {:.2}", time);
    for a in 0..mesh.len() {
        // Prints mesh results
        mesh[a].print();
    }
}
