// cmoran, 2018

mod algorithms_lib;
mod controls_lib;
mod physics_lib;
mod tests;

use crate::controls_lib::{ControlParams, Point, SMD};
use crate::physics_lib::{DoF, Environment, Object, State};

use std::f32;
//for writing to log file
use std::fs::File;
use std::io::{BufWriter, Write};

pub const DT: f32 = 0.01;

fn main() {
    // Creates log file and writes in the header for a csv file
    let f = File::create("run_log.csv").expect("Unable to create file");
    let mut f = BufWriter::new(f);
    f.write_all(b"time,xaxis.var,xaxis.vard,xaxis.vardd,yaxis.var,yaxis.vard,yaxis.vardd,zaxis.var,zaxis.vard,zaxis.vardd,yaw.var,yaw.vard,yaw.vardd\n")
        .expect("couldn't write headers to file");

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
    /*
    mesh.push(Point::new(0.0, 0.0, 1.0, false));
    mesh.push(Point::new(-1.0, 4.0, -2.0, false));
    mesh.push(Point::new(1.0, 3.0, 3.0, false));
    mesh.push(Point::new(0.0, 0.0, 0.0, false));
    */

    // Creates a csv file with all the waypoints in it, for post-processing
    let g = File::create("waypoints.csv").expect("Unable to create file");
    let mut g = BufWriter::new(g);
    for a in 0..mesh.len() {
        // Writes waypoints in mesh[] to file
        let xyz_pos = format!(
            "{},{},{}\n",
            mesh[a].x.to_string(),
            mesh[a].y.to_string(),
            mesh[a].z.to_string()
        );
        // println!("{}", xyz_pos);
        g.write_all(xyz_pos.as_bytes())
            .expect("Couldn't write formatted waypoint data to file");
    }

    // Creates the environment, a spherical object, and sets the intial position to the origin
    let environment = Environment::new(false);

    // Creates basis for controls laws for specified Object, as well has sets control limits for various physical constraints
    // Builds ControlParams struct before the Object struct so it can be included
    let control_params = ControlParams {
        position_smd: SMD::new(1.0, 0.3, 0.90),
        yaw_smd: SMD::new(0.1, 0.3, 0.7),
        z_smd: SMD::new(0.8, 0.3, 0.5),
        yaw_allowance: 0.2,
        yaw_speed_allowance: 3.0,
        max_speed: 0.5,
        zero_speed_allowance: 0.01,
    };

    let sphere_radius: f32 = 0.1;
    let sphere_mass: f32 = 1.0;
    let sphere_surface: f32 = 4.0 * std::f32::consts::PI * (sphere_radius.powf(2.0));
    let mut sphere = Object::new(
        sphere_mass,
        sphere_surface,
        control_params,
        environment,
        State::default(),
    );

    let time_max: f32 = 1800.0;

    // Starts time for simulation loop
    let mut time: f32 = 0.0;
    for counter_1 in 0..mesh.len() {
        let desired = State::new(
            DoF::new(mesh[counter_1].x, 0.0, 0.0),
            DoF::new(mesh[counter_1].y, 0.0, 0.0),
            DoF::new(mesh[counter_1].z, 0.0, 0.0),
            DoF::new(0.0, 0.0, 0.0),
            DoF::new(0.0, 0.0, 0.0),
            DoF::new(0.0, 0.0, 0.0),
        );

        while mesh[counter_1].yon != true {
            sphere.go_to(desired);
            {
                // Writes state data to position file
                let data_string = format!(
                    "{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
                    time.to_string(),
                    sphere.state.xaxis.var.to_string(),
                    sphere.state.xaxis.vard.to_string(),
                    sphere.state.xaxis.vardd.to_string(),
                    sphere.state.yaxis.var.to_string(),
                    sphere.state.yaxis.vard.to_string(),
                    sphere.state.yaxis.vardd.to_string(),
                    sphere.state.zaxis.var.to_string(),
                    sphere.state.zaxis.vard.to_string(),
                    sphere.state.zaxis.vardd.to_string(),
                    sphere.state.yaw.var.to_string(),
                    sphere.state.yaw.vard.to_string(),
                    sphere.state.yaw.vardd.to_string()
                );
                f.write_all(data_string.as_bytes())
                    .expect("Could write data string with Object State data to file");
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
