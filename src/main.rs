// Kinematic mesh Search file
// cmoran 2018


// Imports library module
mod kinematics_lib;

// Pulls public functions and custom data types in from library
use kinematics_lib::*;
use kinematics_lib::SMD;
use kinematics_lib::DoF;
use kinematics_lib::State;
use std::f32;
//for writing to log file
use std::fs::File;
use std::io::{Write, BufWriter};

// DT is the kinematic time step for state integration
pub const DT: f32 = 0.05;
// 'length' is used for geneating a plotting the search mesh
pub const length: isize = 4;

fn main() {

    // Creates new log file
    let f = File::create("run_log.csv").expect("Unable to create file");
    let mut f = BufWriter::new(f);

    // Creates meshtor of desired Points, defaulted to un-searched
    let mut mesh: Vec<Point> = Vec::new();
    for z in 0isize..2isize {
        for y in -5isize..6isize {
            for x in -5isize..6isize {

                let mut point = Point::new(x as f32,y as f32,z as f32,false);
                mesh.push(point);

            }
        }
    }

    // Test points that can replace the for-loop generated ones
    /*mesh.push(Point::new(4.0,-4.0,-1.0,false));
    mesh.push(Point::new(2.0,-6.0,3.0,false));
    mesh.push(Point::new(-6.0,-4.0,-7.0,false));
    mesh.push(Point::new(4.0,3.0,-1.0,false));*/


    // Creates robot at with initial conditions
    let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0));


    // Creates spring-mass-damper controllers for both yaw and 2D position
    // Remember, with too strong a spring and too small a mass, accelerations
    // may jump to high and lead to unstable behavior
    let mut position_smd = SMD::new(2.0,0.2,0.3);
    let mut yaw_smd = SMD::new(0.2,0.2,3.0);
    let mut pitch_smd = SMD::new(1.0,0.2,1.0);

    // Writes current position to log
    f.write_all(current.xaxis.var.to_string().as_bytes());
    f.write_all(b",");
    f.write_all(current.yaxis.var.to_string().as_bytes());
    f.write_all(b"\n");


    // Creates time variable, setting to zero intially
    let mut time: f32 = 0.0;

    for counter_1 in 0..mesh.len() {

        let mut desired = State::new(DoF::new(mesh[counter_1].x,0.0,0.0),
                                     DoF::new(mesh[counter_1].y,0.0,0.0),
                                     DoF::new(mesh[counter_1].z,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0),
                                     DoF::new(0.0,0.0,0.0) );
        //current.print_state(desired);

        while mesh[counter_1].yon != true
        {

            let quadrant = current.determine_quadrant(desired);
            // Calculates required yaw
            current.yaw.var = current.calculate_yaw(desired,quadrant);
            // Sets calculated yaw to a variable, to satisfy borrow checker
            let calculated_yaw = current.yaw.var;
            // Updates yaw variable
            current.yaw.update_w_smd(calculated_yaw,yaw_smd);
            // Updates positional variables
            current.update_pos_smd(desired,position_smd);
            let calculated_pitch = current.calculate_pitch(desired);
            //println!("desired pitch is: {}",calculated_pitch);
            current.pitch.update_w_smd(calculated_pitch,pitch_smd);
            //current.print_state(desired);

            let min_distance: f32 = 0.015;
            for counter_2 in 0..mesh.len()
            {

                if find_error(current.xaxis.var,mesh[counter_2].x).abs() < min_distance
                    && find_error(current.yaxis.var,mesh[counter_2].y).abs() < min_distance
                        && find_error(current.zaxis.var,mesh[counter_2].z).abs() < min_distance
                            && mesh[counter_2].yon!=false
                            {
                                let x_error = find_error(current.xaxis.var,mesh[counter_2].x).abs();
                                let y_error = find_error(current.yaxis.var,mesh[counter_2].y).abs();
                                let z_error = find_error(current.zaxis.var,mesh[counter_2].z).abs();
                                //println!("errors: x: {}, y: {}, z: {}",x_error,y_error,z_error);
                                current.print_state(desired);
                                mesh[counter_2].make_true();

                            }

            }


                // Writes new position to log

                f.write_all(current.xaxis.var.to_string().as_bytes());
                f.write_all(b",");
                f.write_all(current.yaxis.var.to_string().as_bytes());
                f.write_all(b",");
                f.write_all(current.zaxis.var.to_string().as_bytes());
                f.write_all(b"\n");

                // Increases total time
                time = time + DT;
                //if time > 4000f32 {break;}
        }

        /*
        for counter_3 in 0..mesh.len()
        {
            // TO_DO: Should be replaced with idiomatic Rust iterator
            let mut counter_4 = 0isize;
            if mesh[counter_3].yon==true {counter_4 = counter_4 + 1isize;}
            if counter_4 == (mesh.len() as isize) {break;}
        }*/

    }

    // Prints total time to search mesh to the command line
    if time < 10000f32 {println!("Total time of completion is {:.2} seconds",time);}


    //println!("Now plotting updated mesh:");
    // Plots fully searched mesh
    for a in 0..mesh.len() {
        mesh[a].print();
    }
}
