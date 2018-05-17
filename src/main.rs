// Kinematic mesh Search file
// cmoran 2018


// Imports library module
mod kinematics_lib;
mod tests;

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
pub const DT: f32 = 0.1;
// 'length' is used for geneating a plotting the search mesh
pub const length: isize = 4;

fn main() {

    // Creates new log files
    let f = File::create("run_log.csv").expect("Unable to create file");
    let g = File::create("waypoints.csv").expect("Unable to create file");
    let mut f = BufWriter::new(f);
    let mut g = BufWriter::new(g);

    // Creates meshtor of desired Points, defaulted to un-searched
    let mut mesh: Vec<Point> = Vec::new();
    /*for z in 0isize..1isize {
        for y in -2isize..3isize {
            for x in -2isize..3isize {

                let mut point = Point::new(x as f32,y as f32,z as f32,false);
                mesh.push(point);

            }
        }
    }*/

    // Test points that can replace the for-loop generated ones
    mesh.push(Point::new(4.0,-4.0,1.0,false));
    mesh.push(Point::new(2.0,-6.0,0.50,false));
    mesh.push(Point::new(-6.0,-4.0,-1.0,false));
    mesh.push(Point::new(4.0,3.0,0.50,false));
    //mesh.push(Point::new(2.5,2.5,0.0,false));


    // Creates robot at with initial conditions
    let mut current_global = State::new(DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0));


    // Creates spring-mass-damper controllers for both yaw and 2D position
    // Remember, with too strong a spring and too small a mass, accelerations
    // may jump to high and lead to unstable behavior
    let mut position_smd = SMD::new(1.3,0.3,0.2);
    let mut yaw_smd = SMD::new(0.1,0.1,2.5);
    let mut pitch_smd = SMD::new(1.0,0.7,1.0);

    // Writes current position to log
    f.write_all(current_global.xaxis.var.to_string().as_bytes());
    f.write_all(b",");
    f.write_all(current_global.yaxis.var.to_string().as_bytes());
    f.write_all(b"\n");


    // Creates time variable, setting to zero intially
    let mut time: f32 = 0.0;
    let mut cumulative_error = 0.0f32;
    let mut cumulative_z_error = 0.0f32;

    for counter_1 in 0..mesh.len() {

        let mut desired_global = State::new(DoF::new(mesh[counter_1].x,0.0,0.0),
                                            DoF::new(mesh[counter_1].y,0.0,0.0),
                                            DoF::new(mesh[counter_1].z,0.0,0.0),
                                            DoF::new(0.0,0.0,0.0),
                                            DoF::new(0.0,0.0,0.0) );

                // Puts waypoints into log file
        g.write_all(desired_global.xaxis.var.to_string().as_bytes());
        g.write_all(b",");
        g.write_all(desired_global.yaxis.var.to_string().as_bytes());
        g.write_all(b",");
        g.write_all(desired_global.zaxis.var.to_string().as_bytes());
        g.write_all(b"\n");

        //current.print_state(desired);

        while mesh[counter_1].yon != true
        {
            // Need to create a local reference frame
            let mut desired_local = State::new(DoF::new(desired_global.xaxis.var-current_global.xaxis.var,0.0,0.0),
                                               DoF::new(desired_global.yaxis.var-current_global.yaxis.var,0.0,0.0),
                                               DoF::default(),
                                               DoF::default(),
                                               DoF::default()
                                           );
            let mut current_local = State::new(DoF::default(),
                                               DoF::default(),
                                               DoF::default(),
                                               current_global.yaw,
                                               DoF::default() );
            let quadrant = current_local.determine_quadrant(desired_local);
            let desired_yaw = current_local.calculate_yaw(desired_local,quadrant);
            current_local.yaw.update_w_smd(desired_yaw,yaw_smd); // This should create the appropriate angular acceleration
            //print!("Desired yaw: {} ",desired_yaw);
            current_global.yaw.vardd = current_local.yaw.vardd;
            current_global.yaw.vard = current_global.yaw.vard + current_global.yaw.vardd*DT;
            current_global.yaw.var = current_global.yaw.var + current_global.yaw.vard*DT;
            //println!(" current_global yaw: {}",current_global.yaw.var);

            let calculated_pitch = current_global.calculate_pitch(desired_global);
            //println!("desired pitch is: {}",calculated_pitch);
            current_global.pitch.update_w_smd(calculated_pitch,pitch_smd);
            //current.print_state(desired);

            current_global.update_pos_smd(desired_global,position_smd);

            let x_error = find_error(desired_global.xaxis.var,current_global.xaxis.var);
            let y_error = find_error(desired_global.yaxis.var,current_global.yaxis.var);
            let z_error = find_error(desired_global.zaxis.var,current_global.zaxis.var);
            let total_pos_error = (x_error.powf(2.0) + y_error.powf(2.0) + z_error.powf(2.0)).powf(0.5);
            if total_pos_error > 50f32
            {
                println!("Total pos error: {}",total_pos_error);
                for a in 0..mesh.len() {
                    mesh[a].print();
                }
                panic!("too far out of bounds");
            }
            println!("x: {:.2} y: {:.2} z: {:.2}",current_global.xaxis.var,current_global.yaxis.var,current_global.zaxis.var);
            let min_distance: f32 = 0.3;
            for counter_2 in 0..mesh.len()
            {

                if find_error(current_global.xaxis.var,mesh[counter_2].x).abs() < min_distance
                    && find_error(current_global.yaxis.var,mesh[counter_2].y).abs() < min_distance
                        && find_error(current_global.zaxis.var,mesh[counter_2].z).abs() < min_distance
                            && mesh[counter_2].yon!=true
                            {
                                let x_error = find_error(current_global.xaxis.var,mesh[counter_2].x).abs();
                                let y_error = find_error(current_global.yaxis.var,mesh[counter_2].y).abs();
                                let z_error = find_error(current_global.zaxis.var,mesh[counter_2].z).abs();
                                println!("errors: x: {}, y: {}, z: {}",x_error,y_error,z_error);
                                current_global.print_state(desired_global);
                                mesh[counter_2].make_true();

                            }

            }


                // Writes new position to log

                f.write_all(current_global.xaxis.var.to_string().as_bytes());
                f.write_all(b",");
                f.write_all(current_global.yaxis.var.to_string().as_bytes());
                f.write_all(b",");
                f.write_all(current_global.zaxis.var.to_string().as_bytes());
                f.write_all(b"\n");

                // Increases total time
                time = time + DT;
                if time > 200f32 {break;}
                cumulative_error = cumulative_error + total_pos_error;
                cumulative_z_error = cumulative_z_error + z_error.abs();
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
    if time < 10000f32 {
        println!("Total time of completion is {:.2} seconds",time);
        let norm_error = cumulative_error/(time/DT);
        println!("Normalized error for this run: {}",norm_error);
        println!("Normalized z-error: {}",cumulative_z_error/(time/DT));
    }
    //println!("Now plotting updated mesh:");
    // Plots mesh updated with searched points
    for a in 0..mesh.len() {
        mesh[a].print();
    }
}
