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
pub const DT: f32 = 0.1;
// 'length' is used for geneating a plotting the search mesh
pub const length: usize = 8;

fn main() {

    // Creates new log file
    let f = File::create("run_log.csv").expect("Unable to create file");
    let mut f = BufWriter::new(f);

    // Generates and plots new mesh
    let mut mesh = create_mesh();

    // Creates robot at with initial conditions
    let mut current = State::new(DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0));

    /*let mut desired = State::new(DoF::new(1.0,0.0,0.0),
                                 DoF::new(1.0,0.0,0.0),
                                 DoF::new(1.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0),
                                 DoF::new(0.0,0.0,0.0) );*/

    // Creates spring-mass-damper controllers for both yaw and 2D position
    // Remember, with too strong a spring and too small a mass, accelerations
    // may jump to high and lead to unstable behavior
    let mut position_smd = SMD::new(2.0,1.0,0.3); //TO_DO: Can these inputs be expressions?
    let mut yaw_smd = SMD::new(1.0,0.2,1.0);
    let mut pitch_smd = SMD::new(1.0,0.20,1.20);

    // Writes current position to log
    f.write_all(current.xaxis.var.to_string().as_bytes());
    f.write_all(b",");
    f.write_all(current.yaxis.var.to_string().as_bytes());
    f.write_all(b"\n");

    // Creates time variable, setting to zero intially
    let mut time: f32 = 0.0;

    // Basic search, iterating through the mesh
    for a in 1i32..(length as i32)
    {
        for b in 1i32..(length as i32)
        {
            for c in 1i32..(length as i32)
            {

                let mut desired = State::new(DoF::new((a as f32),0.0,0.0),
                                             DoF::new((b as f32),0.0,0.0),
                                             DoF::new(0.0,0.0,0.0),
                                             DoF::new(0.0,0.0,0.0),
                                             DoF::new(0.0,0.0,0.0) );

                while mesh[desired.xaxis.var as usize][desired.yaxis.var as usize][desired.zaxis.var as usize].yon != true
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
                    current.pitch.update_w_smd(calculated_pitch,pitch_smd);

                    // Transforms current position into a mesh point
                    let position_as_point: Point = current.as_mesh_point();
                    // println!("PaP: ({},{},{})",position_as_point.x,position_as_point.y,position_as_point.z);
                    // If that mesh point is in the search mesh...
                    if (position_as_point.x < length) && (position_as_point.x >= 0usize)
                        && (position_as_point.y < length) && (position_as_point.y >= 0usize)
                            && (position_as_point.z < length) && (position_as_point.z >= 0usize)
                        {
                            // Sets that search mesh point to 'true'
                            mesh[position_as_point.x][position_as_point.y][position_as_point.z] = mesh[position_as_point.x][position_as_point.y][position_as_point.z].make_true();
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
                    if time > 20000f32 {break;}
                }
            }
        }
    }
    // Prints total time to search mesh to the command line
    println!("Total time of completion is {:.2} seconds", time);
    println!("Now plotting updated mesh:");
    // Plots fully searched mesh
    //plot_mesh(mesh);
}
