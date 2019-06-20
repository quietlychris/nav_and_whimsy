#![allow(dead_code)]

use crate::DT;
use std::f32;

use crate::controls_lib::ControlParams;
use crate::controls_lib::*;

#[derive(Clone, Copy)]
pub struct Object {
    pub mass: f32,
    pub area: f32,
    pub environment: Environment,
    pub ctrlparams: ControlParams,
    pub state: State,
}

#[derive(Clone, Copy)]
pub struct Environment {
    pub gravity: bool,
}

impl Object {
    pub fn new(
        mass: f32,
        area: f32,
        ctrlparams: ControlParams,
        environment: Environment,
        state: State,
    ) -> Self {
        Object {
            mass: mass,
            area: area,
            ctrlparams: ctrlparams,
            environment: environment,
            state: state,
        }
    }

    pub fn kinematic_update(&mut self, xvardd: f32, yvardd: f32, zvardd: f32, yawvardd: f32) {
        // Let's not use the DoF kinematic_update() function quite yet

        self.state.xaxis.vardd = xvardd;
        self.state.xaxis.vard = self.state.xaxis.vard + self.state.xaxis.vardd * DT;
        self.state.xaxis.var = self.state.xaxis.var + self.state.xaxis.vard * DT;

        self.state.yaxis.vardd = yvardd;
        self.state.yaxis.vard = self.state.yaxis.vard + self.state.yaxis.vardd * DT;
        self.state.yaxis.var = self.state.yaxis.var + self.state.yaxis.vard * DT;

        self.state.zaxis.vardd = zvardd;
        if self.environment.gravity == true {
            self.state.zaxis.vardd = self.state.zaxis.vardd - 9.81f32;
        }
        self.state.zaxis.vard = self.state.zaxis.vard + self.state.zaxis.vardd * DT;
        self.state.zaxis.var = self.state.zaxis.var + self.state.zaxis.vard * DT;

        self.state.yaw.vardd = yawvardd;
        self.state.yaw.vard = self.state.yaw.vard + self.state.yaw.vardd * DT;
        self.state.yaw.var = self.state.yaw.var + self.state.yaw.vard * DT;

        if self.state.yaw.var > 180f32 {
            self.state.yaw.var = self.state.yaw.var - 360f32;
        }
        if self.state.yaw.var < -180f32 {
            self.state.yaw.var = self.state.yaw.var + 360f32;
        }
        // TO_DO: Error handling for this NaN requires desired, but I'm not sure I want to pass another variable to this functions
        // let desired_yaw = self.calculate_yaw(desired);
        //if self.yaw.var.is_nan() == true { println!("State::kinematic_update() : self.yaw.var.is_nan() == true"); self.yaw.var = desired_yaw;}
    }

    // Uses control laws to apply acclerations to Object (actual physics are run in the 'kinematic_update' function)
    pub fn go_to(&mut self, desired: State) {
        let desired_yaw = self.state.calculate_yaw(desired);
        let total_velocity =
            (self.state.xaxis.vard.powf(2.0) + self.state.yaxis.vard.powf(2.0)).powf(0.5);
        //println!("In main(): desired_yaw vs self.state: {:.3} {:.3}",desired_yaw,self.state.yaw.var);
        let mut acceleration: (f32, f32, f32) = (0.0, 0.0, 0.0);
        let yaw_difference = (desired_yaw - self.state.yaw.var).abs();

        // NEED THREE CASES HERE:
        // In right direction, proceed to target
        // In wrong direction, slow to zero velocity
        // At zero velocity in wrong direction, turn to target

        if yaw_difference < self.ctrlparams.yaw_allowance
            && self.state.yaw.vard.abs() < self.ctrlparams.yaw_speed_allowance
            && total_velocity < self.ctrlparams.max_speed
        {
            //println!("CASE 1: Everything's right, b/c yaw {:.3} ~ {:.3} && speed {:.3} < {:.3}", self.state.yaw.var,desired_yaw,total_velocity,self.ctrlparams.max_speed);
            acceleration = (
                self.state.get_xy_acceleration(
                    desired,
                    self.ctrlparams.position_smd,
                    self.ctrlparams.yaw_allowance,
                ),
                self.state
                    .get_z_acceleration(desired, self.ctrlparams.z_smd),
                self.state
                    .get_yaw_acceleration(desired_yaw, self.ctrlparams.yaw_smd),
            );
        } else if total_velocity < self.ctrlparams.zero_speed_allowance
            && yaw_difference > self.ctrlparams.yaw_allowance
        {
            //println!("CASE 2: Just updating direction, b/c total_v = {:.3} < {:.3} and yaw {:.3} ~ {:3} -> yaw-delta {:.3} > {:.3} ",total_velocity,self.ctrlparams.zero_speed_allowance,self.state.yaw.var,desired_yaw,yaw_difference,self.ctrlparams.yaw_allowance);
            acceleration.1 = self
                .state
                .get_z_acceleration(desired, self.ctrlparams.z_smd);
            acceleration.2 = self
                .state
                .get_yaw_acceleration(desired_yaw, self.ctrlparams.yaw_smd);
        } else {
            //println!("CASE 3: trending speed towards zero, b/c ({:.3} > {:.3})",total_velocity,self.ctrlparams.zero_speed_allowance);
            // If direction is right, and angular speed isn't too high, update direction and position
            acceleration = self.state.trend_speed_towards_zero(self.ctrlparams.yaw_smd);
            acceleration.1 = self
                .state
                .get_z_acceleration(desired, self.ctrlparams.z_smd);
        }

        // Assigns acceleration values for the various degrees of freedom
        let xvardd = acceleration.0 * self.state.yaw.var.to_radians().cos();
        let yvardd = acceleration.0 * self.state.yaw.var.to_radians().sin();
        let zvardd = acceleration.1;
        let yawvardd = acceleration.2;
        //println!("acceleration tuple: {:?},acceleration);
        self.state
            .kinematic_update(xvardd, yvardd, zvardd, yawvardd);
        //self.state.print();
    }
}

impl Environment {
    pub fn new(gravity: bool) -> Self {
        Environment { gravity: gravity }
    }
}
