#![allow(dead_code)]

use DT;
use std::f32;

use kinematics_lib::*;

#[derive(Clone, Copy)]
pub struct Object {
    pub mass: f32,
    pub area: f32,
    pub environment: Environment,
    pub state: State,
}

#[derive(Clone, Copy)]
pub struct Environment {
    pub gravity: bool,
}

impl Object {
    pub fn new(mass: f32, area: f32, environment: Environment, state: State) -> Self {
        Object {
            mass: mass,
            area: area,
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
}

impl Environment {
    pub fn new(gravity: bool) -> Self {
        Environment { gravity: gravity }
    }
}
