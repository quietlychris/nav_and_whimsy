#![allow(dead_code)]
extern crate ansi_term;
extern crate rand;

use DT;
use std::f32;

#[derive(Clone, Copy)]
pub struct DoF {
    pub var: f32,
    pub vard: f32,
    pub vardd: f32,
}

#[derive(Clone, Copy)]
pub struct PIDSet {
    pub pre_error: f32,
    pub integral: f32,
}

#[derive(Clone, Copy)]
pub struct PIDController {
    pub input: DoF,
    pub setpoint: DoF,
    kp: f32,
    ki: f32,
    kd: f32,
    rmv: PIDSet,
}

#[derive(Clone, Copy)]
pub struct State {
    pub xaxis: DoF,
    pub yaxis: DoF,
    pub zaxis: DoF,
    pub yaw: DoF,
}

#[derive(Clone, Copy)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub yon: bool
}

#[derive(Clone, Copy)]
pub struct SMD {
    pub error: f32,
    pub m: f32,
    pub c: f32,
    pub k: f32,
} // spring-mass-damper

impl State {
    pub fn default() -> Self {
        State {
            xaxis: DoF::default(),
            yaxis: DoF::default(),
            zaxis: DoF::default(),
            yaw: DoF::default(),
        }
    }

    pub fn new(xaxis: DoF, yaxis: DoF, zaxis: DoF, yaw: DoF) -> Self {
        State {
            xaxis: xaxis,
            yaxis: yaxis,
            zaxis: zaxis,
            yaw: yaw,
        }
    }

    pub fn determine_quadrant(&self, desired: State) -> String {
        // Only to be used for yaw DoF

        let mut quadrant = "nada";
        let mut _return_key = "default_key";
        // desired in quadrant 1
        if (desired.xaxis.var > self.xaxis.var) && (desired.yaxis.var > self.yaxis.var) {
            quadrant = "q1";
        }
        // desired in quadrant 2
        if (desired.xaxis.var < self.xaxis.var) && (desired.yaxis.var > self.yaxis.var) {
            quadrant = "q2";
            // if in quadrant two, then the the increase the degree by (pi/2) or 90deg
        }
        //desired in quadrant 3
        if (desired.xaxis.var < self.xaxis.var) && (desired.yaxis.var < self.yaxis.var) {
            // if in quadrant two, then the the increase the degree by (pi) or 180deg
            quadrant = "q3";
        }
        // desired in quadrant 4
        if (desired.xaxis.var > self.xaxis.var) && (desired.yaxis.var < self.yaxis.var) {
            // if in quadrant two, then the the increase the degree by (3pi/2) or 270deg
            quadrant = "q4";
        }

        if (desired.xaxis.var == self.xaxis.var) && (desired.yaxis.var > self.yaxis.var) {
            // if to the North, then direction of travel is exactly 90 degrees
            quadrant = "north";
        }
        if (desired.xaxis.var == self.xaxis.var) && (desired.yaxis.var < self.yaxis.var) {
            // if to the South, the the direction of travel is exactly 270 degrees
            quadrant = "south";
        }
        if (desired.yaxis.var == self.yaxis.var) && (desired.xaxis.var < self.xaxis.var) {
            // if to the East, the direction of travel is exactly 0 degrees
            quadrant = "west";
        }
        if (desired.yaxis.var == self.yaxis.var) && (desired.xaxis.var > self.xaxis.var) {
            // if to the West, the direction of travel is exactly 180 degrees
            quadrant = "east";
        }

        // For debugging
        /*match quadrant {
            "q1" => println!("Quadrant one"),
            "q2" => println!("Quadrant two"),
            "q3" => println!("Quadrant three"),
            "q2" => println!("Quadrant four"),

            "north" => println!("Position is directly North"),
            "south" => println!("Position is directly South"),
            "east" => println!("Position is directly East"),
            "east" => println!("Position is directly West"),
            _ => println!("Are we going where we already are?"),
        }*/

        quadrant.to_string()
    }

    pub fn calculate_yaw(&mut self, desired: State) -> f32 {
        //if self.yaw.var.is_normal() == false {println!("current yaw is not normal! => current.yaw.var = 0");self.yaw.var = 0f32;}
        if self.xaxis.var.is_nan() == true {
            println!("calculate_yaw::xaxis.var is nan -> 0f32");
            self.xaxis.var = 0f32;
        }
        if self.yaxis.var.is_nan() == true {
            println!("calculate_yaw::yaxis.var is nan -> 0f32");
            self.yaxis.var = 0f32;
        }
        let mut desired_yaw = ((desired.yaxis.var - self.yaxis.var)
            / (desired.xaxis.var - self.xaxis.var))
            .atan()
            .to_degrees();
        //println!("calculate_yaw self values: {} {}",self.xaxis.var,self.yaxis.var);

        if desired_yaw.is_normal() == false {
            desired_yaw = (desired.yaxis.var / desired.xaxis.var).atan().to_degrees();
            //println!("calculate_yaw::desired_yaw was not normal! -> desired_yaw = {}",desired_yaw);
        }
        let quadrant = self.determine_quadrant(desired);
        match quadrant.as_ref() {
            "q2" => desired_yaw = desired_yaw + 180f32,
            "q3" => desired_yaw = desired_yaw - 180f32,
            "west" => desired_yaw = 180f32,
            _ => (),
        }

        if desired_yaw > 180f32 {
            desired_yaw = desired_yaw - 360f32;
        }
        if desired_yaw < -180f32 {
            desired_yaw = desired_yaw + 360f32;
        }
        //println!("according to calculate_yaw, the desired_yaw is {}",desired_yaw);
        desired_yaw
    }

    pub fn get_yaw_acceleration(&self, desired_yaw: f32, yaw_smd: SMD) -> f32 {
        //println!("in get_yaw_acceleration(), desired_yaw is {}",desired_yaw);
        let mut yawvardd: f32 = 0.0;
        let mut error = desired_yaw - self.yaw.var;
        if error.abs() < 180f32 {
            yawvardd = (-yaw_smd.c / yaw_smd.m) * self.yaw.vard + (yaw_smd.k / yaw_smd.m) * error;
        } else {
            if self.yaw.var >= 0f32 {
                error = 360f32 - error.abs();
            } else {
                error = error - 360f32;
            }
            yawvardd = (-yaw_smd.c / yaw_smd.m) * self.yaw.vard + (yaw_smd.k / yaw_smd.m) * error;
        }
        //yawvardd = (-yaw_smd.c/yaw_smd.m)*self.yaw.vard + (yaw_smd.k/yaw_smd.m)*error;
        if yawvardd.is_nan() == true {
            yawvardd = 0.0;
        }
        yawvardd
    }

    pub fn get_xy_acceleration(&mut self, desired: State, smd: SMD, yaw_allowance: f32) -> f32 {
        let x_error = desired.xaxis.var - self.xaxis.var;
        let y_error = desired.yaxis.var - self.yaxis.var;
        let total_pos_error = (x_error.powf(2.0) + y_error.powf(2.0)).powf(0.5);
        //println!("x_error is {}, y_error is {}, total is {}",x_error,y_error,total_pos_error);
        let total_velocity = (self.xaxis.vard.powf(2.0) + self.yaxis.vard.powf(2.0)).powf(0.5);
        //println!("x_veloc is {}, y_veloc is {}",self.xaxis.vard,self.yaxis.vard);
        //let acceleration = if total_pos_error <= slow_distance { (-smd.c/smd.m)*total_velocity }
        //    else {(-smd.c/smd.m)*total_velocity + (smd.k/smd.m)*total_pos_error  };
        let desired_yaw = self.calculate_yaw(desired);
        //println!("In get_xy_acceleration(): desired_yaw vs current: {} {}",desired_yaw,self.yaw.var);
        let mut acceleration = 0.0f32;
        // TO_DO: Not sure I like having this control logic here
        if (self.yaw.var - desired_yaw).abs() < yaw_allowance {
            acceleration = (-smd.c / smd.m) * total_velocity + (smd.k / smd.m) * total_pos_error;
        }
        acceleration
    }

    pub fn get_z_acceleration(&self, desired: State, smd: SMD) -> f32 {
        //let max_z_accel: f32 = 3.0;
        let z_error = desired.zaxis.var - self.zaxis.var;
        let mut acceleration = (-smd.c / smd.m) * (self.zaxis.vard) + (smd.k / smd.m) * z_error;
        //if acceleration > max_z_accel {acceleration = max_z_accel};
        acceleration
    }

    pub fn trend_speed_towards_zero(&mut self, yaw_smd: SMD) -> (f32, f32, f32) {
        // Defining slow-down SMD directly in this function
        let smd = SMD::new(1.0, 2.0, 0.0);
        //let yaw_smd = SMD::new(0.5,0.1,0.5);
        //TO_DO: Not the most elegant error handling for NaN values
        if self.xaxis.vard.is_nan() == true {
            println!("trend_speed_towards_zero::xaxis.var is nan -> 0.0001f32");
            self.xaxis.vard = 0.0001f32;
        }
        if self.yaxis.vard.is_nan() == true {
            println!("trend_speed_towards_zero::yaxis.var is nan -> 0.0001f32");
            self.yaxis.vard = 0.0001f32;
        }
        let total_velocity = (self.xaxis.vard.powf(2.0) + self.yaxis.vard.powf(2.0)).powf(0.5);
        //println!("current.xaxis.vard: {:.2}, current.yaxis.vard: {:.2}",self.xaxis.vard,self.yaxis.vard);
        let desired_yaw = (self.yaxis.vard / self.xaxis.vard).atan().to_degrees();
        //println!("for slow-down, desired_yaw is {:.3} and current yaw is {:.3}, leading to difference of {:.3}",desired_yaw,self.yaw.var,(desired_yaw - self.yaw.var).abs());

        let mut xy_acceleration = 0.0f32;
        let yaw_allowance: f32 = 1.0;
        //println!("for slow-down, difference between desired and current yaw is {} ",(desired_yaw - self.yaw.var).abs());
        if (desired_yaw - self.yaw.var).abs() < yaw_allowance {
            //println!("xy velocity: {:.3} {:.3} self.yaw.var = {}",self.xaxis.vard,self.yaxis.vard,self.yaw.var);
            if self.xaxis.vard >= 0f32 && self.yaxis.vard > 0f32 {
                // both positive x and y velocity
                xy_acceleration = (-smd.c / smd.m) * total_velocity;
            } else if self.xaxis.vard > 0f32 && self.yaxis.vard <= 0f32 {
                // positive x and negative y velocity
                xy_acceleration = (-smd.c / smd.m) * total_velocity;
            } else if self.xaxis.vard <= 0f32 && self.yaxis.vard <= 0f32 {
                // negative x and negative y velocity
                xy_acceleration = (smd.c / smd.m) * total_velocity;
            } else if self.xaxis.vard <= 0f32 && self.yaxis.vard >= 0f32 {
                // negative x and positive y velocity
                xy_acceleration = (smd.c / smd.m) * total_velocity;
            }
        }
        //println!("Slowing down: ({:.2},{:.2}) for velocity ({:.2},{:.2})",acceleration,self.get_yaw_acceleration(desired_yaw,yaw_smd),self.xaxis.vard,self.yaxis.vard);
        (
            xy_acceleration,
            0.0,
            self.get_yaw_acceleration(desired_yaw, yaw_smd),
        )
    }

    pub fn kinematic_update(&mut self, xvardd: f32, yvardd: f32, zvardd: f32, yawvardd: f32) {
        // Let's not use the DoF kinematic_update() function quite yet

        self.xaxis.vardd = xvardd;
        self.xaxis.vard = self.xaxis.vard + self.xaxis.vardd * DT;
        self.xaxis.var = self.xaxis.var + self.xaxis.vard * DT;

        self.yaxis.vardd = yvardd;
        self.yaxis.vard = self.yaxis.vard + self.yaxis.vardd * DT;
        self.yaxis.var = self.yaxis.var + self.yaxis.vard * DT;

        self.zaxis.vardd = zvardd;
        self.zaxis.vard = self.zaxis.vard + self.zaxis.vardd * DT;
        self.zaxis.var = self.zaxis.var + self.zaxis.vard * DT;

        self.yaw.vardd = yawvardd;
        self.yaw.vard = self.yaw.vard + self.yaw.vardd * DT;
        self.yaw.var = self.yaw.var + self.yaw.vard * DT;

        if self.yaw.var > 180f32 {
            self.yaw.var = self.yaw.var - 360f32;
        }
        if self.yaw.var < -180f32 {
            self.yaw.var = self.yaw.var + 360f32;
        }
        // TO_DO: Error handling for this NaN requires desired, but I'm not sure I want to pass another variable to this functions
        // let desired_yaw = self.calculate_yaw(desired);
        //if self.yaw.var.is_nan() == true { println!("State::kinematic_update() : self.yaw.var.is_nan() == true"); self.yaw.var = desired_yaw;}
    }

    pub fn print(&self) {
        println!("  x: {:.2} {:.2} {:.2} \n  y: {:.2} {:.2} {:.2} \n  z: {:.2} {:.2} {:.2} \nyaw: {:.2} {:.2} {:.2} ",
        self.xaxis.var,self.xaxis.vard,self.xaxis.vardd,
        self.yaxis.var,self.yaxis.vard,self.yaxis.vardd,
        self.zaxis.var,self.zaxis.vard,self.zaxis.vardd,
        self.yaw.var,self.yaw.vard,self.yaw.vardd);
    }
}

impl DoF {
    pub fn default() -> Self {
        DoF {
            var: 0.0,
            vard: 0.0,
            vardd: 0.0,
        }
    }

    pub fn new(var: f32, vard: f32, vardd: f32) -> Self {
        DoF {
            var: var,
            vard: vard,
            vardd: vardd,
        }
    }

    pub fn update_w_smd(&mut self, desired: f32, smd: SMD) {
        let error = find_error(desired, self.var);
        //println!("desired.position.x = {}, error = {}",desired.position.x,error);
        self.vardd = (-smd.c / smd.m) * self.vard + (smd.k / smd.m) * error;
        self.vard = self.vard + self.vardd * DT;
        self.var = self.var + self.vard * DT;
        //println!("")
    }

    pub fn kinematic_update(&mut self, acceleration: f32) {
        self.vardd = acceleration;
        self.vard = self.vard + self.vardd * DT;
        self.var = self.var + self.vard * DT;
    }
}

pub fn random_num() -> f32 {
    // Defines a random variable in the range [-1:1] in the form
    // f32. Can be used to define a variable.
    let value = rand::random::<f32>();
    let boolrand: bool = rand::random();
    let condition: f32 = if boolrand == true { 1.0 } else { -1.0 };
    let u = value * condition as f32;
    //let y = u as f32;
    u
}

// find_error(desired, current)
pub fn find_error(a: f32, b: f32) -> f32 {
    // Takes two arguments, then determines the difference between them
    //
    // This is pretty important for angular PID controllers, since
    // transformations from 3 to 2 degrees shouldn't be seen as moving
    // all the way around the unit circle

    if a >= 0f32 && b >= 0f32
    // CASE 1
    {
        if a > b {
            (a - b)
        } else if a < b {
            (a - b)
        } else {
            0f32
        }
    } else if a < 0f32 && b < 0f32
    // CASE 2
    {
        if a > b {
            (a - b)
        } else if a < b {
            (a - b)
        } else {
            0f32
        }
    } else if a > 0f32 && b < 0f32
    // CASE 3
    {
        if a > b {
            (a - b)
        } else if a < b {
            panic!("something's very wrong");
        } else {
            0f32
        }
    } else
    //if a < 0f32 && b > 0f32 // CASE 4
    {
        if a > b {
            panic!("something's very wrong");
        } else {
            (a - b)
        }
    }
}

pub fn sign_check(a: f32, b: f32) -> bool {
    let sign_check: bool = loop {
        if (a >= 0f32 && b >= 0f32) || (a <= 0f32 && b <= 0f32) {
            break true;
        } else {
            break false;
        }
    };
    sign_check
}

impl PIDController {
    pub fn new(kp: f32, ki: f32, kd: f32, rmv: PIDSet) -> Self {
        // Creates a new PID controller
        // Takes 4 arguments: proportional, integral, and derivative gains, plus
        // a set of two 'remembered values' that are used in each iteration of the
        // controller's logic

        PIDController {
            input: DoF::default(),
            setpoint: DoF::default(),
            kp: kp,
            ki: ki,
            kd: kd,
            rmv: rmv,
        }
    }

    pub fn default() -> Self {
        // Resets all PID controller values back to zero
        // TODO: Could be useful for doing error handling, or for gain tuning loops,
        // if the default values for the gains are set to something other than zero
        // when a value exceeds desired limits

        PIDController {
            input: DoF::default(),
            setpoint: DoF::default(),
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            rmv: PIDSet {
                pre_error: 0.0,
                integral: 0.0,
            },
        }
    }

    /*pub fn control(&mut self,mut value: Position, setpoint: Position) -> Position {

        // Primary functionality of the PID loop, takes a state (here, a position)
        // and a desired state (named setpoint)

        let error = setpoint.x - value.x;
        let integral = self.rmv.integral + (error*DT);
        let derivative = (error - self.rmv.pre_error)/DT;
        let output = (self.kp*error) + (self.ki*self.rmv.integral) + (self.kd*derivative);
        value.x = value.x + output;
        self.rmv = PIDSet {pre_error: error, integral: integral};
        value
    }*/

    pub fn control_dof(&mut self, mut value: DoF, setpoint: DoF) -> DoF {
        // Primary functionality of the PID loop, takes a state (here, a position)
        // and a desired state (named setpoint)

        let error = setpoint.var - value.var;
        let integral = self.rmv.integral + (error * DT);
        let derivative = (error - self.rmv.pre_error) / DT;
        let output = (self.kp * error) + (self.ki * self.rmv.integral) + (self.kd * derivative);
        value.var = value.var + output;
        self.rmv = PIDSet {
            pre_error: error,
            integral: integral,
        };
        value
    }
}

impl SMD {
    pub fn default() -> Self {
        SMD {
            error: 0.0,
            m: 0.0,
            c: 0.0,
            k: 0.0,
        }
    }

    pub fn new(m: f32, c: f32, k: f32) -> Self {
        SMD {
            error: 0.0,
            m: m,
            c: c,
            k: k,
        }
    }
}

impl PIDSet {
    pub fn new(pre_error: f32, integral: f32) -> Self {
        PIDSet {
            pre_error: pre_error,
            integral: integral,
        }
    }

    pub fn default() -> Self {
        PIDSet {
            pre_error: 0.0,
            integral: 0.0,
        }
    }
}

impl Point {
    // 'yon' stands for 'yes or no

    // Creates a point at the origin (0,0,0)
    pub fn default() -> Point {
        Point {
            x: 0f32,
            y: 0f32,
            z: 0f32,
            yon: false,
        }
    }

    // Writes the state of the desired point to the command line
    pub fn print(&self) {
        println!(
            "[x: {:.3}, y: {:.3}, z: {:.3}, state: {} ]",
            self.x, self.y, self.z, self.yon
        );
    }

    pub fn new(x: f32, y: f32, z: f32, yon: bool) -> Point {
        Point {
            x: x,
            y: y,
            z: z,
            yon: yon,
        }
    }

    // Changes the state of a Point to 'true'
    pub fn make_true(&mut self) {
        self.yon = true;
    }

    // Changes the state of a Point to 'false'
    pub fn make_false(&mut self) {
        self.yon = false;
    }
}
