#![allow(dead_code)]

// importing external libraries
extern crate rand;
extern crate ansi_term;

use self::ansi_term::Color::Red;
use self::ansi_term::Color::Green;

// Pulling in public constants from main.rs
use DT;
use length;
use std::f32;

// Degree of Freedom data type, has a variable, and that variable first and
// second derivative (ex. position, velocity, acceleration)
#[derive(Clone,Copy)]
pub struct DoF {pub var: f32, pub vard: f32, pub vardd: f32 }

// Memory data type for PID controller implementation
#[derive(Clone, Copy)]
pub struct PIDSet {pub pre_error: f32, pub integral: f32}

// Implementation of a 1D PID controller -- unused as of 3/15/18
#[derive(Clone, Copy)]
pub struct PIDController {pub input: DoF, pub setpoint: DoF, kp: f32, ki: f32, kd: f32, rmv: PIDSet}

// Compound data type for all degrees of freedom
// Used to define the robot's exact state
#[derive(Clone, Copy)]
pub struct State {pub xaxis: DoF, pub yaxis: DoF, pub zaxis: DoF, pub yaw: DoF, pub pitch: DoF}

// Spring-mass-damper, with mass m, viscous damping coefficient c, spring
// constant k,
#[derive(Clone, Copy)]
pub struct SMD {pub error: f32, m: f32, c: f32, k: f32} // spring-mass-damper

// Point is the basic element of the virtual search grid, containing several axes
#[derive(Clone,Copy)]
pub struct Point {pub x: usize, pub y: usize, pub z: usize, pub yon: bool }


impl State {

    // default() and new() are basic type constructors
    pub fn default() -> Self {
        State {xaxis: DoF::default(), yaxis: DoF::default(),
               zaxis: DoF::default(), yaw: DoF::default(),
               pitch: DoF::default() }
    }

    pub fn new(xaxis: DoF, yaxis: DoF, zaxis: DoF, yaw: DoF, pitch: DoF) -> Self {
        State {xaxis: xaxis, yaxis: yaxis,
               zaxis: zaxis, yaw: yaw, pitch: pitch }
    }

    // Determines the 3D quadrant of the desired position, with the current
    // position as the origin
    // TO_DO: Convert to 3D
    pub fn determine_quadrant(&self, desired: State) -> usize {
        // Only to be used for yaw DoF

        let mut quadrant = 0;
        let mut _return_key = 1;
        if (desired.xaxis.var > self.xaxis.var) && (desired.yaxis.var > self.yaxis.var) {
            quadrant = 1;
        }
        if (desired.xaxis.var < self.xaxis.var) && (desired.yaxis.var > self.yaxis.var) {
            quadrant = 2;
            // if in quadrant two, then the the increase the degree by (pi/2) or 90deg
        }
        if (desired.xaxis.var < self.xaxis.var) && (desired.yaxis.var < self.yaxis.var) {
            // if in quadrant two, then the the increase the degree by (pi) or 180deg
            quadrant = 3;
        }
        if (desired.xaxis.var > self.xaxis.var) && (desired.yaxis.var < self.yaxis.var) {
            // if in quadrant two, then the the increase the degree by (3pi/2) or 270deg
            quadrant = 4;
        }

        if (desired.xaxis.var == self.xaxis.var) && (desired.yaxis.var > self.yaxis.var) {
            // if to the North, then direction of travel is exactly 90 degrees
            quadrant = 11;
        }
        if (desired.xaxis.var == self.xaxis.var) && (desired.yaxis.var < self.yaxis.var) {
            // if to the South, the the direction of travel is exactly 270 degrees
            quadrant = 22;
        }
        if (desired.yaxis.var == self.yaxis.var) && (desired.xaxis.var < self.xaxis.var) {
            // if to the East, the direction of travel is exactly 0 degrees
            quadrant = 33;
        }
        if (desired.yaxis.var == self.yaxis.var) && (desired.xaxis.var > self.xaxis.var) {
            // if to the West, the direction of travel is exactly 180 degrees
            quadrant = 44;
        }


        // For debugging
        /*match quadrant {
            1 => println!("Quadrant one"),
            2 => println!("Quadrant two"),
            3 => println!("Quadrant three"),
            4 => println!("Quadrant four"),

            11 => println!("Position is directly North"),
            22 => println!("Position is directly South"),
            33 => println!("Position is directly East"),
            44 => println!("Position is directly West"),
            _ => println!("Are we going where we already are?"),
        }*/

        quadrant
    }

    // Updates the State variable using a SMD controller
    pub fn update_pos_smd(&mut self, desired: State, smd: SMD) {

        let x_error = find_error(desired.xaxis.var,self.xaxis.var);
        //let x_error = desired.xaxis.var - self.xaxis.var;
        let y_error = find_error(desired.yaxis.var,self.yaxis.var);
        //let y_error = desired.yaxis.var - self.yaxis.var;
        let z_error = find_error(desired.zaxis.var,self.zaxis.var);
        let total_pos_error = (x_error.powf(2.0) + y_error.powf(2.0) + z_error.powf(2.0)).powf(0.5);
        //println!("x_error is {}, y_error is {}, total is {}",x_error,y_error,total_pos_error);
        let total_velocity = (self.xaxis.vard.powf(2.0) + self.yaxis.vard.powf(2.0)).powf(0.5) + self.zaxis.vard.powf(2.0);
        //println!("x_veloc is {}, y_veloc is {}",self.xaxis.vard,self.yaxis.vard);
        let acceleration = (-smd.c/smd.m)*total_velocity + (smd.k/smd.m)*total_pos_error;
        //println!("acceleration is {}",acceleration);
        self.xaxis.vardd = acceleration*self.yaw.var.to_radians().cos();
        self.xaxis.vard = self.xaxis.vard + self.xaxis.vardd*DT;
        self.xaxis.var = self.xaxis.var + self.xaxis.vard*DT;

        self.yaxis.vardd = acceleration*self.yaw.var.to_radians().sin();
        //println!("yaw {}",self.yaw.var);
        self.yaxis.vard = self.yaxis.vard + self.yaxis.vardd*DT;
        self.yaxis.var = self.yaxis.var + self.yaxis.vard*DT;
        //println!("x-accel is {}, y accel is {}",self.xaxis.vardd,self.yaxis.vardd);

        //TO_DO: Just broke this by adding a 'pitch variable' cmoran, 4/12/18
        self.zaxis.vardd = acceleration*self.pitch.var.to_radians().cos();
        self.zaxis.vard = self.zaxis.vard + self.zaxis.vardd*DT;
        self.zaxis.var = self.zaxis.var + self.zaxis.vard*DT;
    }

    // Calculates the desired yaw (in absolute, not relative) required to move
    // from one point to another
    pub fn calculate_yaw(&mut self, desired: State, quadrant: usize) -> f32 {
        let mut yaw = ((desired.yaxis.var - self.yaxis.var)/(desired.xaxis.var - self.xaxis.var)).atan().to_degrees();
        let mut additive = 0f32;
        match quadrant {
            1 => additive = 0f32,
            2 => additive = 180f32,
            3 => additive = 180f32,
            4 => additive = 360f32,

            11 => self.yaxis.var = 90f32,
            22 => self.yaxis.var = 270f32,
            33 => self.yaxis.var = 180f32,
            44 => self.yaxis.var = 0f32,
            _ => println!("error grid_search_smd_lib::State::calculate_yaw function"),
        }
        //println!("yaw before additive: {}, with additive {}",self.yaxis.varaw,additive);
        yaw = yaw + additive;
        //println!("yaw after additive: {}",self.yaxis.varaw);
        yaw
    }

    pub fn calculate_pitch(&mut self, desired: State) {
        let above = if (self.zaxis.var > desired.zaxis.var) {
            true
            }
        else {false};
        println!("{}",above);

    }

    // Creates a search grid point based on the existing State
    pub fn as_mesh_point(&self) -> Point {
        Point {x: (self.xaxis.var.round() as usize), y: (self.yaxis.var.round() as usize),
               z: (self.zaxis.var.round() as usize), yon: false }
    }

}

impl DoF {

    pub fn default() -> Self {
        DoF {var: 0.0, vard: 0.0, vardd: 0.0}
    }

    pub fn new(var: f32, vard: f32, vardd: f32) -> Self {
        DoF {var: var, vard: vard, vardd: vardd}
    }

    // One-dimensional update of a DoF using an SMD controller
    pub fn update_w_smd(&mut self, desired: f32, smd: SMD) {

        let error = find_error(desired,self.var);
        //println!("desired.position.x = {}, error = {}",desired.position.x,error);
        self.vardd = (-smd.c/smd.m)*self.vard + (smd.k/smd.m)*error;
        self.vard = self.vard + self.vardd*DT;
        self.var = self.var + self.vard*DT;
    }

}

// generates a random number between [-1,1]
pub fn random_num() -> f32{
    // Defines a random variable in the range [-1:1] in the form
    // f32. Can be used to define a variable.
    let value = rand::random::<f32>();
    let boolrand: bool = rand::random();
    let condition: f32 = if boolrand == true { 1.0 } else { -1.0 };
    let u = value*condition as f32;
    //let y = u as f32;
    u
}

// Determines the closest difference between two inputs
// Particularly useful for angular distances
// find_error(desired, current)
pub fn find_error(a: f32, b: f32) -> f32 {

        // Takes two arguments, then determines the difference between them
        //
        // This is pretty important for angular PID controllers, since
        // transformations from 3 to 2 degrees shouldn't be seen as moving
        // all the way around the unit circle

        if a >= 0f32 && b >= 0f32 // CASE 1
        {
            if a > b { (a - b) }
            else if a < b { (a - b) }
            else { 0f32 }
        }
        else if a < 0f32 && b < 0f32 // CASE 2
        {
            if a > b { (a - b) }
            else if a < b { (a - b) }
            else { 0f32 }
        }
        else if a > 0f32 && b < 0f32 // CASE 3
        {
            if a > b { (a - b) }
            else if a < b { panic!("something's very wrong"); }
            else { 0f32 }
        }
        else if a < 0f32 && b > 0f32 // CASE 4
        {
            if a > b { panic!("something's very wrong");}
            else if a < b { (a - b) }
            else { 0f32 }
        }
        else {
            println!("hmm, either we're at t=0, or an unexpected condition has appeared (a | b == 0)");
            0f32
        }
}


// Used for debugging, for making sure that the signs of variables that should
// be tied to one another are actually the same
pub fn sign_check(a: f32, b: f32) -> bool {
    let sign_check: bool = loop
    {
        if (a >= 0f32 && b >= 0f32) || (a <= 0f32 && b <= 0f32)
        {
            break true
        }
        else {
            break false
        }
    };
    sign_check
}

impl PIDController {


    pub fn new(kp: f32, ki: f32, kd: f32, rmv:PIDSet) -> Self {

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
            rmv: rmv
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
            rmv: PIDSet {pre_error: 0.0, integral: 0.0}
        }
    }

    // Primary functionality of the PID loop, takes a state (here, a position)
    // and a desired state (named setpoint)
    pub fn control_dof(&mut self,mut value: DoF, setpoint: DoF) -> DoF {

        // Primary functionality of the PID loop, takes a state (here, a position)
        // and a desired state (named setpoint)

        let error = setpoint.var - value.var;
        let integral = self.rmv.integral + (error*DT);
        let derivative = (error - self.rmv.pre_error)/DT;
        let output = (self.kp*error) + (self.ki*self.rmv.integral) + (self.kd*derivative);
        value.var = value.var + output;
        self.rmv = PIDSet {pre_error: error, integral: integral};
        value
    }

}

impl SMD {

    pub fn default() -> Self {
        SMD {error: 0.0, m: 0.0, c: 0.0, k: 0.0}
    }

    pub fn new(m: f32, c: f32, k:f32 ) -> Self {
        SMD {error: 0.0, m: m, c: c, k: k}
    }

}

impl PIDSet {

    pub fn new(pre_error: f32, integral: f32) -> Self {
        PIDSet {pre_error: pre_error, integral: integral}
    }

    pub fn default() -> Self {
        PIDSet {pre_error: 0.0, integral: 0.0}
    }

}

impl Point {
    // TO_DO: Might want to create the basic constructors and editor methods
    // 'yon' stands for 'yes or no

    pub fn origin() -> Point {
        Point { x: 0usize, y: 0usize, z:0usize, yon: false }
    }

    // Writes the state of the desired point to the command line
    pub fn print(&self) {
        println!("[x:{},y:{},z:{},state:{}]",self.x,self.y,self.z,self.yon);
    }

    pub fn new(x: usize, y: usize, z: usize, yon: bool) -> Point {
        Point { x: x, y: y, z: z, yon: false }
    }

    // Changes the state of a Point to 'true'
    pub fn make_true(&self) -> Self {
        Point {x: self.x, y: self.y, z: self.z, yon: true }
    }

    // Changes the state of a Point to 'false'
    pub fn make_false(&self) -> Self {
        Point {x: self.x, y: self.y, z: self.z, yon: false }
    }

}

// creates a virtual search grid, based on the 'length' variable defined in
// main.rs. The default setting of all points is as 'false'
pub fn create_mesh() -> [[[Point; length]; length];length] {
    let mut state = [[[Point {x: 0usize, y: 0usize, z: 0usize, yon: false}; length]; length]; length] ;
    for x in 0..length
    {
        for y in 0..length
        {
            for z in 0..length
            {
                let mut point = Point::new(x,y,z,false);
                state[x][y][z] = point;
            }
        }
    }
    state
}
