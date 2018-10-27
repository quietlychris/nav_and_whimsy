#![allow(dead_code)]

use std::f32;

use physics_lib::*;
use kinematics_lib::*;

// Since we're currently just using the location of the point to calculate score, using Point
// might be fine, but if we eventually want to use direct parameters of the Object like
// direction, velocities, etc. (if we're doing real-time self-score calcs) it might make sense
// to eventually add another list-object using the State struct
#[derive(Clone, Copy)]
pub struct ListPoint {
    pub point: Point,
    pub score: f32
}

impl ListPoint {
    pub fn calculate_score(a: Point, b: Point) -> Self {
        let x_error = b.x - a.x;
        let y_error = b.y - a.y;
        let z_error = b.z - a.z;
        let total_pos_error = (x_error.powf(2.0) + y_error.powf(2.0) + z_error.powf(2.0)).powf(0.5);

        point = ListPoint {a, total_pos_error}

    }


}
