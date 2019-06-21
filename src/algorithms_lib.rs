#![allow(dead_code)]

use std::f32;

use crate::controls_lib::*;

// Since we're currently just using the location of the point to calculate score, using Point
// might be fine, but if we eventually want to use direct parameters of the Object like
// direction, velocities, etc. (if we're doing real-time self-score calcs) it might make sense
// to eventually add another list-object using the State struct
#[derive(Clone, Copy)]
pub struct ListPoint {
    pub point: Point,
    pub score: f32,
}

impl ListPoint {
    pub fn new(point: Point, score: f32) -> Self {
        ListPoint {
            point: point,
            score: score,
        }
    }

    // Seems like a fairly simple way of instituting a greedy path-finding algorithm, but not sure it's really put in the
    // right place at the moment
    pub fn calculate_score(a: Point, b: Point) -> Self {
        let x_error = b.x - a.x;
        let y_error = b.y - a.y;
        let z_error = b.z - a.z;
        let total_pos_error = (x_error.powf(2.0) + y_error.powf(2.0) + z_error.powf(2.0)).powf(0.5);

        let point = ListPoint::new(a, total_pos_error);
        point
    }
}
