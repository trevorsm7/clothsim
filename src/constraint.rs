use cgmath::prelude::*;
use cgmath::Point2;

#[derive(Copy, Clone, Debug)]
pub struct Constraint {
    pub length: f32,
    pub spring_k: f32,
    pub damper_k: f32,
    pub a: usize,
    pub b: usize,
}

impl Constraint {
    pub fn new(points: &[Point2<f32>], spring_k: f32, damper_k: f32, a: usize, b: usize) -> Self {
        let length = points[a].distance(points[a]);
        Constraint {length, spring_k, damper_k, a, b}
    }
}
