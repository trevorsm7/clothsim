use cgmath::prelude::*;
use cgmath::{Point2, Vector2};

pub trait Constraint {
    fn apply_force(&self, pos: &[Point2<f32>], vel: &[Vector2<f32>], force: &mut [Vector2<f32>]);
}

#[derive(Copy, Clone, Debug)]
pub struct TensionConstraint {
    pub tension: f32,
    pub from: usize,
    pub to: usize,
}

impl TensionConstraint {
    pub fn new(tension: f32, from: usize, to: usize) -> Self {
        TensionConstraint {tension, from, to}
    }
}

impl Constraint for TensionConstraint {
    fn apply_force(&self, pos: &[Point2<f32>], _vel: &[Vector2<f32>], force: &mut[Vector2<f32>]) {
        let from = pos[self.from];
        let to = pos[self.to];
        force[self.from] += (to - from) * self.tension;
        force[self.to] += (from - to) * self.tension;
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SpringConstraint {
    pub length: f32,
    pub spring_k: f32,
    pub damper_k: f32,
    pub a: usize,
    pub b: usize,
}

impl SpringConstraint {
    pub fn new(points: &[Point2<f32>], spring_k: f32, damper_k: f32, a: usize, b: usize) -> Self {
        let length = points[a].distance(points[a]);
        SpringConstraint {length, spring_k, damper_k, a, b}
    }
}

impl Constraint for SpringConstraint {
    fn apply_force(&self, pos: &[Point2<f32>], vel: &[Vector2<f32>], force: &mut[Vector2<f32>]) {
        let a = pos[self.a];
        let b = pos[self.b];

        // Compute length and normalized direction to other particle
        let a_to_b = b - a;
        let length = a_to_b.magnitude();
        let a_to_b = a_to_b / length; // .normalize();
        let b_to_a = -a_to_b;

        // Compute component of velocity directed away from other point
        let a_vel = vel[self.a].dot(b_to_a);
        let b_vel = vel[self.b].dot(a_to_b);

        let spring_force = (length - self.length) * self.spring_k;
        let damper_force = (b_vel + a_vel) * self.damper_k;

        force[self.a] += a_to_b * (spring_force + damper_force);
        force[self.b] += b_to_a * (spring_force + damper_force);
    }
}
