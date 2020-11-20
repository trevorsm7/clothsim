use cgmath::prelude::*;
use cgmath::{Point2, Vector2};

use super::sim::Line;

#[derive(Copy, Clone, Debug)]
pub struct Tension(pub f32);

pub fn apply_tension(line: &Line, tension: &Tension, pos: &[Point2<f32>], force: &mut[Vector2<f32>]) {
    let &Line(a, b) = line;
    let a_to_b = (pos[b] - pos[a]).normalize();
    force[a] += a_to_b * tension.0;
    force[b] -= a_to_b * tension.0;
}

#[derive(Copy, Clone, Debug)]
pub struct Spring {
    pub length: f32,
    pub spring_k: f32,
    pub damper_k: f32,
}

pub fn apply_spring(line: &Line, spring: &Spring, pos: &[Point2<f32>], vel: &[Vector2<f32>], force: &mut[Vector2<f32>]) {
    let &Line(a, b) = line;

    // Compute length and normalized direction to other particle
    let a_to_b = pos[b] - pos[a];
    let length = a_to_b.magnitude();
    let a_to_b = a_to_b / length; // .normalize();
    let b_to_a = -a_to_b;

    // Compute component of velocity directed away from other point
    let a_vel = vel[a].dot(b_to_a);
    let b_vel = vel[b].dot(a_to_b);

    let spring_force = (length - spring.length) * spring.spring_k;
    let damper_force = (b_vel + a_vel) * spring.damper_k;

    force[a] += a_to_b * (spring_force + damper_force);
    force[b] += b_to_a * (spring_force + damper_force);
}
