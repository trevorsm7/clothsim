use cgmath::prelude::*;
use cgmath::{Point2, Vector2};

use super::sim::Line;

#[derive(Copy, Clone, Debug)]
pub struct Tension(pub u32);

pub fn apply_tension(line: &Line, tension: &Tension, var: &[f32], pos: &[Point2<f32>], force: &mut[Vector2<f32>]) {
    let &Line(a, b) = line;
    let tension = var[tension.0 as usize];
    let a_to_b = (pos[b] - pos[a]).normalize();
    force[a] += a_to_b * tension;
    force[b] -= a_to_b * tension;
}

#[derive(Copy, Clone, Debug)]
pub struct Spring {
    pub length: f32,
    pub spring_var: u32,
}

pub fn apply_spring(line: &Line, spring: &Spring, var: &[f32], pos: &[Point2<f32>], force: &mut[Vector2<f32>]) {
    let &Line(a, b) = line;
    let spring_k = var[spring.spring_var as usize];

    // Compute length and normalized direction to other particle
    let a_to_b = pos[b] - pos[a];
    let length = a_to_b.magnitude();
    let a_to_b = a_to_b / length; // .normalize();

    let spring_force = a_to_b * ((length - spring.length) * spring_k);
    force[a] += spring_force;
    force[b] -= spring_force;
}
