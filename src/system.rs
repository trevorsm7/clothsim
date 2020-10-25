use cgmath::prelude::*;
use cgmath::{Point2, Vector2};

use image::{ImageBuffer, RgbImage, Rgb};

use super::rasterize_line;
use super::double_buffer::DoubleBuffer;
use super::constraint::Constraint;

fn to_clip_space(point: Point2<f32>, origin: Point2<f32>, size: Vector2<f32>) -> Point2<f32> {
    Point2::new((point.x - origin.x) / size.x, (point.y - origin.y) / size.y)
}

pub struct System {
    pos: DoubleBuffer<Point2<f32>>,
    vel: DoubleBuffer<Vector2<f32>>,
    force: Vec<Vector2<f32>>,
    mass: Vec<f32>,
    constraints: Vec<Constraint>,
}

impl System {
    pub fn new(pos: Vec<Point2<f32>>, mass: Vec<f32>, constraints: Vec<Constraint>) -> Self {
        let vel = DoubleBuffer::from(vec![Vector2::zero(); pos.len()]);
        let force = vec![Vector2::zero(); pos.len()];
        let pos = DoubleBuffer::from(pos);
        System {pos, vel, force, mass, constraints}
    }

    fn reset_force(&mut self) {
        for force in self.force.iter_mut() {
            *force = Vector2::zero();
        }
    }

    fn apply_force(&mut self, i: usize, force: Vector2<f32>) {
        self.force[i] += force;
    }

    pub fn step(&mut self, dt: f32) {
        let down = Vector2::new(0., 9.81); // NOTE down is positive, a hack to flip the output

        for i in 0..self.force.len() { //self.pos.front().len() {
            let mass = self.mass[i];
            if mass != 0. { // HACK using zero mass for static anchor points
                // Add gravitational acceleration first
                self.apply_force(i, down * mass);
            }
        }

        // Apply forces from each constraint
        // TODO Rust is being weird with mutable ownership here
        //for constraint in self.constraints.iter().cloned() {
        for i in 0..self.constraints.len() {
            let constraint = self.constraints[i].clone();

            let a = self.pos.read(constraint.a);
            let b = self.pos.read(constraint.b);

            // Compute length and normalized direction to other particle
            let a_to_b = b - a;
            let length = a_to_b.magnitude();
            let a_to_b = a_to_b / length; // .normalize();
            let b_to_a = -a_to_b;

            // Compute component of velocity directed away from other point
            let a_vel = self.vel.read(constraint.a).dot(b_to_a);
            let b_vel = self.vel.read(constraint.b).dot(a_to_b);

            let spring_force = (length - constraint.length) * constraint.spring_k;
            let damper_force = (b_vel + a_vel) * constraint.damper_k;

            self.apply_force(constraint.a, a_to_b * (spring_force + damper_force));
            self.apply_force(constraint.b, b_to_a * (spring_force + damper_force));
        }

        // Integrate force over each point by 1 time step
        for i in 0..self.force.len() { //self.pos.front().len() {
            if self.mass[i] == 0. { // HACK zero mass for static points
                self.pos.write(i, self.pos.read(i));
            } else {
                let next_vel = self.vel.read(i) + self.force[i] * dt / self.mass[i];
                self.pos.write(i, self.pos.read(i) + next_vel * dt);
                self.vel.write(i, next_vel);
            }
        }

        // Swap the next values from the back buffer
        self.pos.flip();
        self.vel.flip();
        self.reset_force();
    }

    /*fn find_bounds(&self) -> (Point2<f32>, Vector2<f32>) {
        let mut min = Point2::new(f32::INFINITY, f32::INFINITY);
        let mut max = Point2::new(f32::NEG_INFINITY, f32::NEG_INFINITY);
        for point in self.pos.front().iter() {
            min = Point2::new(min.x.min(point.x), min.y.min(point.y));
            max = Point2::new(max.x.max(point.x), max.y.max(point.y));
        }
        (min, max - min)
    }*/

    pub fn rasterize(&self, img: &mut RgbImage, origin: Point2<f32>, size: Vector2<f32>, color: Rgb<u8>) {
        self.pos.front().iter().zip(self.pos.front().iter().skip(1))
            .for_each(|(&start, &end)| {
                let start_clip = to_clip_space(start, origin, size);
                let end_clip = to_clip_space(end, origin, size);
                rasterize_line(img, start_clip, end_clip, color);
            });
    }

    pub fn make_rope(start: Point2<f32>, end: Point2<f32>, mass: f32, spring_k: f32, damper_k: f32, n: usize) -> Self {
        let mut points = vec![];
        let mut masses = vec![];
        let mut constraints = vec![];

        for i in 0..n {
            // TODO Why can't we lerp from Point2?
            let p = start.to_vec().lerp(end.to_vec(), i as f32 / (n - 1) as f32);
            points.push(Point2::from_vec(p));
        }

        // TODO Add anchor constraints instead of setting mass to zero
        masses.push(0.);
        for i in 1..(n - 1) {
            masses.push(mass / n as f32);
            constraints.push(Constraint::new(&points, spring_k, damper_k, i - 1, i));
            constraints.push(Constraint::new(&points, spring_k, damper_k, i, i + 1));
        }
        masses.push(0.);

        /*for i in 2..(n - 2) {
            constraints.push(Constraint::new(&points, spring_k, damper_k, i - 2, i));
            constraints.push(Constraint::new(&points, spring_k, damper_k, i, i + 2));
        }*/

        System::new(points, masses, constraints)
    }

    pub fn make_net(origin: Point2<f32>, u: Vector2<f32>, v: Vector2<f32>, mass: f32, spring_k: f32, damper_k: f32, n: usize) -> Self {
        let mut points = vec![];
        let mut masses = vec![];
        let mut constraints = vec![];

        for j in 0..n {
            for k in 0..n { // TODO use different dimension for v or change to nodes/meter?
                let j_n = j as f32 / (n - 1) as f32;
                let k_n = k as f32 / (n - 1) as f32;
                let p = origin + u * k_n + v * j_n;
                points.push(p);

                if (j == 0 || j == n - 1) && (k == 0 || k == n - 1) {
                    masses.push(0.); // HACK use zero mass at corners as anchors
                } else {
                    masses.push(mass / (n * n) as f32);
                }
            }
        }

        for j in 0..n {
            for k in 0..n {
                let i = j * n + k;
                if k > 0 {
                    constraints.push(Constraint::new(&points, spring_k, damper_k, i - 1, i));
                }
                if k < n - 1 {
                    constraints.push(Constraint::new(&points, spring_k, damper_k, i, i + 1));
                }
                if j > 1 {
                    constraints.push(Constraint::new(&points, spring_k, damper_k, i - n, i));
                }
                if j < n - 1 {
                    constraints.push(Constraint::new(&points, spring_k, damper_k, i, i + n));
                }
            }
        }

        System::new(points, masses, constraints)
    }
}
