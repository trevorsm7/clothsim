use super::double_buffer::DoubleBuffer;
use super::constraint::{Constraint, SpringConstraint, TensionConstraint};

use cgmath::prelude::*;
use cgmath::{Point2, Vector2};

fn to_clip_space(point: Point2<f32>, origin: Point2<f32>, size: Vector2<f32>) -> Point2<f32> {
    Point2::new((point.x - origin.x) / size.x, (point.y - origin.y) / size.y)
}

pub enum Node {
    Point(Point2<f32>),
    Index(usize),
}

pub struct System {
    pos: DoubleBuffer<Point2<f32>>,
    vel: DoubleBuffer<Vector2<f32>>,
    force: Vec<Vector2<f32>>,
    mass: Vec<f32>,
    constraints: Vec<SpringConstraint>,
    tensions: Vec<TensionConstraint>,
    enable_g: bool,
}

impl System {
    pub fn new(pos: Vec<Point2<f32>>, mass: Vec<f32>, constraints: Vec<SpringConstraint>, tensions: Vec<TensionConstraint>, enable_g: bool) -> Self {
        let vel = DoubleBuffer::from(vec![Vector2::zero(); pos.len()]);
        let force = vec![Vector2::zero(); pos.len()];
        let pos = DoubleBuffer::from(pos);
        System {pos, vel, force, mass, constraints, tensions, enable_g}
    }

    fn reset_force(&mut self) {
        for force in self.force.iter_mut() {
            *force = Vector2::zero();
        }
    }

    pub fn step(&mut self, dt: f32) {
        let down = Vector2::new(0., 9.81); // NOTE down is positive, a hack to flip the output

        if self.enable_g {
            for i in 0..self.force.len() {
                let mass = self.mass[i];
                if mass != 0. { // HACK using zero mass for static anchor points
                    // Add gravitational acceleration first
                    self.force[i] += down * mass;
                }
            }
        }

        // Apply forces from each constraint
        for constraint in self.constraints.iter().map(|s| s as &dyn Constraint)
                .chain(self.tensions.iter().map(|s| s as &dyn Constraint)) {
            constraint.apply_force(&self.pos, &self.vel, &mut self.force);
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

    pub fn rasterize<F: FnMut(Point2<f32>, Point2<f32>)>(&self, origin: Point2<f32>, size: Vector2<f32>, mut draw: F) {
        self.constraints.iter()
            .for_each(|constraint| {
                let start = self.pos.read(constraint.a);
                let end = self.pos.read(constraint.b);
                let start_clip = to_clip_space(start, origin, size);
                let end_clip = to_clip_space(end, origin, size);
                draw(start_clip, end_clip);
            });
        self.tensions.iter()
            .for_each(|tension| {
                let start = self.pos.read(tension.from);
                let end = self.pos.read(tension.to);
                let start_clip = to_clip_space(start, origin, size);
                let end_clip = to_clip_space(end, origin, size);
                draw(start_clip, end_clip);
            });
    }

    fn push_point(&mut self, point: Point2<f32>, mass: f32) -> usize {
        self.pos.push(point);
        self.vel.push(Vector2::zero());
        self.force.push(Vector2::zero());
        self.mass.push(mass);
        self.pos.len() - 1
    }

    fn node_to_index(&mut self, node: Node, mass: f32) -> usize {
        match node {
            Node::Point(point) => self.push_point(point, mass),
            Node::Index(index) => index,
        }
    }

    pub fn make_rope(&mut self, start: Node, end: Node, mass: f32, spring_k: f32, damper_k: f32, n: usize) -> Vec<usize> {
        let start_idx = self.node_to_index(start, mass / n as f32);
        let end_idx = self.node_to_index(end, mass / n as f32);

        // Lerp requires vector, not point
        let start = self.pos.read(start_idx).to_vec();
        let end = self.pos.read(end_idx).to_vec();

        let mut indices = Vec::new();
        indices.push(start_idx);
        for i in 1..n-1 {
            let point = Point2::from_vec(start.lerp(end, i as f32 / (n - 1) as f32));
            indices.push(self.push_point(point, mass / n as f32));
        }
        indices.push(end_idx);

        for (&a, &b) in indices.iter().zip(indices.iter().skip(1)) {
            self.constraints.push(SpringConstraint::new(self.pos.front(), spring_k, damper_k, a, b));
        }

        indices
    }

    pub fn make_rope_sys(start: Point2<f32>, end: Point2<f32>, mass: f32, spring_k: f32, damper_k: f32, n: usize) -> Self {
        let mut system = System::new(vec![], vec![], vec![], vec![], true);
        let start_idx = system.push_point(start, 0.);
        let end_idx = system.push_point(end, 0.);
        system.make_rope(Node::Index(start_idx), Node::Index(end_idx), mass, spring_k, damper_k, n);
        system
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
                    constraints.push(SpringConstraint::new(&points, spring_k, damper_k, i - 1, i));
                }
                if k < n - 1 {
                    constraints.push(SpringConstraint::new(&points, spring_k, damper_k, i, i + 1));
                }
                if j > 1 {
                    constraints.push(SpringConstraint::new(&points, spring_k, damper_k, i - n, i));
                }
                if j < n - 1 {
                    constraints.push(SpringConstraint::new(&points, spring_k, damper_k, i, i + n));
                }
            }
        }

        System::new(points, masses, constraints, vec![], true)
    }

    pub fn make_rig(width: i32, tension: f32, spring_k: f32, damper_k: f32) -> Self {
        let mut pos = vec![];
        let mut mass = vec![];
        let mut constraints = vec![];
        let mut tensions = vec![];

        pos.push(Point2::new((-width/2) as f32, (width/2) as f32));
        mass.push(0.);
        for i in 1 .. width {
            let x = i as f32 - 0.5 * width as f32;
            let y = x.abs();
            pos.push(Point2::new(x, y));
            mass.push(if x == 0. {0.} else {1.}); // Middle anchor point!

            let y = x * x / width as f32 + (0.25 * width as f32);
            pos.push(Point2::new(x, y));
            mass.push(1.);

            tensions.push(TensionConstraint::new(tension, pos.len() - 1, pos.len() - 2));
        }
        pos.push(Point2::new((width/2) as f32, (width/2) as f32));
        mass.push(0.);

        constraints.push(SpringConstraint::new(&pos, spring_k, damper_k, 0, 1));
        constraints.push(SpringConstraint::new(&pos, spring_k, damper_k, 0, 2));
        for i in 0 .. width-2 {
            let i = (2 * i + 1) as usize;
            if width % 2 == 0 || i != (width-2) as usize {
                constraints.push(SpringConstraint::new(&pos, spring_k, damper_k, i, i+2));
            } else {
                tensions.push(TensionConstraint::new(tension, i, i+2));
            }
            constraints.push(SpringConstraint::new(&pos, spring_k, damper_k, i+1, i+3));
        }
        constraints.push(SpringConstraint::new(&pos, spring_k, damper_k, pos.len()-1, pos.len()-2));
        constraints.push(SpringConstraint::new(&pos, spring_k, damper_k, pos.len()-1, pos.len()-3));

        // Middle anchor point
        if width % 2 == 1 {
            pos.push(Point2::new(0., 0.));
            mass.push(0.);
            constraints.push(SpringConstraint::new(&pos, spring_k, damper_k, pos.len()-1, (width-2) as usize));
            constraints.push(SpringConstraint::new(&pos, spring_k, damper_k, pos.len()-1, width as usize));
        }

        System::new(pos, mass, constraints, tensions, false)
    }
}
