use super::constraint::{Tension, Spring, apply_tension, apply_spring};

use cgmath::prelude::*;
use cgmath::{Point2, Vector2};

pub enum Node {
    Point(Point2<f32>),
    Index(usize),
}

pub struct Line (pub usize, pub usize);

pub struct Simulation {
    // Components:
    pos: Vec<Point2<f32>>,
    vel: Vec<Vector2<f32>>,
    force: Vec<Vector2<f32>>,
    mass: Vec<f32>,
    line: Vec<Line>,
    spring: Vec<Spring>,
    tension: Vec<Tension>,
    // Resources:
    variables: Vec<f32>,
    gravity: Option<Vector2<f32>>,
    damper_k: f32,
}

impl Simulation {
    fn new(pos: Vec<Point2<f32>>, mass: Vec<f32>, spring: (Vec<Line>, Vec<Spring>), tension: (Vec<Line>, Vec<Tension>), variables: Vec<f32>, gravity: Option<Vector2<f32>>, damper_k: f32) -> Self {
        let len = pos.len();
        let force = vec![Vector2::zero(); len];
        let vel = vec![Vector2::zero(); len];
        let (mut line, spring) = spring;
        let (line_ext, tension) = tension;
        line.extend(line_ext);
        Simulation {pos, vel, force, mass, line, spring, tension, variables, gravity, damper_k}
    }

    pub fn set_variable(&mut self, index: u32, value: f32) {
        self.variables[index as usize] = value;
    }

    fn reset_force(&mut self) {
        for force in self.force.iter_mut() {
            *force = Vector2::zero();
        }
    }

    pub fn step(&mut self, dt: f32) {
        if let Some(gravity) = self.gravity {
            for i in 0..self.force.len() {
                let mass = self.mass[i];
                if mass != 0. { // HACK using zero mass for static anchor points
                    // Add gravitational acceleration first
                    self.force[i] += gravity * mass;
                }
            }
        }

        // Apply forces from each constraint
        // HACK manually splitting Line component between Spring and Tension
        // TODO implement joining components by matching entity ID
        // NOTE for some reason we have to borrow from self outside of the closure
        {
            let var = &self.variables;
            let pos = &self.pos;
            let force = &mut self.force;
            self.line.iter().take(self.spring.len()).zip(self.spring.iter())
                .for_each(|(line, spring)| apply_spring(line, spring, var, pos, force));
            self.line.iter().skip(self.spring.len()).zip(self.tension.iter())
                .for_each(|(line, tension)| apply_tension(line, tension, var, pos, force));
        }

        // Integrate force over each point by 1 time step
        for i in 0..self.force.len() {
            if self.mass[i] != 0. { // HACK zero mass for static points
                let next_vel = self.vel[i] + self.force[i] * dt / self.mass[i];
                self.pos[i] += next_vel * dt;
                self.vel[i] = next_vel * self.damper_k;
            }
        }

        self.reset_force();
    }

    pub fn rasterize<F: FnMut(Point2<f32>, Point2<f32>)>(&self, mut draw: F) {
        self.line.iter()
            .for_each(|&Line(a, b)| {
                let start = self.pos[a];
                let end = self.pos[b];
                draw(start, end);
            });
    }
}

pub struct SimulationBuilder {
    pos: Vec<Point2<f32>>,
    mass: Vec<f32>,
    springs: (Vec<Line>, Vec<Spring>),
    tensions: (Vec<Line>, Vec<Tension>),
    variables: Vec<f32>,
    gravity: Option<Vector2<f32>>,
    damper_k: f32,
}

impl SimulationBuilder {
    pub fn new() -> Self {
        let pos = vec![];
        let mass = vec![];
        let springs = (vec![], vec![]);
        let tensions = (vec![], vec![]);
        let variables = vec![];
        let gravity = None;
        let damper_k = 1.;
        SimulationBuilder {pos, mass, springs, tensions, variables, gravity, damper_k}
    }

    pub fn build(self) -> Simulation {
        Simulation::new(self.pos, self.mass, self.springs, self.tensions, self.variables, self.gravity, self.damper_k)
    }

    pub fn push_variable(&mut self, value: f32) -> u32 {
        let len = self.variables.len();
        self.variables.push(value);
        len as u32 // TODO how many bytes should resource address be? usize seems overkill
    }

    pub fn push_point(&mut self, point: Point2<f32>, mass: f32) -> usize {
        let len = self.pos.len();
        self.pos.push(point);
        self.mass.push(mass);
        len
    }

    pub fn push_spring(&mut self, spring_var: u32, a: usize, b: usize) {
        self.springs.0.push(Line(a, b));
        let length = self.pos[a].distance(self.pos[b]);
        self.springs.1.push(Spring{length, spring_var});
    }

    pub fn push_tension(&mut self, tension_var: u32, a: usize, b: usize) {
        self.tensions.0.push(Line(a, b));
        self.tensions.1.push(Tension(tension_var));
    }

    fn node_to_index(&mut self, node: Node, mass: f32) -> usize {
        match node {
            Node::Point(point) => self.push_point(point, mass),
            Node::Index(index) => index,
        }
    }

    pub fn set_gravity(&mut self, gravity: Vector2<f32>) {
        self.gravity = Some(gravity);
    }

    pub fn set_damper(&mut self, damper_k: f32) {
        self.damper_k = damper_k;
    }

    pub fn make_rope(&mut self, start: Node, end: Node, mass: f32, spring_var: u32, n: usize) -> Vec<usize> {
        self.make_rope_ext(start, end, |_| Vector2::zero(), mass, spring_var, n)
    }

    pub fn make_rope_ext<F: Fn(f32) -> Vector2<f32>>(&mut self, start: Node, end: Node, func: F, mass: f32, spring_var: u32, n: usize) -> Vec<usize> {
        let start_idx = self.node_to_index(start, mass / n as f32);
        let end_idx = self.node_to_index(end, mass / n as f32);

        // Lerp requires vector, not point
        let start = self.pos[start_idx].to_vec();
        let end = self.pos[end_idx].to_vec();

        let mut indices = Vec::new();
        indices.push(start_idx);
        for i in 1..n-1 {
            let t = i as f32 / (n - 1) as f32;
            let point = Point2::from_vec(start.lerp(end, t) + func(t));
            indices.push(self.push_point(point, mass / n as f32));
        }
        indices.push(end_idx);

        for (&a, &b) in indices.iter().zip(indices.iter().skip(1)) {
            self.push_spring(spring_var, a, b);
        }

        indices
    }

    pub fn make_net(&mut self, origin: Point2<f32>, u: Vector2<f32>, v: Vector2<f32>, mass: f32, spring_var: u32, n: usize) {
        for j in 0..n {
            for k in 0..n { // TODO use different dimension for v or change to nodes/meter?
                let j_n = j as f32 / (n - 1) as f32;
                let k_n = k as f32 / (n - 1) as f32;
                let p = origin + u * k_n + v * j_n;
                self.pos.push(p);

                if (j == 0 || j == n - 1) && (k == 0 || k == n - 1) {
                    self.mass.push(0.); // HACK use zero mass at corners as anchors
                } else {
                    self.mass.push(mass / (n * n) as f32);
                }
            }
        }

        for j in 0..n {
            for k in 0..n {
                let i = j * n + k;
                if k > 0 {
                    self.push_spring(spring_var, i - 1, i);
                }
                if k < n - 1 {
                    self.push_spring(spring_var, i, i + 1);
                }
                if j > 1 {
                    self.push_spring(spring_var, i - n, i);
                }
                if j < n - 1 {
                    self.push_spring(spring_var, i, i + n);
                }
            }
        }
    }

    pub fn make_rig(&mut self, left_anchor: Point2<f32>, mid_anchor: Point2<f32>, right_anchor: Point2<f32>, num_tensioners: usize, tension_var: u32, spring_var: u32) {
        let perp = mid_anchor.to_vec() - left_anchor.midpoint(right_anchor).to_vec();
        let left_edge = left_anchor.to_vec() - mid_anchor.to_vec();
        let right_edge = right_anchor.to_vec() - mid_anchor.to_vec();

        let left_corner = self.push_point(left_anchor + left_edge, 0.);
        let right_corner = self.push_point(right_anchor + right_edge, 0.);

        // TODO Fix magic number 5
        let left_anchor = self.push_point(left_anchor, 5. / num_tensioners as f32);
        let mid_anchor = self.push_point(mid_anchor, 0.);
        let right_anchor = self.push_point(right_anchor, 5. / num_tensioners as f32);

        // TODO add another variable for rope tension?
        //self.push_tension(tension * 8., left_corner, left_anchor);
        //self.push_tension(tension * 8., right_corner, right_anchor);
        self.push_spring(spring_var, left_corner, left_anchor);
        self.push_spring(spring_var, right_corner, right_anchor);

        let scaffold_n = if num_tensioners % 2 == 0 {num_tensioners + 2} else {(num_tensioners + 3) / 2};
        // TODO Fix magic number 5. (Take parameter for mass or density)
        let ropes = [(left_anchor, left_edge), (right_anchor, right_edge)];
        let mut ropes = ropes.iter()
            .map(|&(end_anchor, edge)| {
                self.make_rope(Node::Index(end_anchor), Node::Index(mid_anchor), 5., spring_var, scaffold_n)
                /*self.make_rope_ext(Node::Index(end_anchor), Node::Index(mid_anchor),
                    |_t| { edge * (1. / num_tensioners as f32) },
                    5., spring_var, scaffold_n)*/
            });
        let left_rope = ropes.next().unwrap();
        let right_rope = ropes.next().unwrap();

        let parabola_n = scaffold_n * 2 - 1;
        // TODO Fix magic number 5. (Take parameter for mass or density)
        let parabola_rope = self.make_rope_ext(Node::Index(left_anchor), Node::Index(right_anchor),
            |t| {
                let x = t - 0.5;
                perp * (0.25 - x * x)
            },
            5., spring_var, parabola_n);

        if num_tensioners % 2 == 0 {
            // ex n=14: use 7 on each side (one tensioner per two rope segments out of 16)
            for (&a, &b) in left_rope.iter().zip(parabola_rope.iter())
                    .step_by(2).skip(1).take(num_tensioners / 2) {
                self.push_tension(tension_var, a, b);
            }
            for (&a, &b) in right_rope.iter().zip(parabola_rope.iter().rev())
                    .step_by(2).skip(1).take(num_tensioners / 2) {
                self.push_tension(tension_var, a, b);
            }
            // TODO Run remaining tensioners along scaffold or parabola
            /*sim.tensions.push(TensionConstraint::new(tension_var,
                left_rope.iter().cloned().rev().nth(1).unwrap(),
                right_rope.iter().cloned().rev().nth(1).unwrap()));*/
        } else {
            // ex n=13: use 6 on each side plus one in the middle
            for (&a, &b) in left_rope.iter().zip(parabola_rope.iter())
                    .skip(1).take((num_tensioners - 1) / 2) {
                self.push_tension(tension_var, a, b);
            }
            self.push_tension(tension_var, mid_anchor, parabola_rope[scaffold_n - 1]);
            for (&a, &b) in right_rope.iter().zip(parabola_rope.iter().rev())
                    .skip(1).take((num_tensioners - 1) / 2) {
                self.push_tension(tension_var, a, b);
            }
        }
    }
}
