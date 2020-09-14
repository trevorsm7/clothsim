use cgmath::prelude::*;
use cgmath::{Point2, Vector2, Matrix2, Deg};

use image::{GenericImage, GenericImageView, ImageBuffer, RgbImage, Rgb};

#[derive(Copy, Clone, Debug)]
struct Constraint {
    length: f32,
    spring_k: f32,
    damper_k: f32,
    a: usize,
    b: usize,
}

impl Constraint {
    fn new(length: f32, spring_k: f32, damper_k: f32, a: usize, b: usize) -> Self {
        Constraint {length, spring_k, damper_k, a, b}
    }
}

fn safe_put_pixel(img: &mut RgbImage, x: u32, y: u32, rgb: Rgb<u8>) {
    if x < img.width() && y < img.height() {
        img.put_pixel(x, y, rgb);
    }
}

struct DoubleBuffer<T> {
    front: Vec<T>,
    back: Vec<T>,
}

impl<T: Clone> DoubleBuffer<T> {
    fn from(front: Vec<T>) -> Self { // TODO From trait?
        let back = front.clone();
        DoubleBuffer {front, back}
    }

    // TODO is it ok to use 'front' as both method and data?
    fn front(&self) -> &Vec<T> {
        &self.front
    }

    fn write_next(&mut self, i: usize, value: T) {
        self.back[i] = value;
    }

    fn read(&self, i: usize) -> T {
        self.front[i].clone()
    }

    fn read_next(&self, i: usize) -> T {
        self.back[i].clone()
    }

    fn flip(&mut self) {
        std::mem::swap(&mut self.front, &mut self.back);
    }
}

struct System {
    pos: DoubleBuffer<Point2<f32>>,
    vel: DoubleBuffer<Vector2<f32>>,
    force: Vec<Vector2<f32>>,
    mass: Vec<f32>,
    constraints: Vec<Constraint>,
}

impl System {
    fn new(pos: Vec<Point2<f32>>, mass: Vec<f32>, constraints: Vec<Constraint>) -> Self {
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

    fn step(&mut self, dt: f32) {
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

            // Compute relative scalar velocity
            let a_vel = self.vel.read(constraint.a);
            let b_vel = self.vel.read(constraint.b);
            let a_vel = a_to_b.dot(a_vel);
            let b_vel = a_to_b.dot(b_vel);

            let spring_force = (length - constraint.length) * constraint.spring_k;
            let damper_force = (b_vel - a_vel) * constraint.damper_k;

            self.apply_force(constraint.a, a_to_b * (spring_force + damper_force));
            self.apply_force(constraint.b, b_to_a * (spring_force + damper_force));
        }

        // Integrate force over each point by 1 time step
        for i in 0..self.force.len() { //self.pos.front().len() {
            let next_vel = self.vel.read(i) + self.force[i] * dt / self.mass[i];
            self.pos.write_next(i, self.pos.read(i) + next_vel * dt);
            self.vel.write_next(i, next_vel);
        }

        // Swap the next values from the back buffer
        self.pos.flip();
        self.vel.flip();
        self.reset_force();
    }

    fn find_bounds(&self) -> (Point2<f32>, Vector2<f32>) {
        let mut min = Point2::new(f32::INFINITY, f32::INFINITY);
        let mut max = Point2::new(f32::NEG_INFINITY, f32::NEG_INFINITY);
        for point in self.pos.front().iter() {
            min = Point2::new(min.x.min(point.x), min.y.min(point.y));
            max = Point2::new(max.x.max(point.x), max.y.max(point.y));
        }
        (min, max - min)
    }

    fn rasterize(&self, mut img: &mut RgbImage, origin: Point2<f32>, size: Vector2<f32>, color: Rgb<u8>) {
        let w = img.width();
        let h = img.height();
        self.pos.front().iter().zip(self.pos.front().iter().skip(1))
            .for_each(|(start, end)| {
                //TODO rasterize_line(&mut img, start, end);

                let start_x = ((start.x - origin.x) * (w - 1) as f32 / size.x) as u32;
                let start_y = ((start.y - origin.y) * (h - 1) as f32 / size.y) as u32;
                let end_x = ((end.x - origin.x) * (w - 1) as f32 / size.x) as u32;
                let end_y = ((end.y - origin.y) * (h - 1) as f32 / size.y) as u32;
                safe_put_pixel(&mut img, start_x, start_y, color);
                safe_put_pixel(&mut img, end_x, end_y, color);

                // TODO instead of x, choose largest of x and y to eliminate stipple
                // TODO assuming start.x < end.x
                //assert!(start.x < end.x);
                for x in (start_x+1)..end_x {
                    let y = (x - start_x) as f32 * (end_y as f32 - start_y as f32) / (end_x as f32 - start_x as f32) + start_y as f32;
                    if y >= h as f32 { continue; }
                    safe_put_pixel(&mut img, x, y as u32, color);
                }
            });
    }

    fn make_rope(start: Point2<f32>, end: Point2<f32>, mass: f32, spring_k: f32, damper_k: f32, n: usize) -> Self {
        let mut points = vec![];
        let mut masses = vec![];
        let mut constraints = vec![];

        for i in 0..n {
            // TODO Why can't we lerp from Point2?
            let p = start.to_vec().lerp(end.to_vec(), i as f32 / (n - 1) as f32);
            points.push(Point2::from_vec(p));
        }

        masses.push(0.);
        for i in 1..(n - 1) {
            masses.push(mass / (n - 2) as f32);
            let left = (points[i] - points[i - 1]).magnitude();
            let right = (points[i + 1] - points[i]).magnitude();
            constraints.push(Constraint::new(left, spring_k, damper_k, i - 1, i));
            constraints.push(Constraint::new(right, spring_k, damper_k, i, i + 1));
        }
        masses.push(0.);

        System::new(points, masses, constraints)
    }

    fn make_net(origin: Point2<f32>, u: Vector2<f32>, v: Vector2<f32>, mass: f32, spring_k: f32, damper_k: f32, n: usize) -> Self {
        let mut points = vec![];
        let mut masses = vec![];
        let mut constraints = vec![];

        for j in 0..n {
            for k in 0..n { // TODO use different dimension for v or change to nodes/meter?
                let j_n = j as f32 / (n - 1) as f32;
                let k_n = k as f32 / (n - 1) as f32;
                let p = origin + u * j_n + v * k_n;
                points.push(p);

                if (j == 0 || j == n - 1) && (k == 0 || k == n - 1) {
                    masses.push(0.); // HACK use zero mass at corners as anchors
                } else {
                    masses.push(mass / (n * n - 4) as f32);
                }
            }
        }

        for j in 0..n {
            for k in 0..n {

                let i = j * n + k;
                if k > 0 {
                    let left = (points[i] - points[i - 1]).magnitude();
                    constraints.push(Constraint::new(left, spring_k, damper_k, i - 1, i));
                }
                if k < n - 1 {
                    let right = (points[i + 1] - points[i]).magnitude();
                    constraints.push(Constraint::new(right, spring_k, damper_k, i, i + 1));
                }
                if j > 1 {
                    let up = (points[i - n] - points[i]).magnitude();
                    constraints.push(Constraint::new(up, spring_k, damper_k, i - n, i));
                }
                if j < n - 1 {
                    let down = (points[i + n] - points[i]).magnitude();
                    constraints.push(Constraint::new(down, spring_k, damper_k, i, i + n));
                }
            }
        }

        System::new(points, masses, constraints)
    }
}

fn pixel_size(origin: Point2<f32>, size: Vector2<f32>, pixels: u32) -> (u32, u32) {
    if size.y > size.x {
        (f32::ceil(pixels as f32 * size.x / size.y) as u32, pixels)
    } else {
        (pixels, f32::ceil(pixels as f32 * size.y / size.x) as u32)
    }
}

fn main() {
    let mass = 10.;
    let spring_k = 60.;
    let damper_k = 20.;
    let mut system = System::make_rope(Point2::new(-10., 0.), Point2::new(10., 0.), mass, spring_k, damper_k, 18);
    //let mut system = System::make_net(Point2::new(-10., 10.), Vector2::new(20., 0.), Vector2::new(0., -20.), mass, spring_k, damper_k, 18);

    //let (origin, size) = system.find_bounds();
    let origin = Point2::new(-10., 0.);
    let size = Vector2::new(20., 0.5);
    println!("origin: {:?}, size: {:?}", origin, size);
    //let (w, h) = pixel_size(origin, size, 512);
    let w = 512;
    let h = 512;
    println!("w: {}, h: {}", w, h);
    let mut img: RgbImage = ImageBuffer::new(w, h);
    for i in 0..20 {
        //println!("{:?}", system.points);
        let f = i as f32 / 20 as f32;
        let u = (f * 255.).ceil() as u8;
        system.rasterize(&mut img, origin, size, Rgb([u, 255 - u, u]));
        system.step(0.01);
    }
    img.save("test.png").unwrap();
    /*let point = Vector2::new(3f32, 4f32);
    let matrix = Matrix2::from_angle(Deg(90f32));
    let result = matrix * point;
    println!("{:?} * {:?} = {:?}", matrix, point, result);*/
}
