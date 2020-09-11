use cgmath::prelude::*;
use cgmath::{Point2, Vector2, Matrix2, Deg};

use image::{GenericImage, GenericImageView, ImageBuffer, RgbImage, Rgb};

struct Constraint {
    length: f32,
    spring_k: f32,
    anchor: usize,
}

impl Constraint {
    fn new(length: f32, spring_k: f32, anchor: usize) -> Self {
        Constraint {length, spring_k, anchor}
    }
}

struct System {
    points: Vec<Point2<f32>>,
    masses: Vec<f32>,
    n_constraints: Vec<usize>,
    constraints: Vec<Constraint>,
}

impl System {
    fn step(&mut self) {
        // TODO save swap buffer?
        let mut new_points = self.points.clone();

        let down = Vector2::new(0., 9.81); // NOTE down is positive, a hack to flip the output

        let mut constraint_i = 0;
        for i in 0..self.points.len() {
            let mass = self.masses[i];
            let n_constraints = self.n_constraints[i];
            if mass != 0. { // HACK using zero mass for static anchor points
                let point = &self.points[i];
                let mut net_force = down * mass;
                for c_i in 0..n_constraints {
                    let constraint = &self.constraints[constraint_i + c_i];
                    let other = &self.points[constraint.anchor];
                    let to_other = other - point;
                    let length = to_other.magnitude();
                    let to_other = to_other / length; // .normalize();
                    if length >= constraint.length {
                        net_force += to_other * (length - constraint.length) * constraint.spring_k;
                    }
                }
                let dt = 0.01;
                new_points[i] = point + net_force * dt / mass;
            }
            constraint_i += n_constraints;
        }

        std::mem::swap(&mut self.points, &mut new_points);
    }

    fn find_bounds(&self) -> (Point2<f32>, Vector2<f32>) {
        let mut min = Point2::new(f32::INFINITY, f32::INFINITY);
        let mut max = Point2::new(f32::NEG_INFINITY, f32::NEG_INFINITY);
        for point in &self.points {
            min = Point2::new(min.x.min(point.x), min.y.min(point.y));
            max = Point2::new(max.x.max(point.x), max.y.max(point.y));
        }
        (min, max - min)
    }

    fn rasterize(&self, pixels: u32) {
        let (origin, size) = self.find_bounds();

        let (w, h) = if size.y > size.x {
            (f32::ceil(pixels as f32 * size.x / size.y) as u32, pixels)
        } else {
            (pixels, f32::ceil(pixels as f32 * size.y / size.x) as u32)
        };

        let mut img: RgbImage = ImageBuffer::new(w, h);

        self.points.iter().zip(self.points.iter().skip(1))
            .for_each(|(start, end)| {
                //TODO rasterize_line(&mut img, start, end);

                // TODO assuming start.x < end.x
                assert!(start.x < end.x);
                let start_x = ((start.x - origin.x) * (w - 1) as f32 / size.x) as u32;
                let start_y = ((start.y - origin.y) * (h - 1) as f32 / size.y) as u32;
                let end_x = ((end.x - origin.x) * (w - 1) as f32 / size.x) as u32;
                let end_y = ((end.y - origin.y) * (h - 1) as f32 / size.y) as u32;
                img.put_pixel(start_x, start_y, Rgb([255, 255, 255]));
                img.put_pixel(end_x, end_y, Rgb([255, 255, 255]));

                // TODO by iterating in x, y may be stippled
                for x in (start_x+1)..end_x {
                    let y = (x - start_x) as f32 * (end_y as f32 - start_y as f32) / (end_x as f32 - start_x as f32) + start_y as f32;
                    if y >= h as f32 { continue; }
                    img.put_pixel(x, y as u32, Rgb([255, 0, 255]));
                }
            });
        img.save("test.png").unwrap();
    }

    fn make_rope(start: Point2<f32>, end: Point2<f32>, mass: f32, spring_k: f32, n: usize) -> Self {
        let mut points = vec![];
        let mut masses = vec![];
        let mut n_constraints = vec![];
        let mut constraints = vec![];

        for i in 0..n {
            // TODO Why can't we lerp from Point2?
            let p = start.to_vec().lerp(end.to_vec(), i as f32 / (n - 1) as f32);
            points.push(Point2::from_vec(p));
        }

        masses.push(0.);
        n_constraints.push(0);
        for i in 1..(n - 1) {
            masses.push(mass / (n - 2) as f32);
            n_constraints.push(2);
            let left = (points[i] - points[i - 1]).magnitude();
            let right = (points[i + 1] - points[i]).magnitude();
            constraints.push(Constraint::new(left, spring_k, i - 1));
            constraints.push(Constraint::new(right, spring_k, i + 1));
        }
        masses.push(0.);
        n_constraints.push(0);

        System { points, masses, n_constraints, constraints }
    }
}

fn main() {
    let mut system = System::make_rope(Point2::new(-10., 0.), Point2::new(10., 0.), 10., 60., 18);

    for _ in 0..30 {
        //println!("{:?}", system.points);
        system.step();
    }
    //println!("{:?}", system.find_bounds());
    system.rasterize(512);
    /*let point = Vector2::new(3f32, 4f32);
    let matrix = Matrix2::from_angle(Deg(90f32));
    let result = matrix * point;
    println!("{:?} * {:?} = {:?}", matrix, point, result);*/
}
