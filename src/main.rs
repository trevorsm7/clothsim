mod constraint;
mod rasterize;
mod sim;

use sim::{SimulationBuilder, Node};
use rasterize::rasterize_line;

use cgmath::{Point2, Vector2};
use image::{ImageBuffer, RgbImage, Rgb};
use std::env;

fn main() {
    match env::args().nth(1).as_deref() {
        Some("net") => {
            let mass = 10.;
            let origin = Point2::new(-10., 10.);
            let u = Vector2::new(20., 0.);
            let v = Vector2::new(0., -20.);

            let mut builder = SimulationBuilder::new();
            builder.set_gravity(Vector2::new(0., -9.81));
            builder.set_damper(0.93);
            let spring_var = builder.push_variable(100.);
            builder.make_net(origin, u, v, mass, spring_var, 18);

            let mut sim = builder.build();
            let mut img: RgbImage = ImageBuffer::new(1024, 1024);
            let origin = Point2::new(-12., -14.);
            let size = Vector2::new(24., 24.);

            let steps = 2000;
            for i in 0..steps {
                let f = i as f32 / steps as f32;
                let u = (f * 255.).ceil() as u8;
                if i % 100 == 0 {
                    sim.rasterize(|start, end| rasterize_line(&mut img, origin, size, start, end, Rgb([u, 255 - u, u])));
                }
                sim.step(0.01);
            }
            sim.rasterize(|start, end| rasterize_line(&mut img, origin, size, start, end, Rgb([255, 0, 255])));

            img.save("test.png").unwrap();
        },
        Some("rig") => {
            let left_anchor = Point2::new(0., 14.);
            let mid_anchor = Point2::new(0., 0.);
            let right_anchor = Point2::new(14., 0.);
            let num_tensioners = 13;

            let mut builder = SimulationBuilder::new();
            let tension_var = builder.push_variable(1.);
            let spring_var = builder.push_variable(150.);
            builder.make_rig(left_anchor, mid_anchor, right_anchor, num_tensioners, tension_var, spring_var);
            builder.set_damper(0.97);

            let mut sim = builder.build();
            let mut img: RgbImage = ImageBuffer::new(1024, 1024);
            let origin = Point2::new(-1., -1.);
            let size = Vector2::new(30., 30.);

            let sweep_steps = 16;
            let sim_steps = 600;
            let max_tension = 4.;
            for sweep in 0..sweep_steps {
                let f = sweep as f32 / sweep_steps as f32;
                sim.set_variable(tension_var, max_tension * f);

                // Allow sim to stabilize after adjusting tension
                for _step in 0..sim_steps {
                    sim.step(0.01);
                    /*if sweep == 0 && step % 100 == 0 {
                        let f = step as f32 / sim_steps as f32;
                        let u = (f * 255.).ceil() as u8;
                        sim.rasterize(|start, end| rasterize_line(&mut img, origin, size, start, end, Rgb([u, 255 - u, u])));
                    }*/
                }

                let u = (f * 255.).ceil() as u8;
                sim.rasterize(|start, end| rasterize_line(&mut img, origin, size, start, end, Rgb([u, 255 - u, u])));
            }

            img.save("test.png").unwrap();
        },
        _ => {
            let mass = 1.;
            let start = Point2::new(-10., 0.);
            let end = Point2::new(10., 0.);

            let mut builder = SimulationBuilder::new();
            builder.set_gravity(Vector2::new(0., -9.81));
            builder.set_damper(0.95);
            let spring_var = builder.push_variable(300.);
            let start_idx = builder.push_point(start, 0.);
            let end_idx = builder.push_point(end, 0.);
            builder.make_rope(Node::Index(start_idx), Node::Index(end_idx), mass, spring_var, 8);

            let mut sim = builder.build();
            let mut img: RgbImage = ImageBuffer::new(1024, 1024);
            let origin = Point2::new(-11., -21.);
            let size = Vector2::new(22., 22.);

            let steps = 120;
            for i in 0..steps {
                let f = i as f32 / steps as f32;
                let u = (f * 255.).ceil() as u8;
                if i % 20 == 0 {
                    sim.rasterize(|start, end| rasterize_line(&mut img, origin, size, start, end, Rgb([u, 255 - u, u])));
                }
                sim.step(0.01);
            }
            sim.rasterize(|start, end| rasterize_line(&mut img, origin, size, start, end, Rgb([255, 0, 255])));

            img.save("test.png").unwrap();
        }
    };
}
