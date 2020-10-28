mod constraint;
mod double_buffer;
mod rasterize;
mod system;

use system::System;
use rasterize::rasterize_line;

use cgmath::{Point2, Vector2};
use image::{ImageBuffer, RgbImage, Rgb};
use std::env;

/*fn pixel_size(size: Vector2<f32>, pixels: u32) -> (u32, u32) {
    if size.y > size.x {
        (f32::ceil(pixels as f32 * size.x / size.y) as u32, pixels)
    } else {
        (pixels, f32::ceil(pixels as f32 * size.y / size.x) as u32)
    }
}*/

fn main() {
    let (mut system, origin, size) = match env::args().nth(1).as_deref() {
        Some("net") => {
            let mass = 10.;
            let spring_k = 1.;
            let damper_k = 0.2;
            let origin = Point2::new(-10., 10.);
            let u = Vector2::new(20., 0.);
            let v = Vector2::new(0., -20.);
            (System::make_net(origin, u, v, mass, spring_k, damper_k, 18),
                Point2::new(-11., -11.), Vector2::new(22., 22.))
        },
        Some("rig") => {
            let tension = 5.;
            let spring_k = 10.;
            let damper_k = 0.1;
            (System::make_rig(15, tension, spring_k, damper_k), 
                Point2::new(-8., -4.), Vector2::new(16., 16.))
        },
        _ => {
            let mass = 0.1;
            let spring_k = 40.;
            let damper_k = 0.1;
            let start = Point2::new(-10., 0.);
            let end = Point2::new(10., 0.);
            (System::make_rope_sys(start, end, mass, spring_k, damper_k, 8),
                Point2::new(-11., -0.01), Vector2::new(22., 0.05))
        }
    };

    //let (origin, size) = system.find_bounds();
    //println!("origin: {:?}, size: {:?}", origin, size);

    //let (w, h) = pixel_size(size, 512);
    let w = 512;
    let h = 512;
    //println!("w: {}, h: {}", w, h);

    let mut img: RgbImage = ImageBuffer::new(w, h);

    let steps = 30;
    for i in 0..steps {
        let f = i as f32 / steps as f32;
        let u = (f * 255.).ceil() as u8;
        system.rasterize(origin, size, |start, end| rasterize_line(&mut img, start, end, Rgb([u, 255 - u, u])));
        system.step(0.01);
    }
    system.rasterize(origin, size, |start, end| rasterize_line(&mut img, start, end, Rgb([255, 0, 255])));

    /*let start = to_clip_space(Point2::new(9., 0.09), origin, size);
    let end = to_clip_space(Point2::new(-9., -0.09), origin, size);
    rasterize_line(&mut img, start, end, Rgb([255, 255, 255]));*/

    img.save("test.png").unwrap();
}
