mod constraint;
mod double_buffer;
mod system;

use cgmath::prelude::*;
use cgmath::{Point2, Vector2};

use image::{ImageBuffer, RgbImage, Rgb};

use std::env;


use system::System;

fn rasterize_clipped_line(mut img: &mut RgbImage, start: Point2<f32>, end: Point2<f32>, color: Rgb<u8>) {
    let w = img.width();
    let h = img.height();

    let start_x = start.x * (w - 1) as f32;
    let start_y = start.y * (h - 1) as f32;
    let end_x = end.x * (w - 1) as f32;
    let end_y = end.y * (h - 1) as f32;

    // Lerp from start to end by number of pixels across the widest dimension
    let max_dim = (end_x - start_x).abs().max((end_y - start_y).abs()) as u32;
    for i in 0..max_dim {
        let i_f = i as f32 / (max_dim - 1) as f32;
        let start_p = Vector2::new(start_x, start_y);
        let end_p = Vector2::new(end_x, end_y);
        let p = start_p.lerp(end_p, i_f);
        safe_put_pixel(&mut img, p.x as u32, p.y as u32, color);
    }
}

fn rasterize_line(mut img: &mut RgbImage, start: Point2<f32>, end: Point2<f32>, color: Rgb<u8>) {
    let start_visible = start.x >= 0. && start.x <= 1. && start.y >= 0. && start.y <= 1.;
    let end_visible = end.x >= 0. && end.x <= 1. && end.y >= 0. && end.y <= 1.;

    if start_visible && end_visible {
        rasterize_clipped_line(&mut img, start, end, color);
    } else if start_visible {
        println!("start clipped! ({}, {})", start.x, start.y);
        // TODO Find intersection and lerp
    } else if end_visible {
        println!("end clipped! ({}, {})", end.x, end.y);
        // TODO Find intersection and lerp
    } else {
        println!("both clipped! ({}, {}), ({}, {})", start.x, start.y, end.x, end.y);
        // TODO Find two intersections and lerp OR no intersections and skip
    }
}

fn safe_put_pixel(img: &mut RgbImage, x: u32, y: u32, rgb: Rgb<u8>) {
    if x < img.width() && y < img.height() {
        img.put_pixel(x, y, rgb);
    }
}

/*fn pixel_size(size: Vector2<f32>, pixels: u32) -> (u32, u32) {
    if size.y > size.x {
        (f32::ceil(pixels as f32 * size.x / size.y) as u32, pixels)
    } else {
        (pixels, f32::ceil(pixels as f32 * size.y / size.x) as u32)
    }
}*/

fn main() {
    let use_net = env::args().nth(1).as_deref() == Some("net");
    let mut system = if use_net {
        let mass = 10.;
        let spring_k = 1.;
        let damper_k = 0.2;
        System::make_net(Point2::new(-10., 10.), Vector2::new(20., 0.), Vector2::new(0., -20.), mass, spring_k, damper_k, 18)
    } else {
        let mass = 0.1;
        let spring_k = 40.;
        let damper_k = 0.1;
        System::make_rope(Point2::new(-10., 0.), Point2::new(10., 0.), mass, spring_k, damper_k, 8)
    };

    //let (origin, size) = system.find_bounds();
    let origin = Point2::new(-11., if use_net {-11.} else {-0.01});
    let size = Vector2::new(22., if use_net {22.} else {0.05});
    //println!("origin: {:?}, size: {:?}", origin, size);

    //let (w, h) = pixel_size(size, 512);
    let w = 512;
    let h = 512;
    //println!("w: {}, h: {}", w, h);

    let mut img: RgbImage = ImageBuffer::new(w, h);

    let steps = 15;
    for i in 0..steps {
        let f = i as f32 / steps as f32;
        let u = (f * 255.).ceil() as u8;
        system.rasterize(&mut img, origin, size, Rgb([u, 255 - u, u]));
        system.step(0.01);
    }
    system.rasterize(&mut img, origin, size, Rgb([0, 255, 255]));

    /*let start = to_clip_space(Point2::new(9., 0.09), origin, size);
    let end = to_clip_space(Point2::new(-9., -0.09), origin, size);
    rasterize_line(&mut img, start, end, Rgb([255, 255, 255]));*/

    img.save("test.png").unwrap();
}
