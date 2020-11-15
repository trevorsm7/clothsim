use cgmath::prelude::*;
use cgmath::{Point2, Vector2};
use image::{RgbImage, Rgb};

fn to_clip_space(point: Point2<f32>, origin: Point2<f32>, size: Vector2<f32>) -> Point2<f32> {
    Point2::new((point.x - origin.x) / size.x, (point.y - origin.y) / size.y)
}

pub fn rasterize_line(mut img: &mut RgbImage, origin: Point2<f32>, size: Vector2<f32>, start: Point2<f32>, end: Point2<f32>, color: Rgb<u8>) {
    let start = to_clip_space(start, origin, size);
    let end = to_clip_space(end, origin, size);

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

fn rasterize_clipped_line(mut img: &mut RgbImage, start: Point2<f32>, end: Point2<f32>, color: Rgb<u8>) {
    let w = img.width();
    let h = img.height();

    let start_x = start.x * (w - 1) as f32;
    let start_y = start.y * (h - 1) as f32;
    let end_x = end.x * (w - 1) as f32;
    let end_y = end.y * (h - 1) as f32;

    // Lerp from start to end by number of pixels across the widest dimension
    let max_dim = (end_x - start_x).abs().max((end_y - start_y).abs()).ceil() as u32;
    for i in 0..=max_dim {
        let i_f = i as f32 / max_dim as f32;
        let start_p = Vector2::new(start_x, start_y);
        let end_p = Vector2::new(end_x, end_y);
        let p = start_p.lerp(end_p, i_f);
        safe_put_pixel(&mut img, p.x as u32, p.y as u32, color);
    }
}

fn safe_put_pixel(img: &mut RgbImage, x: u32, y: u32, rgb: Rgb<u8>) {
    if x < img.width() && y < img.height() {
        img.put_pixel(x, y, rgb);
    }
}
