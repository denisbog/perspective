use nalgebra::Vector2;

use anyhow::Result;
use optimization::{Func, GradientDescent, Minimizer, NumericalDifferentiation};

use crate::{
    compute::{find_vanishing_point_for_lines, triangle_ortho_center},
    utils::relative_to_image_plane,
};

pub fn ortho_center_optimize(ratio: f32, points: Vec<Vector2<f32>>) -> Result<Vec<Vector2<f32>>> {
    let points: Vec<f64> = points
        .iter()
        .flat_map(|item| item.as_slice())
        .cloned()
        .map(|item| item as f64)
        .collect();
    let ratio = ratio as f64;

    let minimizer = GradientDescent::new();
    let solution = minimizer
        .gradient_tolerance(1e-7)
        .max_iterations(Some(12))
        .minimize(
            &NumericalDifferentiation::new(Func(|x: &[f64]| {
                let points: Vec<Vector2<f64>> = x
                    .chunks(2)
                    .map(|items| Vector2::new(items[0], items[1]))
                    .collect();

                let vanishing_points = points
                    .chunks(4)
                    .map(|lines| {
                        find_vanishing_point_for_lines(&lines[0], &lines[1], &lines[2], &lines[3])
                    })
                    .collect::<Vec<Vector2<f64>>>();

                let vanishing_points = vanishing_points
                    .iter()
                    .map(|point| relative_to_image_plane(ratio, point))
                    .collect::<Vec<Vector2<f64>>>();

                let ortho_center = triangle_ortho_center(
                    &vanishing_points[0],
                    &vanishing_points[1],
                    &vanishing_points[2],
                );

                ortho_center.norm() as f64
            })),
            points,
        );
    let points: Vec<Vector2<f32>> = solution
        .position
        .chunks(2)
        .map(|items| Vector2::new(items[0] as f32, items[1] as f32))
        .collect();
    Ok(points)
}

pub fn ortho_center_optimize_x(ratio: f32, points: Vec<Vector2<f32>>) -> Result<Vec<Vector2<f32>>> {
    let points: Vec<f64> = points
        .iter()
        .flat_map(|item| item.as_slice())
        .cloned()
        .map(|item| item as f64)
        .collect();
    let ratio = ratio as f64;

    let points_yz: Vec<Vector2<f64>> = points
        .chunks(2)
        .skip(4)
        .map(|items| Vector2::new(items[0], items[1]))
        .collect();
    let points_xb: Vec<Vector2<f64>> = points
        .chunks(2)
        .skip(2)
        .take(2)
        .map(|items| Vector2::new(items[0], items[1]))
        .collect();

    let vanishing_points = points_yz
        .chunks(4)
        .map(|lines| find_vanishing_point_for_lines(&lines[0], &lines[1], &lines[2], &lines[3]))
        .collect::<Vec<Vector2<f64>>>();

    let vanishing_points_yz = vanishing_points
        .iter()
        .map(|point| relative_to_image_plane(ratio, point))
        .collect::<Vec<Vector2<f64>>>();

    let minimizer = GradientDescent::new();
    let solution = minimizer
        .gradient_tolerance(1e-7)
        .max_iterations(Some(12))
        .minimize(
            &NumericalDifferentiation::new(Func(|x: &[f64]| {
                let points: Vec<Vector2<f64>> = x
                    .chunks(2)
                    .map(|items| Vector2::new(items[0], items[1]))
                    .collect();

                //let vanishing_points = points
                //    .chunks(4)
                //    .map(|lines| {
                //        find_vanishing_point_for_lines(&lines[0], &lines[1], &lines[2], &lines[3])
                //    })
                //    .collect::<Vec<Vector2<f64>>>();

                //let vanishing_points = vanishing_points
                //    .iter()
                //    .map(|point| relative_to_image_plane(ratio, point))
                //    .collect::<Vec<Vector2<f64>>>();
                //
                let vanishing_point = find_vanishing_point_for_lines(
                    &points[0],
                    &points[1],
                    &points_xb[0],
                    &points_xb[1],
                );
                let vanishing_point = relative_to_image_plane(ratio, &vanishing_point);

                let ortho_center = triangle_ortho_center(
                    &vanishing_point,
                    &vanishing_points_yz[0],
                    &vanishing_points_yz[1],
                    //&vanishing_points[1],
                    //&vanishing_points[2],
                );

                ortho_center.norm() as f64
            })),
            points.iter().take(4).cloned().collect(),
        );
    let mut optimized_x: Vec<Vector2<f32>> = solution
        .position
        .chunks(2)
        .map(|items| Vector2::new(items[0] as f32, items[1] as f32))
        .collect();
    optimized_x.extend(
        points
            .chunks(2)
            .skip(2)
            .take(2)
            .map(|items| Vector2::new(items[0] as f32, items[1] as f32)),
    );
    optimized_x.extend(
        points
            .chunks(2)
            .skip(4)
            .map(|items| Vector2::new(items[0] as f32, items[1] as f32)),
    );
    Ok(optimized_x)
}

pub fn ortho_center_optimize_y(ratio: f32, points: Vec<Vector2<f32>>) -> Result<Vec<Vector2<f32>>> {
    let points: Vec<f64> = points
        .iter()
        .flat_map(|item| item.as_slice())
        .cloned()
        .map(|item| item as f64)
        .collect();
    let ratio = ratio as f64;

    let mut points_yz: Vec<Vector2<f64>> = points
        .chunks(2)
        .take(4)
        .map(|items| Vector2::new(items[0], items[1]))
        .collect();
    points_yz.extend(
        points
            .chunks(2)
            .skip(8)
            .map(|items| Vector2::new(items[0], items[1])),
    );

    let points_yb: Vec<Vector2<f64>> = points
        .chunks(2)
        .skip(6)
        .take(2)
        .map(|items| Vector2::new(items[0], items[1]))
        .collect();

    let vanishing_points = points_yz
        .chunks(4)
        .map(|lines| find_vanishing_point_for_lines(&lines[0], &lines[1], &lines[2], &lines[3]))
        .collect::<Vec<Vector2<f64>>>();

    let vanishing_points_xz = vanishing_points
        .iter()
        .map(|point| relative_to_image_plane(ratio, point))
        .collect::<Vec<Vector2<f64>>>();

    let minimizer = GradientDescent::new();
    let solution = minimizer
        .gradient_tolerance(1e-7)
        .max_iterations(Some(12))
        .minimize(
            &NumericalDifferentiation::new(Func(|x: &[f64]| {
                let points: Vec<Vector2<f64>> = x
                    .chunks(2)
                    .map(|items| Vector2::new(items[0], items[1]))
                    .collect();

                let vanishing_point = find_vanishing_point_for_lines(
                    &points[0],
                    &points[1],
                    &points_yb[0],
                    &points_yb[1],
                );
                let vanishing_point = relative_to_image_plane(ratio, &vanishing_point);

                let ortho_center = triangle_ortho_center(
                    &vanishing_point,
                    &vanishing_points_xz[0],
                    &vanishing_points_xz[1],
                );

                ortho_center.norm() as f64
            })),
            points.iter().skip(8).take(4).cloned().collect(),
        );

    let mut optimized_y: Vec<Vector2<f32>> = points
        .chunks(2)
        .take(4)
        .map(|items| Vector2::new(items[0] as f32, items[1] as f32))
        .collect();

    optimized_y.extend(
        solution
            .position
            .chunks(2)
            .map(|items| Vector2::new(items[0] as f32, items[1] as f32)),
    );

    optimized_y.extend(
        points
            .chunks(2)
            .skip(6)
            .take(2)
            .map(|items| Vector2::new(items[0] as f32, items[1] as f32)),
    );
    optimized_y.extend(
        points
            .chunks(2)
            .skip(8)
            .map(|items| Vector2::new(items[0] as f32, items[1] as f32)),
    );
    Ok(optimized_y)
}
