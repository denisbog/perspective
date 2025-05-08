use nalgebra::{Matrix3, RowVector3, Vector2, Vector3};

use anyhow::Result;
use optimization::{Func, GradientDescent, Minimizer, NumericalDifferentiation};

use crate::{
    PointInformation,
    compute::{
        compute_camera_pose, compute_camera_pose_scale, compute_camera_pose_translation,
        find_vanishing_point_for_lines, triangle_ortho_center,
    },
    utils::{calculate_location_position_to_2d, relative_to_image_plane},
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

pub fn pose_optimize(
    ratio: f32,
    axis_points: Vec<Vector2<f32>>,
    //draw_lines: Vec<Vector3<f32>>,
    control_point: Vector2<f32>,
    flip: (bool, bool, bool),
    translate_origin: Vector3<f32>,
    //scale_selected_segment: Option<usize>,
    //custom_scale: Option<PointInformation<f32>>,
    custom_error: Option<PointInformation<f32>>,
    scale: f64,
) -> Result<Vec<Vector2<f32>>> {
    // let draw_lines: Vec<Vector3<f64>> = draw_lines
    //     .iter()
    //     .map(|item| Vector3::new(item.x as f64, item.y as f64, item.z as f64))
    //     .collect();
    let translate_origin = Vector3::new(
        translate_origin.x as f64,
        translate_origin.y as f64,
        translate_origin.z as f64,
    );
    let ratio = ratio as f64;
    let control_point = Vector2::new(control_point.x as f64, control_point.y as f64);

    let user_selected_origin = relative_to_image_plane(ratio, &control_point);

    let x = if flip.0 { 1.0 } else { -1.0 };
    let y = if flip.1 { 1.0 } else { -1.0 };
    let z = if flip.2 { 1.0 } else { -1.0 };
    let axis = Matrix3::from_rows(&[
        RowVector3::new(x, 0.0, 0.0),
        RowVector3::new(0.0, y, 0.0),
        RowVector3::new(0.0, 0.0, z),
    ]);

    let points: Vec<f64> = axis_points
        .iter()
        .flat_map(|item| item.as_slice())
        .cloned()
        .map(|item| item as f64)
        .collect();
    // let custom_scale = custom_scale
    //     .as_ref()
    //     .map(Into::<PointInformation<f64>>::into);
    let custom_error = custom_error
        .as_ref()
        .map(Into::<PointInformation<f64>>::into);
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

                let compute_solution =
                    compute_camera_pose(&vanishing_points, &user_selected_origin, axis).unwrap();

                let compute_solution =
                    compute_camera_pose_translation(compute_solution, &translate_origin).unwrap();

                // let compute_solution = if let Some(custom_scale) = &custom_scale {
                //     let scale_vector = calculate_cursor_position_to_3d(
                //         &custom_scale.axis,
                //         &compute_solution,
                //         ratio,
                //         &custom_scale.point,
                //         custom_scale.source_vector,
                //     )
                //     .unwrap();

                //     let scale_vector = scale_vector - custom_scale.source_vector;
                //     let scale = if let Some(scale_selected_segment) = scale_selected_segment {
                //         let start = draw_lines.get(scale_selected_segment).unwrap();
                //         let end = draw_lines.get(scale_selected_segment + 1).unwrap();
                //         let length = start - end;
                //         length.norm()
                //     } else {
                //         1.0
                //     };

                //     let scale = scale_vector.norm() / scale;
                //     let compute_solution =
                //         compute_camera_pose_scale(compute_solution, scale).unwrap();

                //     let scale_vector_after_resize = calculate_cursor_position_to_3d(
                //         &custom_scale.axis,
                //         &compute_solution,
                //         ratio,
                //         &custom_scale.point,
                //         custom_scale.source_vector,
                //     )
                //     .unwrap();
                //     trace!("scale_vector_after_resize {}", scale_vector_after_resize);
                //     compute_solution
                // } else {
                //     compute_solution
                // };

                let compute_solution = compute_camera_pose_scale(compute_solution, scale).unwrap();

                // let projected_new = calculate_cursor_position_to_3d(
                //     &compute_solution,
                //     bounds,
                //     error_axis.clone(),
                //     &error_point,
                //     error_vector,
                // )
                // .unwrap();

                if let Some(custom_error) = &custom_error {
                    let custom_error_image_point =
                        relative_to_image_plane(ratio, &custom_error.point);
                    let projected = calculate_location_position_to_2d(
                        &Some(compute_solution),
                        &custom_error.source_vector,
                    )
                    .unwrap();
                    (projected - custom_error_image_point).norm()
                } else {
                    0.0
                }
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
