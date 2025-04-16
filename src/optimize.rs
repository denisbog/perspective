use nalgebra::{Matrix3, Matrix4, Perspective3, Point2, Point3, RowVector3, Vector2, Vector3};

use anyhow::Result;
use optimization::{Func, GradientDescent, Minimizer, NumericalDifferentiation};

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
                        find_vanishing_point_for_lines_local(
                            &lines[0], &lines[1], &lines[2], &lines[3],
                        )
                    })
                    .collect::<Vec<Vector2<f64>>>();

                let vanishing_points = vanishing_points
                    .iter()
                    .map(|point| relative_to_image_plane_local(ratio, point))
                    .collect::<Vec<Vector2<f64>>>();

                let ortho_center = triangle_ortho_center_local(
                    &vanishing_points[0],
                    &vanishing_points[1],
                    &vanishing_points[2],
                );

                let out = ortho_center.norm() as f64;
                out
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

pub fn pose_optimize(
    ratio: f64,
    bounds: Vector2<f64>,
    points: Vec<Vector2<f64>>,
    control_point: &Vector2<f64>,
    flip: (bool, bool, bool),
    translate_origin: &Option<Vector3<f64>>,
    scale: &Option<f64>,
    scale_selected: usize,
    scale_vector: Vector2<f32>,
    error_selected: usize,
    error_vector: Vector3<f64>,
    error_point: Vector2<f64>,
) -> Result<Vec<Vector2<f32>>> {
    let user_selected_origin = relative_to_image_plane_local(ratio, control_point);

    let x = if flip.0 { 1.0 } else { -1.0 };
    let y = if flip.1 { 1.0 } else { -1.0 };
    let z = if flip.2 { 1.0 } else { -1.0 };
    let axis = Matrix3::from_rows(&[
        RowVector3::new(x, 0.0, 0.0),
        RowVector3::new(0.0, y, 0.0),
        RowVector3::new(0.0, 0.0, z),
    ]);

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
                        find_vanishing_point_for_lines_local(
                            &lines[0], &lines[1], &lines[2], &lines[3],
                        )
                    })
                    .collect::<Vec<Vector2<f64>>>();

                let vanishing_points = vanishing_points
                    .iter()
                    .map(|point| relative_to_image_plane_local(ratio, point))
                    .collect::<Vec<Vector2<f64>>>();

                let compute_solution =
                    compute_camera_pose_local(&vanishing_points, &user_selected_origin, axis);

                let compute_solution = if let Ok(compute_solution) = compute_solution {
                    if let Some(scale) = scale {
                        compute_camera_pose_scale_local(compute_solution, *scale)
                    } else {
                        Ok(compute_solution)
                    }
                } else {
                    compute_solution
                };

                let compute_solution = if let Some(translate_origin) = translate_origin {
                    if let Ok(compute_solution) = compute_solution {
                        compute_camera_pose_translation_local(compute_solution, translate_origin)
                    } else {
                        compute_solution
                    }
                } else {
                    compute_solution
                };

                let projected = calculate_location_position_to_2d_local(
                    compute_solution.unwrap(),
                    bounds,
                    &error_vector,
                )
                .unwrap();
                (projected - error_point).norm()
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

fn calculate_location_position_to_2d_local(
    compute_solution: ComputeSolutionLocal,
    bounds: Vector2<f64>,
    location3d: &Vector3<f64>,
) -> Option<Vector2<f64>> {
    let dc_to_image =
        Matrix3::new_nonuniform_scaling(&Vector2::new(bounds.x / 2.0, bounds.y / -2.0))
            .append_translation(&Vector2::new(bounds.x / 2.0, bounds.y / 2.0));

    let perspective = Perspective3::new(1.0, compute_solution.field_of_view, 0.01, 10.0);

    let mut matrix = perspective.into_inner();
    *matrix.index_mut((0, 2)) = -compute_solution.ortho_center.x;
    *matrix.index_mut((1, 2)) = -compute_solution.ortho_center.y;

    let transform = matrix * compute_solution.view_transform;
    let point = Point3::from(*location3d);

    let point = transform * point.to_homogeneous();
    let point = Point3::from_homogeneous(point)?;

    Some(dc_to_image.transform_point(&point.xy()).coords)
}

#[derive(Clone)]
pub struct ComputeSolutionLocal {
    pub view_transform: Matrix4<f64>,
    pub ortho_center: Vector2<f64>,
    pub focal_length: f64,
    pub field_of_view: f64,
}

impl ComputeSolutionLocal {
    pub fn new(
        view_transform: Matrix4<f64>,
        ortho_center: Vector2<f64>,
        focal_length: f64,
    ) -> Self {
        Self {
            view_transform,
            ortho_center,
            focal_length,
            field_of_view: 2.0 * (1.0 / focal_length).atan(),
        }
    }
}
pub fn compute_camera_pose_translation_local(
    mut compute_solution: ComputeSolutionLocal,
    translate_origin: &Vector3<f64>,
) -> Result<ComputeSolutionLocal> {
    compute_solution.view_transform *=
        Matrix4::new_translation(&(Vector3::zeros() - translate_origin));
    Ok(compute_solution)
}
pub fn compute_camera_pose_scale_local(
    mut compute_solution: ComputeSolutionLocal,
    scale: f64,
) -> Result<ComputeSolutionLocal> {
    compute_solution.view_transform *= Matrix4::new_scaling(scale);
    Ok(compute_solution)
}
pub fn compute_camera_pose_local(
    vanishing_points: &[Vector2<f64>],
    user_selected_origin: &Vector2<f64>,
    axis: Matrix3<f64>,
) -> Result<ComputeSolutionLocal> {
    let ortho_center = triangle_ortho_center_local(
        &vanishing_points[0],
        &vanishing_points[1],
        &vanishing_points[2],
    );

    let focal_length = (ortho_center - vanishing_points[0])
        .dot(&(ortho_center - vanishing_points[1]))
        .abs()
        .sqrt();

    let x_rotation = vanishing_points[0] - ortho_center;
    let x_rotation = Vector3::new(x_rotation.x, x_rotation.y, -focal_length).normalize();
    let y_rotation = vanishing_points[1] - ortho_center;
    let y_rotation = Vector3::new(y_rotation.x, y_rotation.y, -focal_length).normalize();
    let z_rotation = vanishing_points[2] - ortho_center;
    let z_rotation = Vector3::new(z_rotation.x, z_rotation.y, -focal_length).normalize();
    //let z_rotation = x_rotation.cross(&y_rotation);
    let rotation_matrix = Matrix3::from_columns(&[x_rotation, y_rotation, z_rotation]);

    let view_transform = rotation_matrix * axis;
    let mut view_transform = view_transform.to_homogeneous();

    let mut origin3d: Vector3<f64> = (user_selected_origin - ortho_center).to_homogeneous();
    origin3d.z = -focal_length;
    origin3d /= focal_length;
    // apply default scale
    origin3d *= 10.0;
    view_transform.append_translation_mut(&origin3d);

    //let model_view_projection = matrix * translation * view_transform;
    //trace!("model_view_projection: {model_view_projection}");
    //let unproject_matrix = model_view_projection.try_inverse().unwrap();
    //trace!("unproject_matrix: {unproject_matrix}");

    // to ckeck in blender
    // bpy.data.objects["<name>.fspy"].matrix_world
    Ok(ComputeSolutionLocal::new(
        view_transform,
        ortho_center,
        focal_length,
    ))
}
pub fn find_vanishing_point_for_lines_local(
    a: &Vector2<f64>,
    b: &Vector2<f64>,
    c: &Vector2<f64>,
    d: &Vector2<f64>,
) -> Vector2<f64> {
    let x1 = a.x;
    let x2 = b.x;
    let x3 = c.x;
    let x4 = d.x;
    let y1 = a.y;
    let y2 = b.y;
    let y3 = c.y;
    let y4 = d.y;
    let t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4))
        / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    Vector2::new(x1 + t * (x2 - x1), y1 + t * (y2 - y1))
}
pub fn relative_to_image_plane_local(ratio: f64, image_point: &Vector2<f64>) -> Vector2<f64> {
    let transform = Matrix3::new_nonuniform_scaling(&Vector2::new(2.0, -2.0 / ratio))
        .append_translation(&Vector2::new(-1.0, 1.0 / ratio));
    let point = Point2::from(*image_point).to_homogeneous();
    Point2::from_homogeneous(transform * point).unwrap().coords
}

pub fn triangle_ortho_center_local(
    x: &Vector2<f64>,
    y: &Vector2<f64>,
    z: &Vector2<f64>,
) -> Vector2<f64> {
    let a = x.x;
    let b = x.y;
    let c = y.x;
    let d = y.y;
    let e = z.x;
    let f = z.y;

    let n = b * c + d * e + f * a - c * f - b * e - a * d;
    let x = ((d - f) * b * b
        + (f - b) * d * d
        + (b - d) * f * f
        + a * b * (c - e)
        + c * d * (e - a)
        + e * f * (a - c))
        / n;
    let y = ((e - c) * a * a
        + (a - e) * c * c
        + (c - a) * e * e
        + a * b * (f - d)
        + c * d * (b - f)
        + e * f * (d - b))
        / n;
    Vector2::new(x, y)
}
