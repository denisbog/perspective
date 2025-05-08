use std::{
    fs::File,
    io::Read,
    ops::{AddAssign, DivAssign, MulAssign, SubAssign},
};

use anyhow::Result;
use iced::{Point, Size};
use nalgebra::{
    ComplexField, Matrix3, Matrix4, RowVector3, Scalar, SimdComplexField, Vector2, Vector3,
};
use num_traits::Float;
use serde::{Deserialize, Serialize};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_util::{bytes::BytesMut, codec::Encoder};

use crate::{
    AxisData, FSpyData, SceneSettings, encoder::FSpyEncoder,
    fspy::compute_solution_to_scene_settings, utils::relative_to_image_plane,
};

#[derive(Serialize, Deserialize)]
pub struct StorePoint {
    pub x: f32,
    pub y: f32,
}
#[derive(Serialize, Deserialize)]
pub struct Lines {
    pub control_point: StorePoint,
    pub lines: Vec<StoreLine>,
    pub points: Option<Vec<StorePoint3d>>,
    pub flip: Option<[bool; 3]>,
    pub custom_origin_tanslation: Option<StorePoint3d>,
    pub custom_scale: Option<f32>,
}
#[derive(Serialize, Deserialize)]
pub struct StoreLine {
    pub a: StorePoint,
    pub b: StorePoint,
}
#[derive(Serialize, Deserialize)]
pub struct StorePoint3d {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
impl From<&(Point, Point)> for StoreLine {
    fn from((p1, p2): &(Point<f32>, Point<f32>)) -> Self {
        Self {
            a: StorePoint { x: p1.x, y: p1.y },
            b: StorePoint { x: p2.x, y: p2.y },
        }
    }
}

pub fn read_points_from_file(points: &String) -> Result<(AxisData, Option<Vec<Vector3<f32>>>)> {
    let mut file = File::open(points)?;
    let mut content = String::new();
    file.read_to_string(&mut content)?;
    let data: Lines = serde_json::from_str(&content)?;
    let lines = data
        .lines
        .iter()
        .map(|item| {
            (
                Point {
                    x: item.a.x,
                    y: item.a.y,
                },
                Point {
                    x: item.b.x,
                    y: item.b.y,
                },
            )
        })
        .collect();

    let control_point = Point {
        x: data.control_point.x,
        y: data.control_point.y,
    };

    let points = data.points.map(|item| {
        item.iter()
            .map(|point| Vector3::new(point.x, point.y, point.z))
            .collect()
    });

    let flip = if let Some(flip) = data.flip {
        (flip[0], flip[1], flip[2])
    } else {
        (false, false, false)
    };

    let custom_origin_translation = data
        .custom_origin_tanslation
        .map(|item| Vector3::new(item.x, item.y, item.z));

    let custom_scale = data.custom_scale;

    Ok((
        AxisData {
            control_point,
            axis_lines: lines,
            flip,
            custom_origin_translation,
            custom_scale,
        },
        points,
    ))
}

pub fn adaptor_compute_solution_to_scene_settings<T: Float + ComplexField + Into<f32>>(
    image_width: u32,
    image_height: u32,
    compute_solution: &ComputeSolution<T>,
) -> Result<SceneSettings> {
    compute_solution_to_scene_settings(image_width, image_height, compute_solution)
}

pub async fn store_scene_data_to_file<T: Float + ComplexField + Into<f32>>(
    compute_solution: &ComputeSolution<T>,
    image_width: u32,
    image_height: u32,
    image_path: String,
    export_file_name: String,
) -> Result<SceneSettings> {
    let mut image_file = tokio::fs::File::open(image_path).await?;
    let mut contents = vec![];
    image_file.read_to_end(&mut contents).await?;
    let data =
        adaptor_compute_solution_to_scene_settings(image_width, image_height, compute_solution)?;
    let to_export = FSpyData {
        data: data.clone(),
        image: contents,
    };

    let mut encoder = FSpyEncoder::default();

    let mut dst = BytesMut::with_capacity(4096);
    encoder.encode(to_export, &mut dst)?;

    let mut repackage_file = tokio::fs::File::create(export_file_name).await?;
    repackage_file.write_all(&dst).await?;
    Ok(data)
}
pub fn compute_ui_adapter<
    T: Float + SubAssign + MulAssign + DivAssign + AddAssign + ComplexField + Scalar,
>(
    x_lines: [(Point<T>, Point<T>); 2],
    y_lines: [(Point<T>, Point<T>); 2],
    z_lines: [(Point<T>, Point<T>); 2],
    image_size: Size<T>,
    control_point: &Point<T>,
    flip: (bool, bool, bool),
    translate_origin: &Option<Vector3<T>>,
    scale: &Option<T>,
) -> Result<ComputeSolution<T>> {
    let points: [Vector2<T>; 12] = [
        Vector2::new(x_lines[0].0.x, x_lines[0].0.y),
        Vector2::new(x_lines[0].1.x, x_lines[0].1.y),
        Vector2::new(x_lines[1].0.x, x_lines[1].0.y),
        Vector2::new(x_lines[1].1.x, x_lines[1].1.y),
        Vector2::new(y_lines[0].0.x, y_lines[0].0.y),
        Vector2::new(y_lines[0].1.x, y_lines[0].1.y),
        Vector2::new(y_lines[1].0.x, y_lines[1].0.y),
        Vector2::new(y_lines[1].1.x, y_lines[1].1.y),
        Vector2::new(z_lines[0].0.x, z_lines[0].0.y),
        Vector2::new(z_lines[0].1.x, z_lines[0].1.y),
        Vector2::new(z_lines[1].0.x, z_lines[1].0.y),
        Vector2::new(z_lines[1].1.x, z_lines[1].1.y),
    ];
    let control_point: Vector2<T> = Vector2::new(control_point.x, control_point.y);

    let x = if flip.0 { 1.0 } else { -1.0 };
    let y = if flip.1 { 1.0 } else { -1.0 };
    let z = if flip.2 { 1.0 } else { -1.0 };
    let axis = Matrix3::from_rows(&[
        RowVector3::new(
            T::from(x).unwrap(),
            T::from(0.0).unwrap(),
            T::from(0.0).unwrap(),
        ),
        RowVector3::new(
            T::from(0.0).unwrap(),
            T::from(y).unwrap(),
            T::from(0.0).unwrap(),
        ),
        RowVector3::new(
            T::from(0.0).unwrap(),
            T::from(0.0).unwrap(),
            T::from(z).unwrap(),
        ),
    ]);

    let ratio = image_size.width / image_size.height;
    let user_selected_origin = relative_to_image_plane(ratio, &control_point);

    let vanishing_points = points
        .chunks(4)
        .map(|lines| find_vanishing_point_for_lines(&lines[0], &lines[1], &lines[2], &lines[3]))
        .collect::<Vec<Vector2<T>>>();

    let vanishing_points = vanishing_points
        .iter()
        .map(|point| relative_to_image_plane(ratio, point))
        .collect::<Vec<Vector2<T>>>();

    let compute_solution = compute_camera_pose(&vanishing_points, &user_selected_origin, axis);

    let compute_solution = if let Ok(compute_solution) = compute_solution {
        if let Some(scale) = scale {
            compute_camera_pose_scale(compute_solution, *scale)
        } else {
            Ok(compute_solution)
        }
    } else {
        compute_solution
    };
    if let Some(translate_origin) = translate_origin {
        if let Ok(compute_solution) = compute_solution {
            compute_camera_pose_translation(compute_solution, translate_origin)
        } else {
            compute_solution
        }
    } else {
        compute_solution
    }
}

pub fn compute_camera_pose_scale<T: Float + MulAssign + AddAssign + Scalar>(
    mut compute_solution: ComputeSolution<T>,
    scale: T,
) -> Result<ComputeSolution<T>> {
    compute_solution.view_transform *= Matrix4::new_scaling(scale);
    Ok(compute_solution)
}

pub fn compute_camera_pose_translation<T: Float + AddAssign + SubAssign + MulAssign + Scalar>(
    mut compute_solution: ComputeSolution<T>,
    translate_origin: &Vector3<T>,
) -> Result<ComputeSolution<T>> {
    compute_solution.view_transform *=
        Matrix4::new_translation(&(Vector3::zeros() - translate_origin));
    Ok(compute_solution)
}
pub fn compute_camera_pose<
    T: Float
        + std::ops::SubAssign
        + AddAssign
        + MulAssign
        + SimdComplexField
        + DivAssign
        + MulAssign
        + Scalar
        + 'static,
>(
    vanishing_points: &[Vector2<T>],
    user_selected_origin: &Vector2<T>,
    axis: Matrix3<T>,
) -> Result<ComputeSolution<T>> {
    let ortho_center = triangle_ortho_center(
        &vanishing_points[0],
        &vanishing_points[1],
        &vanishing_points[2],
    );

    // let ortho_center = relative_to_image_plane_new(ratio, &ortho_center);
    // axis

    //let minimizer = GradientDescent::new();
    //let solution = minimizer
    //    .gradient_tolerance(1e-7)
    //    .max_iterations(Some(12))
    //    .minimize(
    //        &NumericalDifferentiation::new(Func(|x: &[f32]| {
    //            let ovx = Vector3::new(
    //                vanishing_points[0].x - ortho_center.x,
    //                vanishing_points[0].y - ortho_center.y,
    //                -x[0],
    //            );
    //            let ovy = Vector3::new(
    //                vanishing_points[1].x - ortho_center.x,
    //                vanishing_points[1].y - ortho_center.y,
    //                -x[0],
    //            );
    //            let out = (ovx.dot(&ovy) / ovx.norm() / ovy.norm()).abs();
    //            out
    //        })),
    //        vec![1.0],
    //    );
    //trace!("optimized focal length: {:?}", solution);
    //let focal_length = solution.position()[0];

    let focal_length = (ortho_center - vanishing_points[1])
        .dot(&(ortho_center - vanishing_points[2]))
        .abs()
        .sqrt();

    let x_rotation = vanishing_points[0] - ortho_center;
    let x_rotation = Vector3::new(x_rotation.x, x_rotation.y, -focal_length).normalize();
    let y_rotation = vanishing_points[1] - ortho_center;
    let y_rotation = Vector3::new(y_rotation.x, y_rotation.y, -focal_length).normalize();
    let z_rotation = vanishing_points[2] - ortho_center;
    let z_rotation = Vector3::new(z_rotation.x, z_rotation.y, -focal_length).normalize();
    //let x_rotation = y_rotation.cross(&z_rotation);
    //let z_rotation = x_rotation.cross(&y_rotation);
    let rotation_matrix = Matrix3::from_columns(&[x_rotation, y_rotation, z_rotation]);

    let view_transform = rotation_matrix * axis;
    let mut view_transform = view_transform.to_homogeneous();

    let mut origin3d: Vector3<T> = (user_selected_origin - ortho_center).to_homogeneous();
    origin3d.z = -focal_length;
    origin3d /= focal_length;
    // apply default scale
    origin3d *= T::from(10.0).unwrap();
    view_transform.append_translation_mut(&origin3d);

    //let model_view_projection = matrix * translation * view_transform;
    //trace!("model_view_projection: {model_view_projection}");
    //let unproject_matrix = model_view_projection.try_inverse().unwrap();
    //trace!("unproject_matrix: {unproject_matrix}");

    // to ckeck in blender
    // bpy.data.objects["<name>.fspy"].matrix_world
    Ok(ComputeSolution::new(
        view_transform,
        ortho_center,
        focal_length,
    ))
}

pub fn find_vanishing_point_for_lines<T: Float + Scalar + 'static>(
    a: &Vector2<T>,
    b: &Vector2<T>,
    c: &Vector2<T>,
    d: &Vector2<T>,
) -> Vector2<T> {
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
pub fn triangle_ortho_center<T: Float + Scalar + 'static>(
    x: &Vector2<T>,
    y: &Vector2<T>,
    z: &Vector2<T>,
) -> Vector2<T> {
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

#[derive(Clone)]
pub struct ComputeSolution<T> {
    pub view_transform: Matrix4<T>,
    pub ortho_center: Vector2<T>,
    pub focal_length: T,
    pub field_of_view: T,
}

impl<T: Float> ComputeSolution<T> {
    pub fn new(view_transform: Matrix4<T>, ortho_center: Vector2<T>, focal_length: T) -> Self {
        Self {
            view_transform,
            ortho_center,
            focal_length,
            field_of_view: T::from(2.0).unwrap() * (T::from(1.0).unwrap() / focal_length).atan(),
        }
    }
}
