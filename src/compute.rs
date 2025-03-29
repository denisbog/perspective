use std::{fs::File, io::Read};

use anyhow::Result;
use iced::Point;
use nalgebra::{Matrix3, Matrix4, Perspective3, Point2, Point3, RowVector3, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_util::{bytes::BytesMut, codec::Encoder};
use tracing::trace;

use crate::{
    FSpyData, SceneSettings, encoder::FSpyEncoder, fspy::compute_solution_to_scene_settings,
};

#[derive(Serialize, Deserialize)]
pub struct StorePoint {
    pub x: f32,
    pub y: f32,
}
#[derive(Serialize, Deserialize)]
pub struct Lines {
    pub control_point: StorePoint,
    pub scale: StoreLine,
    pub lines: Vec<StoreLine>,
    pub points: Option<Vec<StorePoint3d>>,
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

pub fn read_points_from_file(
    points: &String,
) -> (
    Point,
    (Point, Point),
    Vec<(Point, Point)>,
    Option<Vec<Vector3<f32>>>,
) {
    trace!("reading points from file {points}");
    let mut file = File::open(points).unwrap();
    let mut content = String::new();
    file.read_to_string(&mut content).unwrap();
    let data: Lines = serde_json::from_str(&content).unwrap();
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

    let scale = (
        Point {
            x: data.scale.a.x,
            y: data.scale.a.y,
        },
        Point {
            x: data.scale.b.x,
            y: data.scale.b.y,
        },
    );

    let control_point = Point {
        x: data.control_point.x,
        y: data.control_point.y,
    };

    let points = data.points.map(|item| {
        item.iter()
            .map(|point| Vector3::new(point.x, point.y, point.z))
            .collect()
    });
    (control_point, scale, lines, points)
}

pub fn adaptor_compute_solution_to_scene_settings(
    image_width: u32,
    image_height: u32,
    compute_solution: &ComputeSolution,
) -> Result<SceneSettings> {
    compute_solution_to_scene_settings(image_width, image_height, compute_solution)
}

pub async fn store_scene_data_to_file(
    compute_solution: &ComputeSolution,
    image_width: u32,
    image_height: u32,
    image_path: String,
    export_file_name: String,
) -> Result<SceneSettings> {
    let mut image_file = tokio::fs::File::open(image_path).await.unwrap();
    let mut contents = vec![];
    image_file.read_to_end(&mut contents).await.unwrap();
    let data =
        adaptor_compute_solution_to_scene_settings(image_width, image_height, compute_solution)
            .unwrap();
    let to_export = FSpyData {
        data: data.clone(),
        image: contents,
    };

    let mut encoder = FSpyEncoder::default();

    let mut dst = BytesMut::with_capacity(4096);
    encoder.encode(to_export, &mut dst)?;

    let mut repackage_file = tokio::fs::File::create(export_file_name).await.unwrap();
    repackage_file.write_all(&dst).await.unwrap();
    Ok(data)
}
pub async fn compute_adapter(
    x_lines: [(Point, Point); 2],
    y_lines: [(Point, Point); 2],
    z_lines: [(Point, Point); 2],
    image_width: u32,
    image_height: u32,
    control_point: &Point,
    scale: &(Point, Point),
) -> Result<ComputeSolution> {
    let points: [Vector2<f32>; 12] = [
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
    let scale = [
        Vector2::new(scale.0.x, scale.0.y),
        Vector2::new(scale.1.x, scale.1.y),
    ];
    let control_point: Vector2<f32> = Vector2::new(control_point.x, control_point.y);
    compute_camera_pose(
        &points,
        image_width as f32 / image_height as f32,
        &control_point,
        &scale,
    )
    .await
}

pub async fn compute_camera_pose(
    points: &[Vector2<f32>; 12],
    ratio: f32,
    user_selected_origin: &Vector2<f32>,
    handle_position: &[Vector2<f32>; 2],
) -> Result<ComputeSolution> {
    let vanishing_points = points
        .chunks(4)
        .map(|lines| find_vanishing_point_for_lines(&lines[0], &lines[1], &lines[2], &lines[3]))
        .collect::<Vec<Vector2<f32>>>();

    let vanishing_points = vanishing_points
        .iter()
        .map(|point| relative_to_image_plane(ratio, point))
        .collect::<Vec<Vector2<f32>>>();
    trace!("vanishing point {:?}", vanishing_points);

    let ortho_center = triangle_ortho_center(
        &vanishing_points[0],
        &vanishing_points[1],
        &vanishing_points[2],
    );

    // let ortho_center = relative_to_image_plane_new(ratio, &ortho_center);
    // axis
    trace!("ortho center {ortho_center}");

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

    let focal_length = ((vanishing_points[0] - ortho_center)
        .dot(&(ortho_center - vanishing_points[1])))
    .abs()
    .sqrt();
    trace!("focal length: {focal_length}");

    let x_rotation = vanishing_points[0] - ortho_center;
    let x_rotation = Vector3::new(x_rotation.x, x_rotation.y, -focal_length).normalize();
    let y_rotation = vanishing_points[1] - ortho_center;
    let y_rotation = Vector3::new(y_rotation.x, y_rotation.y, -focal_length).normalize();
    let z_rotation = vanishing_points[2] - ortho_center;
    let z_rotation = Vector3::new(z_rotation.x, z_rotation.y, -focal_length).normalize();
    //let z_rotation = x_rotation.cross(&y_rotation);
    let rotation_matrix = Matrix3::from_columns(&[x_rotation, y_rotation, z_rotation]);
    trace!("rotation matrix: {rotation_matrix}");
    let axis = Matrix3::from_rows(&[
        RowVector3::new(1.0, 0.0, 0.0),
        RowVector3::new(0.0, -1.0, 0.0),
        RowVector3::new(0.0, 0.0, -1.0),
    ]);

    let view_transform = rotation_matrix * axis;
    let view_transform = view_transform.to_homogeneous();
    trace!("view transform: {view_transform}");

    let user_selected_origin = relative_to_image_plane(ratio, user_selected_origin);

    let mut origin3d: Vector3<f32> = (user_selected_origin - ortho_center).to_homogeneous();
    origin3d.z = -focal_length;
    origin3d /= focal_length;
    // apply default scale
    origin3d *= 10.0;
    trace!("origin3d: {origin3d}");

    let projection = Perspective3::new(1.0, 2.0 * (1.0 / focal_length).atan(), 0.01, 10.0);
    trace!("projection: {:#?}", projection);
    let mut matrix = projection.into_inner();
    *matrix.index_mut((0, 2)) = -ortho_center.x;
    *matrix.index_mut((1, 2)) = -ortho_center.y;
    trace!("matrix: {matrix}");

    let translation = Matrix4::new_translation(&origin3d);
    //let model_view_projection = matrix * translation * view_transform;
    //trace!("model_view_projection: {model_view_projection}");
    //let unproject_matrix = model_view_projection.try_inverse().unwrap();
    //trace!("unproject_matrix: {unproject_matrix}");

    let handle_position = handle_position
        .iter()
        .map(|point| relative_to_image_plane(ratio, point))
        .collect::<Vec<Vector2<f32>>>();

    let handle_position_a = handle_position[0];
    let handle_position_b = handle_position[1];

    let point3d = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(user_selected_origin.x, user_selected_origin.y, 1.0),
        Vector3::new(handle_position_a.x, handle_position_a.y, 1.0),
        Vector3::new(handle_position_b.x, handle_position_b.y, 1.0),
    ];

    let point3d = point3d
        .iter()
        .map(|&point| {
            let temp = view_transform.try_inverse().unwrap()
                * translation.try_inverse().unwrap()
                * matrix.try_inverse().unwrap()
                * Point3::from(point).to_homogeneous();
            Point3::from_homogeneous(temp).unwrap().coords

            // Point3::from_homogeneous(unproject_matrix * Point3::from(point).to_homogeneous())
            //     .unwrap()
            //     .coords
        })
        .collect::<Vec<Vector3<f32>>>();
    trace!("point3d {:#?}", point3d);

    let intersection1_3d = line_insert_with_x_axis(&point3d[0], &point3d[1]);
    trace!("intersection3d: {intersection1_3d}");
    let intersection2_3d = intersection1_3d + Vector3::new(1.0, 0.0, 0.0);

    let distance = point3d[2..]
        .iter()
        .map(|point| {
            find_distrance_between_lines(&point3d[0], point, &intersection1_3d, &intersection2_3d)
        })
        .collect::<Vec<(Vector3<f32>, Vector3<f32>)>>();
    trace!("distance: {:#?}", distance);
    let distance = (distance[0].0 - distance[1].0).norm();

    let view_transform = view_transform.append_translation(&(origin3d / distance));

    // let view_transform = view_transform * Matrix4::new_translation(&Vector3::new(1.0, 2.0, 0.0));
    trace!("view transform: {view_transform}");
    // to ckeck in blender
    // bpy.data.objects["<name>.fspy"].matrix_world
    Ok(ComputeSolution::new(
        view_transform,
        ortho_center,
        focal_length,
    ))
}

pub fn find_vanishing_point_for_lines(
    a: &Vector2<f32>,
    b: &Vector2<f32>,
    c: &Vector2<f32>,
    d: &Vector2<f32>,
) -> Vector2<f32> {
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
pub fn triangle_ortho_center(x: &Vector2<f32>, y: &Vector2<f32>, z: &Vector2<f32>) -> Vector2<f32> {
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

pub fn line_insert_with_x_axis(
    control_point_a3d: &Vector3<f32>,
    control_point_b3d: &Vector3<f32>,
) -> Vector3<f32> {
    let axis = Vector3::new(1.0, 0.0, 0.0);
    line_insert_with_axis(&axis, control_point_a3d, control_point_b3d)
}

pub fn line_insert_with_z_axis(
    control_point_a3d: &Vector3<f32>,
    control_point_b3d: &Vector3<f32>,
) -> Vector3<f32> {
    let axis = Vector3::new(0.0, 0.0, 1.0);
    line_insert_with_axis(&axis, control_point_a3d, control_point_b3d)
}
pub fn line_insert_with_axis(
    axis: &Vector3<f32>,
    control_point_a3d: &Vector3<f32>,
    control_point_b3d: &Vector3<f32>,
) -> Vector3<f32> {
    let t = axis.dot(control_point_a3d) / -(control_point_b3d - control_point_a3d).dot(axis);
    control_point_a3d + (control_point_b3d - control_point_a3d) * t
}

// https://paulbourke.net/geometry/pointlineplane/lineline.c
pub fn find_distrance_between_lines(
    p1: &Vector3<f32>,
    p2: &Vector3<f32>,
    p3: &Vector3<f32>,
    p4: &Vector3<f32>,
) -> (Vector3<f32>, Vector3<f32>) {
    let p13 = p1 - p3;
    let p43 = p4 - p3;
    let p21 = p2 - p1;

    let d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
    let d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
    let d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
    let d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
    let d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

    let denom = d2121 * d4343 - d4321 * d4321;
    let numer = d1343 * d4321 - d1321 * d4343;

    let mua = numer / denom;
    let mub = (d1343 + d4321 * (mua)) / d4343;

    let pa = p1 + p21 * mua;
    let pb = p3 + p43 * mub;

    (pa, pb)
}

pub struct ComputeSolution {
    pub view_transform: Matrix4<f32>,
    pub ortho_center: Vector2<f32>,
    pub focal_length: f32,
    pub field_of_view: f32,
}

impl ComputeSolution {
    fn new(view_transform: Matrix4<f32>, ortho_center: Vector2<f32>, focal_length: f32) -> Self {
        Self {
            view_transform,
            ortho_center,
            focal_length,
            field_of_view: 2.0 * (1.0 / focal_length).atan(),
        }
    }
}
/// translate and scale to image space where center of the image is 0,0
pub fn relative_to_image_plane(ratio: f32, image_point: &Vector2<f32>) -> Vector2<f32> {
    let transform = Matrix3::new_nonuniform_scaling(&Vector2::new(2.0, -2.0 / ratio))
        .append_translation(&Vector2::new(-1.0, 1.0 / ratio));
    let point = Point2::from(*image_point).to_homogeneous();
    Point2::from_homogeneous(transform * point).unwrap().coords
}
