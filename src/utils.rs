use std::ops::{AddAssign, DivAssign, MulAssign, SubAssign};

use iced::{Point, Size, Vector};
use nalgebra::{Matrix3, Perspective3, Point2, Point3, RealField, Scalar, Vector2, Vector3};
use num_traits::Float;

use crate::{EditAxis, compute::data::ComputeSolution};

pub fn check_if_control_point_is_clicked(control_point: Point, cursor: Point) -> bool {
    let error = control_point - cursor;
    let error = error.x.abs() + error.y.abs();
    error < 0.01f32
}

pub fn scale_point(point: Vector, size: Size) -> Point {
    Point {
        x: point.x / size.width,
        y: point.y / size.height,
    }
}
pub fn scale_point_to_canvas(point: &Point, size: Size) -> Point {
    Point {
        x: point.x * size.width,
        y: point.y * size.height,
    }
}
pub fn should_edit_point(clicked_position: Point, p1: Point) -> bool {
    let offset = 0.01f32;
    p1.x + offset > clicked_position.x
        && p1.x - offset < clicked_position.x
        && p1.y + offset > clicked_position.y
        && p1.y - offset < clicked_position.y
}

pub fn get_extension_for_line_within_bounds(
    (p1, p2): &(Point<f32>, Point<f32>),
    size: iced::Size,
) -> Option<Vec<Point>> {
    let a = (p2.y - p1.y) / (p2.x - p1.x);
    let c = p2.y - p2.x * a;
    let offset = 20f32;

    let out: Vec<Point> = vec![
        Point::new(offset, a * offset + c),
        Point::new(size.width - offset, a * (size.width - offset) + c),
        Point::new((offset - c) / a, offset),
        Point::new((size.height - offset - c) / a, size.height - offset),
    ]
    .into_iter()
    .filter(|item| {
        item.x >= offset
            && item.x <= size.width - offset
            && item.y >= offset
            && item.y <= size.height - offset
    })
    .collect();
    if out.len() > 1 { Some(out) } else { None }
}

pub fn check_if_point_is_from_line(
    line_point_a: &Point,
    line_point_b: &Point,
    point: Point,
) -> bool {
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    // Line defined by two points
    let error = ((line_point_b.y - line_point_a.y) * point.x
        - (line_point_b.x - line_point_a.x) * point.y
        + line_point_b.x * line_point_a.y
        - line_point_b.y * line_point_a.x)
        .abs()
        / ((line_point_b.y - line_point_a.y).powi(2) + (line_point_b.x - line_point_a.x).powi(2))
            .sqrt();
    error < 0.01f32
}

pub fn check_if_point_is_from_line_new(
    line_point_a: &Point,
    line_point_b: &Point,
    point: Point,
) -> bool {
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    // Line defined by two points
    let error = ((line_point_b.y - line_point_a.y) * point.x
        - (line_point_b.x - line_point_a.x) * point.y
        + line_point_b.x * line_point_a.y
        - line_point_b.y * line_point_a.x)
        .abs()
        / ((line_point_b.y - line_point_a.y).powi(2) + (line_point_b.x - line_point_a.x).powi(2))
            .sqrt();
    error < 3.0
}

/// translate and scale to image space where center of the image is 0,0
pub fn relative_to_image_plane<T: Float + AddAssign + MulAssign + DivAssign + Scalar + 'static>(
    ratio: T,
    image_point: &Vector2<T>,
) -> Vector2<T> {
    let transform = Matrix3::new_nonuniform_scaling(&Vector2::new(
        T::from(2.0).unwrap(),
        T::from(-2.0).unwrap() / ratio,
    ))
    .append_translation(&Vector2::new(
        -T::from(1.0).unwrap(),
        T::from(1.0).unwrap() / ratio,
    ));
    let point = Point2::from(*image_point).to_homogeneous();
    Point2::from_homogeneous(transform * point).unwrap().coords
}
// corner up left: 0,0; bottom right: size.width, size.height;
pub fn to_canvas<T: Float + AddAssign + MulAssign + DivAssign + Scalar + 'static>(
    bounds: Size<T>,
    image_point: &Vector2<T>,
) -> Vector2<T> {
    let transform = Matrix3::new_nonuniform_scaling(&Vector2::new(
        bounds.width / T::from(2.0).unwrap(),
        bounds.width / -T::from(2.0).unwrap(),
    ))
    .append_translation(&Vector2::new(
        bounds.width / T::from(2.0).unwrap(),
        bounds.height / T::from(2.0).unwrap(),
    ));
    let point = Point2::from(*image_point).to_homogeneous();
    Point2::from_homogeneous(transform * point).unwrap().coords
}

pub fn calculate_cursor_position_to_3d<
    T: Float + AddAssign + MulAssign + DivAssign + RealField + Scalar,
>(
    edit_state: &EditAxis,
    compute_solution: &ComputeSolution<T>,
    ratio: T,
    cursor_canvas: &Vector2<T>,
    last_point: Vector3<T>,
) -> Option<Vector3<T>> {
    let click_location = relative_to_image_plane(ratio, cursor_canvas);

    let perspective = Perspective3::new(
        T::from(1.0).unwrap(),
        compute_solution.field_of_view(),
        T::from(0.01).unwrap(),
        T::from(10.0).unwrap(),
    );

    let mut matrix = perspective.into_inner();
    *matrix.index_mut((0, 2)) = -compute_solution.ortho_center().x;
    *matrix.index_mut((1, 2)) = -compute_solution.ortho_center().y;

    let model_view_projection = matrix * compute_solution.view_transform();
    let model_view_projection = model_view_projection.try_inverse()?;
    let last_point_axis = Vector3::zeros();
    let point = model_view_projection * Point3::from(last_point_axis).to_homogeneous();
    let point3d1 = Point3::from_homogeneous(point)?;

    let point =
        Point3::new(click_location.x, click_location.y, T::from(1.0).unwrap()).to_homogeneous();
    let point = model_view_projection * point;

    let point3d2 = Point3::from_homogeneous(point)?;

    let axis = match edit_state {
        EditAxis::EditZ => Vector3::new(
            T::from(1.0).unwrap(),
            T::from(0.0).unwrap(),
            T::from(0.0).unwrap(),
        ),
        _ => Vector3::new(
            T::from(0.0).unwrap(),
            T::from(0.0).unwrap(),
            T::from(1.0).unwrap(),
        ),
    };

    let intersection1_3d =
        line_insert_with_plane(&last_point, &axis, &point3d1.coords, &point3d2.coords);
    Some(intersection1_3d)
}

pub fn line_insert_with_yz_plane(
    control_point_a3d: &Vector3<f32>,
    control_point_b3d: &Vector3<f32>,
) -> Vector3<f32> {
    let axis = Vector3::new(1.0, 0.0, 0.0);
    line_insert_with_axis(&axis, control_point_a3d, control_point_b3d)
}

pub fn line_insert_with_xy_plane(
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
    line_insert_with_plane(
        &Vector3::zeros(),
        axis,
        control_point_a3d,
        control_point_b3d,
    )
}

pub fn line_insert_with_plane<T: Float + AddAssign + MulAssign + SubAssign + Scalar>(
    plane_point: &Vector3<T>,
    normal_to_plane: &Vector3<T>,
    a: &Vector3<T>,
    b: &Vector3<T>,
) -> Vector3<T> {
    let t = normal_to_plane.dot(&(a - plane_point)) / -normal_to_plane.dot(&(b - a));
    a + (b - a) * t
}
