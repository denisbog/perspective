use iced::{Point, Size, Vector};

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
