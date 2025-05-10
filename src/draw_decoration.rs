use iced::{
    Color, Point,
    advanced::graphics::geometry,
    widget::canvas::{self, Frame},
};
use nalgebra::{Matrix3, Matrix4, Point3, Vector2};

use iced::Rectangle;
use iced::widget::canvas::Text;
use iced::widget::canvas::stroke::Stroke;
use num_traits::ToPrimitive;

use crate::{
    Edit,
    compute::{find_vanishing_point_for_lines, triangle_ortho_center},
    utils::{relative_to_image_plane, scale_point_to_canvas, to_canvas},
};

pub fn draw_vanishing_points<Renderer>(
    control_point: &Point,
    axis_lines: &[(Point, Point)],
    edit: &Edit,
    bounds: Rectangle,
    frame: &mut Frame<Renderer>,
) -> (Vector2<f32>, Vector2<f32>, Vector2<f32>)
where
    Renderer: geometry::Renderer,
{
    let mut builder = canvas::path::Builder::new();
    builder.circle(scale_point_to_canvas(control_point, bounds.size()), 3f32);

    let vanishing_point_x = find_vanishing_point_for_lines(
        &Vector2::new(axis_lines[0].0.x, axis_lines[0].0.y),
        &Vector2::new(axis_lines[0].1.x, axis_lines[0].1.y),
        &Vector2::new(axis_lines[1].0.x, axis_lines[1].0.y),
        &Vector2::new(axis_lines[1].1.x, axis_lines[1].1.y),
    );
    let vanishing_point_y = find_vanishing_point_for_lines(
        &Vector2::new(axis_lines[2].0.x, axis_lines[2].0.y),
        &Vector2::new(axis_lines[2].1.x, axis_lines[2].1.y),
        &Vector2::new(axis_lines[3].0.x, axis_lines[3].0.y),
        &Vector2::new(axis_lines[3].1.x, axis_lines[3].1.y),
    );
    let vanishing_point_z = find_vanishing_point_for_lines(
        &Vector2::new(axis_lines[4].0.x, axis_lines[4].0.y),
        &Vector2::new(axis_lines[4].1.x, axis_lines[4].1.y),
        &Vector2::new(axis_lines[5].0.x, axis_lines[5].0.y),
        &Vector2::new(axis_lines[5].1.x, axis_lines[5].1.y),
    );

    builder.move_to(scale_point_to_canvas(control_point, bounds.size()));
    builder.line_to(scale_point_to_canvas(
        &Point::new(vanishing_point_x.x, vanishing_point_x.y),
        bounds.size(),
    ));
    builder.move_to(scale_point_to_canvas(control_point, bounds.size()));
    builder.line_to(scale_point_to_canvas(
        &Point::new(vanishing_point_y.x, vanishing_point_y.y),
        bounds.size(),
    ));
    builder.move_to(scale_point_to_canvas(control_point, bounds.size()));
    builder.line_to(scale_point_to_canvas(
        &Point::new(vanishing_point_z.x, vanishing_point_z.y),
        bounds.size(),
    ));
    let path = builder.build();
    let style = if let Edit::ControlPoint(_) = edit {
        canvas::Style::Solid(Color::from_rgba(8.0, 6.0, 5.0, 0.4))
    } else {
        canvas::Style::Solid(Color::from_rgba(8.0, 6.0, 0.0, 0.4))
    };
    frame.stroke(
        &path,
        Stroke {
            style,
            width: 1.0,
            ..Stroke::default()
        },
    );

    //   let ortho_center =
    //       triangle_ortho_center(&vanishing_point_x, &vanishing_point_y, &vanishing_point_z);
    //   trace!("{:?}", ortho_center);
    let ratio = bounds.width / bounds.height;
    let ortho_center = triangle_ortho_center(
        &relative_to_image_plane(ratio, &vanishing_point_x),
        &relative_to_image_plane(ratio, &vanishing_point_y),
        &relative_to_image_plane(ratio, &vanishing_point_z),
    );
    let ortho_center = to_canvas(bounds.size(), &ortho_center);
    let yellow = Color::from_rgba(0.8, 0.8, 0.2, 0.8);

    let mut builder = canvas::path::Builder::new();

    let point = Point::new(ortho_center.x, ortho_center.y);
    builder.circle(point, 5.0);
    builder.move_to(point);

    let point = Point::new(bounds.size().width / 2.0, bounds.size().height / 2.0);
    builder.line_to(point);
    builder.circle(point, 3.0);
    let path = builder.build();
    frame.stroke(
        &path,
        Stroke {
            style: canvas::Style::Solid(yellow),
            width: 2.0,
            ..Stroke::default()
        },
    );

    (vanishing_point_x, vanishing_point_y, vanishing_point_z)
}

pub fn draw_grid_for_origin<Renderer>(
    frame: &mut Frame<Renderer>,
    color_red: Color,
    transform: Matrix4<f32>,
    dc_to_image: Matrix3<f32>,
) where
    Renderer: geometry::Renderer,
{
    let mut builder = canvas::path::Builder::new();
    for j in -35..=35 {
        for i in -35..=35 {
            if i % 5 != 0 && j % 5 != 0 {
                continue;
            }
            let point =
                nalgebra::Point3::new(0.2 * i.to_f32().unwrap(), 0.2 * j.to_f32().unwrap(), 0.0);

            let point = transform * point.to_homogeneous();
            let point = Point3::from_homogeneous(point).unwrap();
            let center = dc_to_image.transform_point(&point.xy());
            builder.circle(Point::new(center.x, center.y), 1f32);
        }
    }
    let path = builder.build();
    frame.stroke(
        &path,
        Stroke {
            style: canvas::Style::Solid(color_red),
            width: 2.0,
            ..Stroke::default()
        },
    );
}

pub fn draw_origin_with_axis<Renderer>(
    frame: &mut Frame<Renderer>,
    color_red: Color,
    color_green: Color,
    color_blue: Color,
    transform: Matrix4<f32>,
    dc_to_image: Matrix3<f32>,
) where
    Renderer: geometry::Renderer,
{
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
    ];

    let points = points
        .iter()
        .map(|point| {
            let point = transform.transform_point(point);
            let point = dc_to_image.transform_point(&point.xy());
            Point::new(point.x, point.y)
        })
        .collect::<Vec<Point<f32>>>();
    let center = points.first().unwrap();

    let mut builder = canvas::path::Builder::new();

    builder.move_to(*center);
    frame.fill_text(Text {
        content: "O".to_string(),
        position: *center,
        ..Default::default()
    });

    builder.move_to(*center);
    let point = points.get(1).unwrap();
    builder.line_to(*point);
    //x axis

    frame.fill_text(Text {
        content: "X".to_string(),
        position: *point,
        color: color_red,
        ..Default::default()
    });

    let path = builder.build();
    frame.stroke(
        &path,
        Stroke {
            style: canvas::Style::Solid(color_red),
            width: 3.0,
            ..Stroke::default()
        },
    );
    //y axis

    let mut builder = canvas::path::Builder::new();
    // axis

    builder.move_to(*center);
    let point = points.get(2).unwrap();
    builder.line_to(*point);
    frame.fill_text(Text {
        content: "Y".to_string(),
        position: *point,
        color: color_green,
        ..Default::default()
    });

    let path = builder.build();
    frame.stroke(
        &path,
        Stroke {
            style: canvas::Style::Solid(color_green),
            width: 3.0,
            ..Stroke::default()
        },
    );

    let mut builder = canvas::path::Builder::new();

    //y axis
    builder.move_to(*center);
    let point = points.get(3).unwrap();
    builder.line_to(*point);
    frame.fill_text(Text {
        content: "Z".to_string(),
        position: *point,
        color: color_blue,
        ..Default::default()
    });

    let path = builder.build();
    frame.stroke(
        &path,
        Stroke {
            style: canvas::Style::Solid(color_blue),
            width: 3.0,
            ..Stroke::default()
        },
    );
}
