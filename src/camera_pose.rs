use std::{cell::RefCell, f32, marker::PhantomData, rc::Rc};

use iced::{
    Color, Element,
    Length::{self},
    Pixels, Point, Rectangle, Size, Vector,
    advanced::{
        Clipboard, Layout, Shell, Widget,
        graphics::geometry::{self},
        layout, mouse,
        renderer::Style,
        widget::{
            Tree,
            tree::{self},
        },
    },
    event::Status,
    mouse::ScrollDelta,
    widget::canvas::{self, Event, Fill, LineDash, Stroke, Text},
};
use nalgebra::{Matrix3, Perspective3, Point2, Vector2, Vector3};

use crate::{
    AxisData, Component, Edit,
    compute::{ComputeSolution, compute_ui_adapter},
    draw_decoration::{draw_grid_for_origin, draw_origin_with_axis, draw_vanishing_points},
    utils::{
        calculate_location_position_to_2d, check_if_control_point_is_clicked,
        check_if_point_is_from_line, get_extension_for_line_within_bounds, scale_point,
        scale_point_to_canvas, should_edit_point, to_canvas,
    },
};

#[derive(Debug, Clone)]
enum CameraPoseMessage {
    EditEndpoint { cursor: Point },
    HighlightLine { highlight: Option<usize> },
    Editline { component: Option<Component> },
    MoveControlPoint { cursor: Point },
}
pub struct ComputeCameraPose<'a, Message, Theme = iced::Theme, Renderer = iced::Renderer>
where
    Renderer: geometry::Renderer,
{
    width: Length,
    height: Length,
    message_: PhantomData<Message>,
    cache: geometry::Cache<Renderer>,
    axis_cache: geometry::Cache<Renderer>,
    draw_cache: geometry::Cache<Renderer>,

    compute_solution: &'a Option<ComputeSolution<f32>>,
    renderer_: PhantomData<Renderer>,
    theme_: PhantomData<Theme>,
    axis_data: Rc<RefCell<AxisData>>,
    image_size: Size<f32>,
    draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
}
impl<'a, M, Theme, Renderer> ComputeCameraPose<'a, M, Theme, Renderer>
where
    Renderer: geometry::Renderer,
{
    const DEFAULT_SIZE: f32 = 100.0;
    pub fn new(
        axis_data: Rc<RefCell<AxisData>>,
        draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
        compute_solution: &'a Option<ComputeSolution<f32>>,
    ) -> Self {
        ComputeCameraPose {
            width: Length::Fixed(Self::DEFAULT_SIZE),
            height: Length::Fixed(Self::DEFAULT_SIZE),
            compute_solution,
            axis_data,
            message_: PhantomData,
            renderer_: PhantomData,
            theme_: PhantomData,
            cache: geometry::Cache::default(),
            axis_cache: geometry::Cache::default(),
            draw_cache: geometry::Cache::default(),
            draw_lines,
            image_size: Size::default(),
        }
    }
    pub fn width(mut self, width: impl Into<Length>) -> Self {
        self.width = width.into();
        self
    }

    /// Sets the height of the [`Canvas`].
    pub fn height(mut self, height: impl Into<Length>) -> Self {
        self.height = height.into();
        self
    }

    pub fn image_size(mut self, image_size: Size) -> Self {
        self.image_size = image_size;
        self
    }

    fn handle_internal_event(&mut self, state: &mut State, message: CameraPoseMessage) {
        match message {
            CameraPoseMessage::HighlightLine { highlight } => {
                state.highlight = highlight;
                self.cache.clear();
            }
            CameraPoseMessage::Editline { component } => {
                if component.is_some() {
                } else {
                    self.cache.clear();
                }
                state.edit = component;
            }
            CameraPoseMessage::EditEndpoint { cursor } => {
                if let Some(component_to_edit) = &state.edit {
                    match component_to_edit {
                        Component::A => {
                            self.axis_data.borrow_mut().axis_lines[state.highlight.unwrap()].0 =
                                cursor;
                        }
                        Component::B => {
                            self.axis_data.borrow_mut().axis_lines[state.highlight.unwrap()].1 =
                                cursor;
                        }
                    };
                    self.cache.clear();
                    self.compute_pose(state);
                }
            }
            CameraPoseMessage::MoveControlPoint { cursor } => {
                self.axis_data.borrow_mut().control_point = cursor;
                self.cache.clear();
                self.compute_pose(state);
            }
        }
    }

    fn update_inner(
        &self,
        state: &mut State,
        event: &Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> (Status, Option<CameraPoseMessage>) {
        let Some(cursor) = cursor.position_over(bounds) else {
            return (Status::Ignored, None);
        };
        let cursor = cursor - bounds.position();
        let scale_cursor = scale_point(cursor, bounds.size());
        match event {
            Event::Mouse(mouse::Event::WheelScrolled {
                delta: ScrollDelta::Lines { x: _x, y },
            }) => {
                if let Some(_component_to_edit) = &state.edit {
                    let delta = y / 100.0;
                    state.captured_delta += delta;
                    (
                        Status::Captured,
                        Some(CameraPoseMessage::EditEndpoint {
                            cursor: scale_cursor
                                + (state.captured.unwrap()
                                    - Vector::new(scale_cursor.x, scale_cursor.y))
                                    * state.captured_delta,
                        }),
                    )
                } else {
                    (Status::Ignored, None)
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                match &state.edit_state {
                    Edit::ControlPoint => {
                        state.edit_state = Edit::None;
                        self.cache.clear();
                        (Status::Ignored, None)
                    }
                    Edit::None => {
                        if state.edit.is_some() {
                            state.captured = None;
                            (
                                Status::Ignored,
                                Some(CameraPoseMessage::Editline { component: None }),
                            )
                        } else if let Some(line_index) = state.highlight {
                            let (p1, p2) = self.axis_data.borrow_mut().axis_lines[line_index];
                            let clicked_position = scale_cursor;
                            if should_edit_point(clicked_position, p1) {
                                state.captured =
                                    Some(Vector::new(clicked_position.x, clicked_position.y));
                                state.captured_delta = 0.0;
                                (
                                    Status::Ignored,
                                    Some(CameraPoseMessage::Editline {
                                        component: Some(Component::A),
                                    }),
                                )
                            } else if should_edit_point(clicked_position, p2) {
                                state.captured =
                                    Some(Vector::new(clicked_position.x, clicked_position.y));
                                state.captured_delta = 0.0;
                                (
                                    Status::Ignored,
                                    Some(CameraPoseMessage::Editline {
                                        component: Some(Component::B),
                                    }),
                                )
                            } else {
                                state.captured = None;
                                state.captured_delta = 0.0;
                                (
                                    Status::Captured,
                                    Some(CameraPoseMessage::HighlightLine { highlight: None }),
                                )
                            }
                        } else {
                            (Status::Ignored, None)
                        }
                    }

                    _ => (Status::Ignored, None),
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)) => {
                if check_if_control_point_is_clicked(
                    self.axis_data.borrow().control_point,
                    scale_cursor,
                ) {
                    state.edit_state = Edit::ControlPoint;
                    self.cache.clear();
                    return (Status::Captured, None);
                } else {
                    for (index, (p1, p2)) in self.axis_data.borrow().axis_lines.iter().enumerate() {
                        if check_if_point_is_from_line(p1, p2, scale_cursor) {
                            return (
                                Status::Captured,
                                Some(CameraPoseMessage::HighlightLine {
                                    highlight: Some(index),
                                }),
                            );
                        };
                    }
                }

                let is_captured = if state.highlight.is_some() {
                    Status::Captured
                } else {
                    Status::Ignored
                };
                state.edit_state = Edit::None;
                (
                    is_captured,
                    Some(CameraPoseMessage::HighlightLine { highlight: None }),
                )
            }
            Event::Mouse(mouse::Event::CursorMoved { position: _ }) => {
                state.captured_delta = 0.0;
                match &state.edit_state {
                    Edit::ControlPoint => (
                        Status::Captured,
                        Some(CameraPoseMessage::MoveControlPoint {
                            cursor: scale_cursor,
                        }),
                    ),
                    Edit::None => (
                        Status::Captured,
                        Some(CameraPoseMessage::EditEndpoint {
                            cursor: scale_cursor,
                        }),
                    ),
                    _ => (Status::Ignored, None),
                }
            }
            _ => (Status::Ignored, None),
        }
    }

    fn draw_inner(
        &self,
        state: &State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Vec<Renderer::Geometry> {
        let draw_cache = self.draw_cache.draw(renderer, bounds.size(), |frame| {
            *state.points.borrow_mut() = self
                .draw_lines
                .borrow()
                .iter()
                .flat_map(|item| calculate_location_position_to_2d(&state.compute_solution, item))
                .map(|item| to_canvas(bounds.size(), &item))
                .map(|item| Point::new(item.x, item.y))
                .collect();

            let mut builder = canvas::path::Builder::new();
            state
                .points
                .borrow()
                .windows(2)
                .enumerate()
                .for_each(|(index, items)| {
                    let start = items[0];
                    let end = items[1];
                    builder.move_to(start);
                    builder.line_to(end);
                    let location3d_a = *self.draw_lines.borrow().get(index).unwrap();
                    let location3d_b = *self.draw_lines.borrow().get(index + 1).unwrap();
                    let distance = (location3d_b - location3d_a).norm();

                    frame.fill_rectangle(
                        Point::new(end.x + 2.0, end.y + 2.0),
                        Size::new(150.0, 15.0),
                        Fill {
                            style: canvas::Style::Solid(Color::from_rgba(0.3, 0.3, 0.3, 0.8)),
                            ..Fill::default()
                        },
                    );
                    frame.fill_text(Text {
                        content: format!(
                            "{:>7.3},{:>7.3},{:>7.3} ({:.3})",
                            location3d_b.x, location3d_b.y, location3d_b.z, distance
                        ),
                        position: Point::new(end.x + 4.0, end.y + 4.0),
                        color: Color::from_rgba(0.8, 0.8, 0.8, 0.8),
                        size: Pixels(10.0),
                        ..Default::default()
                    });
                });

            let path = builder.build();
            frame.stroke(
                &path,
                Stroke {
                    style: canvas::Style::Solid(Color::from_rgba(0.8, 0.8, 0.8, 0.8)),
                    width: 2.0,
                    ..Stroke::default()
                },
            );
        });

        let color_red = Color::from_rgba(0.8, 0.2, 0.2, 0.8);
        let color_green = Color::from_rgba(0.2, 0.8, 0.2, 0.8);
        let color_blue = Color::from_rgba(0.2, 0.2, 0.8, 0.8);
        let draw = self.cache.draw(renderer, bounds.size(), |frame| {
            if let Some(highlight) = state.highlight {
                let mut builder = canvas::path::Builder::new();

                let (p1, p2) = self.axis_data.borrow().axis_lines[highlight];
                let p1 = scale_point_to_canvas(&Point::new(p1.x, p1.y), bounds.size());
                let p2 = scale_point_to_canvas(&Point::new(p2.x, p2.y), bounds.size());
                builder.move_to(p1);
                builder.line_to(p2);

                builder.circle(p1, 5f32);
                builder.circle(p2, 5f32);

                let path = builder.build();
                frame.stroke(
                    &path,
                    Stroke {
                        style: canvas::Style::Solid(Color::from_rgba(1.0, 0.0, 0.0, 0.8)),
                        width: 2.0,
                        ..Stroke::default()
                    },
                );

                if let Some(points) = get_extension_for_line_within_bounds(&(p1, p2), bounds.size())
                {
                    let mut builder = canvas::path::Builder::new();
                    for (index, point) in points.into_iter().enumerate() {
                        match index {
                            0 => builder.move_to(point),
                            _ => builder.line_to(point),
                        }
                    }
                    let path = builder.build();
                    frame.stroke(
                        &path,
                        Stroke {
                            style: canvas::Style::Solid(Color::from_rgba(0.0, 0.0, 0.9, 0.7)),
                            width: 2.0,
                            ..Stroke::default()
                        },
                    );
                };
                // get new points for the line
            }

            let mut builder = canvas::path::Builder::new();
            let axis_lines = &self.axis_data.borrow().axis_lines;
            if state.highlight.is_none() {
                let (p1, p2) = axis_lines[0];
                let p1 = scale_point_to_canvas(&Point::new(p1.x, p1.y), bounds.size());
                let p2 = scale_point_to_canvas(&Point::new(p2.x, p2.y), bounds.size());
                builder.move_to(p1);
                builder.line_to(p2);
                let path = builder.build();
                frame.stroke(
                    &path,
                    Stroke {
                        style: canvas::Style::Solid(color_red),
                        width: 2.0,
                        line_dash: LineDash {
                            segments: &[8.0, 6.0],
                            offset: 0,
                        },
                        ..Stroke::default()
                    },
                );

                builder = canvas::path::Builder::new();
                let (p1, p2) = axis_lines[1];
                let p1 = scale_point_to_canvas(&Point::new(p1.x, p1.y), bounds.size());
                let p2 = scale_point_to_canvas(&Point::new(p2.x, p2.y), bounds.size());
                builder.move_to(p1);
                builder.line_to(p2);

                let path = builder.build();
                frame.stroke(
                    &path,
                    Stroke {
                        style: canvas::Style::Solid(color_red),
                        width: 2.0,
                        ..Stroke::default()
                    },
                );

                builder = canvas::path::Builder::new();
                let (p1, p2) = axis_lines[2];
                let p1 = scale_point_to_canvas(&Point::new(p1.x, p1.y), bounds.size());
                let p2 = scale_point_to_canvas(&Point::new(p2.x, p2.y), bounds.size());
                builder.move_to(p1);
                builder.line_to(p2);
                let (p1, p2) = axis_lines[3];
                let p1 = scale_point_to_canvas(&Point::new(p1.x, p1.y), bounds.size());
                let p2 = scale_point_to_canvas(&Point::new(p2.x, p2.y), bounds.size());
                builder.move_to(p1);
                builder.line_to(p2);

                let path = builder.build();
                frame.stroke(
                    &path,
                    Stroke {
                        style: canvas::Style::Solid(color_green),
                        width: 2.0,
                        ..Stroke::default()
                    },
                );
                builder = canvas::path::Builder::new();
                let (p1, p2) = axis_lines[4];
                let p1 = scale_point_to_canvas(&Point::new(p1.x, p1.y), bounds.size());
                let p2 = scale_point_to_canvas(&Point::new(p2.x, p2.y), bounds.size());
                builder.move_to(p1);
                builder.line_to(p2);
                let (p1, p2) = axis_lines[5];
                let p1 = scale_point_to_canvas(&Point::new(p1.x, p1.y), bounds.size());
                let p2 = scale_point_to_canvas(&Point::new(p2.x, p2.y), bounds.size());
                builder.move_to(p1);
                builder.line_to(p2);

                let path = builder.build();
                frame.stroke(
                    &path,
                    Stroke {
                        style: canvas::Style::Solid(color_blue),
                        width: 2.0,
                        ..Stroke::default()
                    },
                );
                builder = canvas::path::Builder::new();
            } else {
                for (index, (p1, p2)) in axis_lines.iter().enumerate() {
                    if state.highlight.is_none() || index != state.highlight.unwrap() {
                        let p1 = scale_point_to_canvas(&Point::new(p1.x, p1.y), bounds.size());
                        let p2 = scale_point_to_canvas(&Point::new(p2.x, p2.y), bounds.size());
                        builder.move_to(p1);
                        builder.line_to(p2);
                    }
                }
            }

            let path = builder.build();
            frame.stroke(
                &path,
                Stroke {
                    style: canvas::Style::Solid(Color::BLACK),
                    width: 1.0,
                    ..Stroke::default()
                },
            );
            draw_vanishing_points(
                &self.axis_data.borrow().control_point,
                &self.axis_data.borrow().axis_lines,
                &state.edit_state,
                bounds,
                frame,
            );

            if let Some(point) = state.captured {
                builder = canvas::path::Builder::new();
                builder.circle(
                    scale_point_to_canvas(&Point::new(point.x, point.y), bounds.size()),
                    5.0,
                );

                let path = builder.build();
                frame.stroke(
                    &path,
                    Stroke {
                        style: canvas::Style::Solid(Color::BLACK),
                        width: 1.0,
                        ..Stroke::default()
                    },
                );
                builder = canvas::path::Builder::new();
                builder.move_to(scale_point_to_canvas(
                    &Point::new(point.x, point.y),
                    bounds.size(),
                ));

                let current_cursor = cursor.position().unwrap() - bounds.position();
                builder.line_to(Point::new(current_cursor.x, current_cursor.y));
                let path = builder.build();
                frame.stroke(
                    &path,
                    Stroke {
                        style: canvas::Style::Solid(Color::from_rgba(0.2, 0.2, 0.2, 0.2)),
                        width: 1.0,
                        line_dash: LineDash {
                            segments: &[15.0, 7.0],
                            ..LineDash::default()
                        },
                        ..Stroke::default()
                    },
                );
            }
        });

        let axis_cache = self.axis_cache.draw(renderer, bounds.size(), |frame| {
            if let Some(compute_solution) = &self.compute_solution {
                let dc_to_image = Matrix3::new_nonuniform_scaling(&Vector2::new(
                    bounds.width / 2.0,
                    bounds.width / -2.0,
                ))
                .append_translation(&Vector2::new(bounds.width / 2.0, bounds.height / 2.0));

                let perspective =
                    Perspective3::new(1.0, compute_solution.field_of_view, 0.01, 10.0);

                let mut matrix = perspective.into_inner();
                *matrix.index_mut((0, 2)) = -compute_solution.ortho_center.x;
                *matrix.index_mut((1, 2)) = -compute_solution.ortho_center.y;

                let transform = matrix * compute_solution.view_transform;
                draw_grid_for_origin(frame, color_red, transform, dc_to_image);
                draw_origin_with_axis(
                    frame,
                    color_red,
                    color_green,
                    color_blue,
                    transform,
                    dc_to_image,
                );
                let yellow = Color::from_rgba(0.8, 0.8, 0.2, 0.8);
                let ortho_center =
                    dc_to_image * Point2::from(compute_solution.ortho_center.xy()).to_homogeneous();

                let mut builder = canvas::path::Builder::new();
                let point = Point::new(ortho_center.x, ortho_center.y);
                builder.circle(point, 5.0);
                builder.move_to(point);

                let orthor_center = dc_to_image * Point2::origin().to_homogeneous();
                let point = Point::new(orthor_center.x, orthor_center.y);
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
            }
        });

        vec![draw, axis_cache, draw_cache]
    }

    fn compute_pose(&self, state: &mut State) {
        self.draw_cache.clear();
        let lines_x = [
            self.axis_data.borrow().axis_lines[0],
            self.axis_data.borrow().axis_lines[1],
        ];
        let lines_y = [
            self.axis_data.borrow().axis_lines[2],
            self.axis_data.borrow().axis_lines[3],
        ];
        let lines_z = [
            self.axis_data.borrow().axis_lines[4],
            self.axis_data.borrow().axis_lines[5],
        ];
        let control_point = &self.axis_data.borrow().control_point;
        state.compute_solution = Some(
            compute_ui_adapter(
                lines_x,
                lines_y,
                lines_z,
                self.image_size,
                control_point,
                self.axis_data.borrow().flip,
                &self.axis_data.borrow().custom_origin_translation,
                &self.axis_data.borrow().custom_scale,
            )
            .unwrap(),
        );
    }
}

impl<'a, Message, Theme, Renderer> Widget<Message, Theme, Renderer>
    for ComputeCameraPose<'a, Message, Theme, Renderer>
where
    Renderer: geometry::Renderer,
{
    fn tag(&self) -> tree::Tag {
        struct Tag<T>(T);
        tree::Tag::of::<Tag<State>>()
    }

    fn state(&self) -> tree::State {
        tree::State::new(State::default())
    }

    fn size(&self) -> Size<Length> {
        Size {
            width: self.width,
            height: self.height,
        }
    }

    fn layout(
        &self,
        _tree: &mut Tree,
        _renderer: &Renderer,
        limits: &layout::Limits,
    ) -> layout::Node {
        layout::atomic(limits, self.width, self.height)
    }

    fn update(
        &mut self,
        tree: &mut Tree,
        event: &Event,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        _renderer: &Renderer,
        _clipboard: &mut dyn Clipboard,
        shell: &mut Shell<'_, Message>,
        _viewport: &Rectangle,
    ) {
        let bounds = layout.bounds();

        let state = tree.state.downcast_mut::<State>();

        let (event_status, message) = self.update_inner(state, event, bounds, cursor);
        if let Some(message) = message {
            self.handle_internal_event(state, message);
        }

        if let Status::Captured = event_status {
            shell.capture_event();
            shell.request_redraw();
        }
    }

    fn mouse_interaction(
        &self,
        _tree: &Tree,
        _layout: Layout<'_>,
        _cursor: mouse::Cursor,
        _viewport: &Rectangle,
        _renderer: &Renderer,
    ) -> mouse::Interaction {
        //let bounds = layout.bounds();
        //let state = tree.state.downcast_ref::<State>();
        //self.program.mouse_interaction(state, bounds, cursor)
        mouse::Interaction::default()
    }

    fn draw(
        &self,
        tree: &Tree,
        renderer: &mut Renderer,
        theme: &Theme,
        _style: &Style,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        _viewport: &Rectangle,
    ) {
        let bounds = layout.bounds();

        if bounds.width < 1.0 || bounds.height < 1.0 {
            return;
        }
        let state = tree.state.downcast_ref::<State>();

        renderer.with_translation(Vector::new(bounds.x, bounds.y), |renderer| {
            let layers = self.draw_inner(state, renderer, theme, bounds, cursor);

            for layer in layers {
                renderer.draw_geometry(layer);
            }
        });
    }
}

#[derive(Default, Clone)]
pub struct State {
    pub highlight: Option<usize>,
    pub edit: Option<Component>,
    pub image_path: String,
    pub edit_state: Edit,
    pub points: RefCell<Vec<Point>>,
    pub compute_solution: Option<ComputeSolution<f32>>,
    pub captured: Option<Vector>,
    pub captured_delta: f32,
}

impl<'a, Message, Theme, Renderer> From<ComputeCameraPose<'a, Message, Theme, Renderer>>
    for Element<'a, Message, Theme, Renderer>
where
    Message: 'a,
    Theme: 'a,
    Renderer: 'a + geometry::Renderer,
{
    fn from(
        axis_decoration: ComputeCameraPose<'a, Message, Theme, Renderer>,
    ) -> Element<'a, Message, Theme, Renderer> {
        Element::new(axis_decoration)
    }
}
