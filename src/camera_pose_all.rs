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
    keyboard::{self, Key, key::Named},
    mouse::ScrollDelta,
    widget::canvas::{self, Event, Fill, LineDash, Stroke, Text},
};
use nalgebra::{Matrix3, Perspective3, Point2, Point3, Vector2, Vector3};

use crate::{
    AxisData, Component, Edit, EditAxis, PointInformation,
    compute::{compute_ui_adapter, data::ComputeSolution},
    draw_decoration::{draw_origin_with_axis, draw_vanishing_points},
    utils::{
        calculate_cursor_position_to_3d, check_if_control_point_is_clicked,
        check_if_point_is_from_line, check_if_point_is_from_line_new,
        get_extension_for_line_within_bounds, scale_point, scale_point_to_canvas,
        should_edit_point, to_canvas,
    },
};

#[derive(Debug, Clone)]
enum CameraPoseMessage {
    EditEndpoint { cursor: Point },
    HighlightAxisLine { highlight: Option<usize> },
    Editline { component: Option<Component> },
    MoveControlPoint { cursor: Point },
}
pub struct ComputeCameraPose<Message, Theme = iced::Theme, Renderer = iced::Renderer>
where
    Renderer: geometry::Renderer,
{
    width: Length,
    height: Length,
    message_: PhantomData<Message>,
    cache: geometry::Cache<Renderer>,
    axis_cache: geometry::Cache<Renderer>,
    draw_lines_cache: geometry::Cache<Renderer>,
    vanishing_lines_cache: geometry::Cache<Renderer>,

    compute_solution: RefCell<Option<ComputeSolution<f32>>>,
    renderer_: PhantomData<Renderer>,
    theme_: PhantomData<Theme>,
    axis_data: Rc<RefCell<AxisData>>,
    image_size: Size<f32>,
    draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
    reference_cub: Rc<RefCell<Vec<Point3<f32>>>>,
    vanishing_points: Rc<RefCell<Vec<(EditAxis, Point)>>>,
    custom_origin_translation: Rc<RefCell<Option<Vector3<f32>>>>,
    custom_scale_segment: Rc<RefCell<Option<usize>>>,
    custom_scale: Rc<RefCell<Option<PointInformation<f32>>>>,
}
impl<'a, M, Theme, Renderer> ComputeCameraPose<M, Theme, Renderer>
where
    Renderer: geometry::Renderer,
{
    const DEFAULT_SIZE: f32 = 100.0;
    pub fn new(
        axis_data: Rc<RefCell<AxisData>>,
        draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
        reference_cub: Rc<RefCell<Vec<Point3<f32>>>>,
        compute_solution: &'a Option<ComputeSolution<f32>>,

        custom_origin_translation: Rc<RefCell<Option<Vector3<f32>>>>,
        custom_scale_segment: Rc<RefCell<Option<usize>>>,
        custom_scale: Rc<RefCell<Option<PointInformation<f32>>>>,
    ) -> Self {
        ComputeCameraPose {
            width: Length::Fixed(Self::DEFAULT_SIZE),
            height: Length::Fixed(Self::DEFAULT_SIZE),
            compute_solution: RefCell::new(compute_solution.clone()),
            axis_data,
            message_: PhantomData,
            renderer_: PhantomData,
            theme_: PhantomData,
            cache: geometry::Cache::default(),
            axis_cache: geometry::Cache::default(),
            draw_lines_cache: geometry::Cache::new(),
            vanishing_lines_cache: geometry::Cache::default(),
            draw_lines,
            reference_cub,
            image_size: Size::default(),
            vanishing_points: Rc::new(RefCell::new(Vec::<(EditAxis, Point)>::new())),
            custom_origin_translation,
            custom_scale_segment,
            custom_scale,
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
            CameraPoseMessage::HighlightAxisLine { highlight } => {
                state.highlight_axis_line = highlight;
                self.cache.clear();
                self.axis_cache.clear();
            }
            CameraPoseMessage::Editline { component } => {
                if component.is_none() {
                    self.cache.clear();
                    self.axis_cache.clear();
                }
                state.edit = component;
            }
            CameraPoseMessage::EditEndpoint { cursor } => {
                if let Some(component_to_edit) = &state.edit {
                    match component_to_edit {
                        Component::A => {
                            self.axis_data.borrow_mut().axis_lines
                                [state.highlight_axis_line.unwrap()]
                            .0 = cursor;
                        }
                        Component::B => {
                            self.axis_data.borrow_mut().axis_lines
                                [state.highlight_axis_line.unwrap()]
                            .1 = cursor;
                        }
                    };
                    self.cache.clear();
                    self.axis_cache.clear();
                    self.vanishing_lines_cache.clear();
                    self.draw_lines_cache.clear();
                    self.compute_pose();
                }
            }
            CameraPoseMessage::MoveControlPoint { cursor } => {
                self.axis_data.borrow_mut().control_point = cursor;
                self.cache.clear();
                self.draw_lines_cache.clear();
                self.compute_pose();
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
        let adjusted_cursor = cursor - bounds.position();
        let scale_cursor = scale_point(adjusted_cursor, bounds.size());
        match event {
            Event::Keyboard(keyboard::Event::KeyPressed {
                key: Key::Character(c),
                ..
            }) => {
                let c = c.as_str();
                match c {
                    "w" => {
                        if let Edit::VanishingLines(_) = state.edit_state {
                            state.edit_state = Edit::None;
                            self.vanishing_lines_cache.clear();
                        } else {
                            state.edit_state = Edit::VanishingLines(EditAxis::None);
                            self.vanishing_lines_cache.clear();
                        }
                        (Status::Captured, None)
                    }
                    "f" => {
                        self.vanishing_lines_cache.clear();
                        if let Edit::Draw = state.edit_state {
                            state.edit_state = Edit::None;
                        } else {
                            state.edit_state = Edit::Draw;
                        }
                        (Status::Captured, None)
                    }
                    "r" => match state.edit_state {
                        Edit::ControlPoint(_) => {
                            state.captured_delta = 0.0;
                            state.edit_state = Edit::ControlPoint(EditAxis::EditX);
                            (Status::Captured, None)
                        }
                        Edit::VanishingPoint(_) => {
                            state.captured_delta = 0.0;
                            state.edit_state = Edit::VanishingPoint(EditAxis::EditX);
                            (Status::Captured, None)
                        }
                        Edit::VanishingLines(_) => {
                            state.captured_delta = 0.0;
                            self.vanishing_lines_cache.clear();
                            state.edit_state = Edit::VanishingLines(EditAxis::EditX);
                            (Status::Captured, None)
                        }
                        Edit::Extrude(_) => {
                            state.edit_state = Edit::Extrude(EditAxis::EditX);
                            (Status::Captured, None)
                        }
                        Edit::Scale(_) => {
                            state.edit_state = Edit::Scale(EditAxis::EditX);
                            (Status::Captured, None)
                        }
                        _ => (Status::Captured, None),
                    },
                    "s" => match state.edit_state {
                        Edit::ControlPoint(_) => {
                            state.captured_delta = 0.0;
                            state.edit_state = Edit::ControlPoint(EditAxis::EditY);
                            (Status::Captured, None)
                        }
                        Edit::VanishingPoint(_) => {
                            state.captured_delta = 0.0;
                            state.edit_state = Edit::VanishingPoint(EditAxis::EditY);
                            (Status::Captured, None)
                        }
                        Edit::VanishingLines(_) => {
                            state.captured_delta = 0.0;
                            self.vanishing_lines_cache.clear();
                            state.edit_state = Edit::VanishingLines(EditAxis::EditY);
                            (Status::Captured, None)
                        }
                        Edit::Extrude(_) => {
                            state.edit_state = Edit::Extrude(EditAxis::EditY);
                            (Status::Captured, None)
                        }
                        Edit::Scale(_) => {
                            state.edit_state = Edit::Scale(EditAxis::EditY);
                            (Status::Captured, None)
                        }
                        _ => (Status::Captured, None),
                    },
                    "t" => match state.edit_state {
                        Edit::VanishingLines(_) => {
                            state.captured_delta = 0.0;
                            self.cache.clear();
                            self.vanishing_lines_cache.clear();
                            state.edit_state = Edit::VanishingLines(EditAxis::EditZ);
                            (Status::Captured, None)
                        }
                        Edit::Extrude(_) => {
                            state.edit_state = Edit::Extrude(EditAxis::EditZ);
                            (Status::Captured, None)
                        }
                        Edit::Scale(_) => {
                            state.edit_state = Edit::Scale(EditAxis::EditZ);
                            (Status::Captured, None)
                        }
                        _ => (Status::Captured, None),
                    },
                    "x" => {
                        state.edit_state = Edit::Scale(EditAxis::None);
                        (Status::Captured, None)
                    }
                    "c" => {
                        state.edit_state = Edit::Extrude(EditAxis::None);
                        (Status::Captured, None)
                    }
                    "d" => {
                        if self.draw_lines.borrow().len() > 1 {
                            self.draw_lines.borrow_mut().pop();
                            self.draw_lines_cache.clear();
                        }
                        state.edit_state = Edit::Draw;
                        (Status::Captured, None)
                    }
                    _ => (Status::Ignored, None),
                }
            }
            Event::Keyboard(keyboard::Event::KeyPressed {
                key: Key::Named(Named::Escape),
                ..
            }) => match &state.edit_state {
                Edit::ControlPoint(_) => (
                    Status::Captured,
                    Some(CameraPoseMessage::MoveControlPoint {
                        cursor: Point::new(state.captured.unwrap().x, state.captured.unwrap().y),
                    }),
                ),
                Edit::VanishingPoint(_) => (
                    Status::Captured,
                    Some(CameraPoseMessage::EditEndpoint {
                        cursor: Point::new(state.captured.unwrap().x, state.captured.unwrap().y),
                    }),
                ),
                _ => (Status::Ignored, None),
            },
            Event::Mouse(mouse::Event::WheelScrolled {
                delta: ScrollDelta::Lines { x: _x, y },
            }) => {
                let delta = y / 1000.0;
                state.captured_delta += delta;
                let vector_for_delta = match &state.edit_state {
                    Edit::ControlPoint(EditAxis::EditX) | Edit::VanishingPoint(EditAxis::EditX) => {
                        state.captured.unwrap()
                            - Vector::new(scale_cursor.x, state.captured.unwrap().y)
                    }
                    Edit::ControlPoint(EditAxis::EditY) | Edit::VanishingPoint(EditAxis::EditY) => {
                        state.captured.unwrap()
                            - Vector::new(state.captured.unwrap().x, scale_cursor.y)
                    }
                    _ => Vector::new(scale_cursor.x, scale_cursor.y),
                };
                match &state.edit_state {
                    Edit::ControlPoint(_) => (
                        Status::Captured,
                        Some(CameraPoseMessage::MoveControlPoint {
                            cursor: scale_cursor + vector_for_delta * state.captured_delta,
                        }),
                    ),
                    Edit::VanishingPoint(_) => (
                        Status::Captured,
                        Some(CameraPoseMessage::EditEndpoint {
                            cursor: scale_cursor + vector_for_delta * state.captured_delta,
                        }),
                    ),
                    Edit::Extrude(_) | Edit::Scale(_) => (Status::Captured, None),
                    _ => (Status::Ignored, None),
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let clicked_position = scale_cursor;
                state.captured = Some(Vector::new(clicked_position.x, clicked_position.y));
                match &state.edit_state {
                    Edit::VanishingLines(axis) => {
                        self.vanishing_points.borrow_mut().push((
                            axis.clone(),
                            Point::new(adjusted_cursor.x, adjusted_cursor.y),
                        ));
                        self.vanishing_lines_cache.clear();
                        (Status::Captured, None)
                    }
                    Edit::ControlPoint(_) => {
                        state.edit_state = Edit::None;
                        self.cache.clear();
                        self.axis_cache.clear();
                        (Status::Ignored, None)
                    }
                    Edit::VanishingPoint(_) => {
                        state.edit_state = Edit::None;
                        state.edit = None;
                        state.captured = None;
                        state.highlight_axis_line = None;
                        self.cache.clear();
                        self.axis_cache.clear();
                        (Status::Ignored, None)
                    }
                    Edit::Draw => {
                        let cursor = Point::new(adjusted_cursor.x, adjusted_cursor.y);
                        for (index, point) in state.points.borrow().iter().enumerate() {
                            if cursor.distance(*point) < 10.0 {
                                state.selected = index;
                                self.custom_origin_translation
                                    .replace(self.draw_lines.borrow().get(index).copied());
                                return (Status::Captured, None);
                            };
                        }
                        if state
                            .points
                            .borrow()
                            .windows(2)
                            .find(|items| {
                                let start = items[0];
                                let end = items[1];
                                check_if_point_is_from_line_new(&start, &end, cursor)
                            })
                            .iter()
                            .enumerate()
                            .map(|(index, _items)| {
                                self.custom_scale_segment.borrow_mut().replace(index);
                            })
                            .count()
                            > 0
                        {
                            return (Status::Captured, None);
                        }
                        (Status::Ignored, None)
                    }
                    Edit::None => {
                        if state.edit.is_some() {
                            state.captured = None;
                            (
                                Status::Ignored,
                                Some(CameraPoseMessage::Editline { component: None }),
                            )
                        } else if let Some(line_index) = state.highlight_axis_line {
                            let (p1, p2) = self.axis_data.borrow_mut().axis_lines[line_index];
                            if should_edit_point(clicked_position, p1) {
                                state.captured = Some(Vector::new(p1.x, p1.y));

                                state.edit_state = Edit::VanishingPoint(EditAxis::None);
                                state.captured_delta = 0.0;
                                (
                                    Status::Ignored,
                                    Some(CameraPoseMessage::Editline {
                                        component: Some(Component::A),
                                    }),
                                )
                            } else if should_edit_point(clicked_position, p2) {
                                state.captured = Some(Vector::new(p2.x, p2.y));
                                state.edit_state = Edit::VanishingPoint(EditAxis::None);
                                state.captured_delta = 0.0;
                                (
                                    Status::Ignored,
                                    Some(CameraPoseMessage::Editline {
                                        component: Some(Component::B),
                                    }),
                                )
                            } else {
                                state.captured = None;
                                state.edit_state = Edit::None;
                                state.captured_delta = 0.0;
                                (
                                    Status::Captured,
                                    Some(CameraPoseMessage::HighlightAxisLine { highlight: None }),
                                )
                            }
                        } else {
                            (Status::Ignored, None)
                        }
                    }
                    _ => (Status::Ignored, None),
                }
            }

            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                let Some((new_point_3d, last_point_3d, _color)) =
                    self.extract_last_point_details_for_mode(state, bounds, &adjusted_cursor)
                else {
                    return (Status::Ignored, None);
                };

                let new_point_3d =
                    new_point_3d + (last_point_3d - new_point_3d) * state.captured_delta;

                match &state.edit_state {
                    Edit::Extrude(_axis) => {
                        self.draw_lines.borrow_mut().push(new_point_3d);
                        self.draw_lines_cache.clear();
                        state.edit_state = Edit::Draw;
                    }
                    Edit::Scale(axis) => {
                        self.custom_scale.borrow_mut().replace(PointInformation {
                            vector: new_point_3d,
                            source_vector: *self.draw_lines.borrow().get(state.selected).unwrap(),
                            point: Vector2::new(
                                adjusted_cursor.x / bounds.width,
                                adjusted_cursor.y / bounds.height,
                            ),
                            axis: axis.clone(),
                        });
                        self.draw_lines_cache.clear();
                        state.edit_state = Edit::Draw;
                    }

                    _ => (),
                }
                (Status::Captured, None)
            }

            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)) => {
                match state.edit_state {
                    Edit::None | Edit::VanishingPoint(_) => {
                        if check_if_control_point_is_clicked(
                            self.axis_data.borrow().control_point,
                            scale_cursor,
                        ) {
                            state.captured = Some(Vector::new(
                                self.axis_data.borrow().control_point.x,
                                self.axis_data.borrow().control_point.y,
                            ));
                            state.edit_state = Edit::ControlPoint(EditAxis::None);
                            self.cache.clear();
                            return (Status::Captured, None);
                        } else {
                            for (index, (p1, p2)) in
                                self.axis_data.borrow().axis_lines.iter().enumerate()
                            {
                                if check_if_point_is_from_line(p1, p2, scale_cursor) {
                                    return (
                                        Status::Captured,
                                        Some(CameraPoseMessage::HighlightAxisLine {
                                            highlight: Some(index),
                                        }),
                                    );
                                };
                            }
                        }
                        let is_captured = if state.highlight_axis_line.is_some() {
                            Status::Captured
                        } else {
                            Status::Ignored
                        };
                        (
                            is_captured,
                            Some(CameraPoseMessage::HighlightAxisLine { highlight: None }),
                        )
                    }
                    _ => (Status::Ignored, None),
                }
            }
            Event::Mouse(mouse::Event::CursorMoved { position: _ }) => {
                state.captured_delta = 0.0;
                match &state.edit_state {
                    Edit::ControlPoint(_) => {
                        self.axis_cache.clear();
                        (
                            Status::Captured,
                            Some(CameraPoseMessage::MoveControlPoint {
                                cursor: scale_cursor,
                            }),
                        )
                    }
                    Edit::VanishingPoint(_) => {
                        self.axis_cache.clear();
                        (
                            Status::Captured,
                            Some(CameraPoseMessage::EditEndpoint {
                                cursor: scale_cursor,
                            }),
                        )
                    }
                    Edit::VanishingLines(_) => {
                        self.vanishing_lines_cache.clear();
                        (Status::Captured, None)
                    }
                    Edit::Extrude(_) | Edit::Scale(_) => (Status::Captured, None),
                    Edit::None => (
                        // Status::Ignored, //TODO: check to avoid requesting redraw
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
        let color_red = Color::from_rgba(0.8, 0.2, 0.2, 0.8);
        let color_green = Color::from_rgba(0.2, 0.8, 0.2, 0.8);
        let color_blue = Color::from_rgba(0.2, 0.2, 0.8, 0.8);
        let draw = self.cache.draw(renderer, bounds.size(), |frame| {
            if self.compute_solution.borrow().as_ref().is_none() {
                return;
            }
            *state.points.borrow_mut() = self
                .draw_lines
                .borrow()
                .iter()
                .flat_map(|item| {
                    self.compute_solution
                        .borrow()
                        .as_ref()
                        .unwrap()
                        .calculate_location_position_to_2d(item)
                })
                .map(|item| to_canvas(bounds.size(), &item))
                .map(|item| Point::new(item.x, item.y))
                .collect();

            *state.reference_cub_2d.borrow_mut() = self
                .compute_solution
                .borrow()
                .as_ref()
                .unwrap()
                .calculate_location_position_to_2d_frustum(&self.reference_cub.as_ref().borrow())
                .iter()
                .map(|&(start, end)| {
                    let start = to_canvas(bounds.size(), &start.coords.xy());
                    let end = to_canvas(bounds.size(), &end.coords.xy());
                    (Point::new(start.x, start.y), Point::new(end.x, end.y))
                })
                .collect();

            if let Edit::Draw = state.edit_state {
                let selected_color = match &state.edit_state {
                    Edit::ControlPoint(_) => Color::from_rgba(0.8, 0.8, 0.2, 0.8),
                    Edit::Draw => Color::from_rgba(0.8, 0.8, 0.2, 0.8),
                    Edit::Extrude(_) => Color::from_rgba(0.8, 0.8, 0.8, 0.8),
                    Edit::Scale(_) => Color::from_rgba(0.2, 0.8, 0.2, 0.8),
                    Edit::None => Color::from_rgba(0.8, 0.8, 0.2, 0.8),
                    _ => Color::from_rgba(0.8, 0.8, 0.2, 0.8),
                };
                if let Some(item) = state.points.borrow().get(state.selected) {
                    let mut builder = canvas::path::Builder::new();
                    builder.circle(*item, 5.0);
                    let path = builder.build();
                    frame.stroke(
                        &path,
                        Stroke {
                            style: canvas::Style::Solid(selected_color),
                            width: 2.0,
                            ..Stroke::default()
                        },
                    );
                };

                if let Some(custom_scale_segment) = self.custom_scale_segment.borrow().as_ref() {
                    state
                        .points
                        .borrow()
                        .windows(2)
                        .enumerate()
                        .filter(|(index, _items)| index == custom_scale_segment)
                        .for_each(|(_index, items)| {
                            let mut builder = canvas::path::Builder::new();
                            let start = items[0];
                            let end = items[1];
                            builder.move_to(start);
                            builder.line_to(end);
                            let path = builder.build();
                            frame.stroke(
                                &path,
                                Stroke {
                                    style: canvas::Style::Solid(Color::from_rgba(
                                        0.8, 0.8, 0.2, 0.8,
                                    )),
                                    width: 2.0,
                                    ..Stroke::default()
                                },
                            );
                        });
                }
                if let Some(scale) = self.custom_scale.borrow().as_ref() {
                    let start = to_canvas(
                        bounds.size(),
                        &self
                            .compute_solution
                            .borrow()
                            .as_ref()
                            .unwrap()
                            .calculate_location_position_to_2d(&scale.source_vector)
                            .unwrap(),
                    );
                    let end = to_canvas(
                        bounds.size(),
                        &self
                            .compute_solution
                            .borrow()
                            .as_ref()
                            .unwrap()
                            .calculate_location_position_to_2d(&scale.vector)
                            .unwrap(),
                    );
                    let start = Point::new(start.x, start.y);
                    let end = Point::new(end.x, end.y);
                    let mut builder = canvas::path::Builder::new();
                    builder.move_to(start);
                    builder.line_to(end);
                    let path = builder.build();
                    frame.stroke(
                        &path,
                        Stroke {
                            style: canvas::Style::Solid(Color::from_rgba(0.2, 0.8, 0.2, 0.8)),
                            width: 2.0,
                            ..Stroke::default()
                        },
                    );
                }
            }

            match state.edit_state {
                Edit::ControlPoint(_) | Edit::VanishingPoint(_) => {
                    if let Some(point) = state.captured {
                        let mut builder = canvas::path::Builder::new();
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
                }
                Edit::Extrude(_) | Edit::Scale(_) => {
                    let Some(cursor) = cursor.position() else {
                        return;
                    };
                    let cursor = cursor - bounds.position();

                    let Some((new_point_3d, last_point_3d, color)) =
                        self.extract_last_point_details_for_mode(state, bounds, &cursor)
                    else {
                        return;
                    };

                    let new_point_3d =
                        new_point_3d + (last_point_3d - new_point_3d) * state.captured_delta;

                    let last_point = to_canvas(
                        bounds.size(),
                        &self
                            .compute_solution
                            .borrow()
                            .as_ref()
                            .unwrap()
                            .calculate_location_position_to_2d(&last_point_3d)
                            .unwrap(),
                    );

                    let new_point = to_canvas(
                        bounds.size(),
                        &self
                            .compute_solution
                            .borrow()
                            .as_ref()
                            .unwrap()
                            .calculate_location_position_to_2d(&new_point_3d)
                            .unwrap(),
                    );

                    self.draw_current_location_helpers(bounds, frame, new_point_3d, new_point);

                    let mut builder = canvas::path::Builder::new();
                    builder.move_to(Point::new(last_point.x, last_point.y));
                    builder.line_to(Point::new(new_point.x, new_point.y));
                    let path = builder.build();
                    frame.stroke(
                        &path,
                        Stroke {
                            style: canvas::Style::Solid(color),
                            width: 1.5,
                            ..Stroke::default()
                        },
                    );
                    frame.fill_rectangle(
                        Point::new(new_point.x + 2.0, new_point.y + 2.0),
                        Size::new(150.0, 15.0),
                        Fill {
                            style: canvas::Style::Solid(Color::from_rgba(0.3, 0.3, 0.3, 0.9)),
                            ..Fill::default()
                        },
                    );
                    frame.fill_text(Text {
                        content: format!(
                            "{:>5.3}, {:>5.3}, {:>5.3}",
                            new_point_3d.x, new_point_3d.y, new_point_3d.z
                        ),
                        position: Point::new(new_point.x + 15.0, new_point.y + 4.0),
                        color,
                        size: Pixels(10.0),
                        ..Default::default()
                    });
                }
                _ => (),
            }
        });

        let axis_cache = self.axis_cache.draw(renderer, bounds.size(), |frame| {
            state.vanishing_points.replace(draw_vanishing_points(
                &self.axis_data.borrow().control_point,
                &self.axis_data.borrow().axis_lines,
                &state.edit_state,
                bounds,
                frame,
            ));
            if let Some(highlight) = state.highlight_axis_line {
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
                            style: canvas::Style::Solid(Color::from_rgba(1.0, 1.0, 0.9, 0.7)),
                            width: 1.0,
                            ..Stroke::default()
                        },
                    );
                };
                // get new points for the line
            } else {
                let mut builder = canvas::path::Builder::new();
                let axis_lines = &self.axis_data.borrow().axis_lines;
                if state.highlight_axis_line.is_none() {
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
                            width: 1.0,
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
                            width: 1.0,
                            ..Stroke::default()
                        },
                    );

                    builder = canvas::path::Builder::new();
                    let (p1, p2) = axis_lines[2];
                    let p1 = scale_point_to_canvas(&Point::new(p1.x, p1.y), bounds.size());
                    let p2 = scale_point_to_canvas(&Point::new(p2.x, p2.y), bounds.size());
                    builder.move_to(p1);
                    builder.line_to(p2);
                    let path = builder.build();
                    frame.stroke(
                        &path,
                        Stroke {
                            style: canvas::Style::Solid(color_green),
                            width: 1.0,
                            line_dash: LineDash {
                                segments: &[8.0, 6.0],
                                offset: 0,
                            },
                            ..Stroke::default()
                        },
                    );

                    builder = canvas::path::Builder::new();
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
                            width: 1.0,
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
                            width: 1.0,
                            ..Stroke::default()
                        },
                    );
                    builder = canvas::path::Builder::new();
                } else {
                    for (index, (p1, p2)) in axis_lines.iter().enumerate() {
                        if state.highlight_axis_line.is_none()
                            || index != state.highlight_axis_line.unwrap()
                        {
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
            }

            if let Some(compute_solution) = self.compute_solution.borrow().as_ref() {
                let dc_to_image = Matrix3::new_nonuniform_scaling(&Vector2::new(
                    bounds.width / 2.0,
                    bounds.width / -2.0,
                ))
                .append_translation(&Vector2::new(bounds.width / 2.0, bounds.height / 2.0));

                let perspective =
                    Perspective3::new(1.0, compute_solution.field_of_view(), 0.01, 10.0);

                let mut matrix = perspective.into_inner();
                *matrix.index_mut((0, 2)) = -compute_solution.ortho_center().x;
                *matrix.index_mut((1, 2)) = -compute_solution.ortho_center().y;

                let transform = matrix * compute_solution.view_transform();
                //draw_grid_for_origin(frame, color_red, transform, dc_to_image);
                draw_origin_with_axis(
                    frame,
                    color_red,
                    color_green,
                    color_blue,
                    transform,
                    dc_to_image,
                );
                let yellow = Color::from_rgba(0.8, 0.8, 0.2, 0.8);
                let ortho_center = dc_to_image
                    * Point2::from(compute_solution.ortho_center().xy()).to_homogeneous();

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

        let vanishing_lines_cache =
            self.vanishing_lines_cache
                .draw(renderer, bounds.size(), |frame| {
                    let (vanishing_point_x, vanishing_point_y, vanishing_point_z) =
                        *state.vanishing_points.borrow();
                    if let Some(_position) = cursor.position() {
                        let current_cursor = cursor.position().unwrap() - bounds.position();
                        let mut builder = canvas::path::Builder::new();
                        self.vanishing_points
                            .borrow()
                            .iter()
                            .for_each(|(axis, current_cursor)| match axis {
                                EditAxis::EditX => {
                                    builder.move_to(scale_point_to_canvas(
                                        &Point::new(vanishing_point_x.x, vanishing_point_x.y),
                                        bounds.size(),
                                    ));
                                    builder.line_to(*current_cursor);
                                }
                                EditAxis::EditY => {
                                    builder.move_to(scale_point_to_canvas(
                                        &Point::new(vanishing_point_y.x, vanishing_point_y.y),
                                        bounds.size(),
                                    ));
                                    builder.line_to(*current_cursor);
                                }
                                EditAxis::EditZ => {
                                    builder.move_to(scale_point_to_canvas(
                                        &Point::new(vanishing_point_z.x, vanishing_point_z.y),
                                        bounds.size(),
                                    ));
                                    builder.line_to(*current_cursor);
                                }
                                EditAxis::None => {
                                    builder.move_to(scale_point_to_canvas(
                                        &Point::new(vanishing_point_x.x, vanishing_point_x.y),
                                        bounds.size(),
                                    ));
                                    builder.line_to(*current_cursor);
                                    builder.move_to(scale_point_to_canvas(
                                        &Point::new(vanishing_point_y.x, vanishing_point_y.y),
                                        bounds.size(),
                                    ));
                                    builder.line_to(*current_cursor);
                                    builder.move_to(scale_point_to_canvas(
                                        &Point::new(vanishing_point_z.x, vanishing_point_z.y),
                                        bounds.size(),
                                    ));
                                    builder.line_to(*current_cursor);
                                }
                            });

                        match state.edit_state {
                            Edit::VanishingLines(EditAxis::EditX) => {
                                builder.move_to(scale_point_to_canvas(
                                    &Point::new(vanishing_point_x.x, vanishing_point_x.y),
                                    bounds.size(),
                                ));
                                builder.line_to(Point::new(current_cursor.x, current_cursor.y));
                            }
                            Edit::VanishingLines(EditAxis::EditY) => {
                                builder.move_to(scale_point_to_canvas(
                                    &Point::new(vanishing_point_y.x, vanishing_point_y.y),
                                    bounds.size(),
                                ));
                                builder.line_to(Point::new(current_cursor.x, current_cursor.y));
                            }
                            Edit::VanishingLines(EditAxis::EditZ) => {
                                builder.move_to(scale_point_to_canvas(
                                    &Point::new(vanishing_point_z.x, vanishing_point_z.y),
                                    bounds.size(),
                                ));
                                builder.line_to(Point::new(current_cursor.x, current_cursor.y));
                            }
                            Edit::VanishingLines(EditAxis::None) => {
                                builder.move_to(scale_point_to_canvas(
                                    &Point::new(vanishing_point_x.x, vanishing_point_x.y),
                                    bounds.size(),
                                ));
                                builder.line_to(Point::new(current_cursor.x, current_cursor.y));

                                builder.move_to(scale_point_to_canvas(
                                    &Point::new(vanishing_point_y.x, vanishing_point_y.y),
                                    bounds.size(),
                                ));
                                builder.line_to(Point::new(current_cursor.x, current_cursor.y));

                                builder.move_to(scale_point_to_canvas(
                                    &Point::new(vanishing_point_z.x, vanishing_point_z.y),
                                    bounds.size(),
                                ));
                                builder.line_to(Point::new(current_cursor.x, current_cursor.y));
                            }
                            _ => {}
                        }

                        let path = builder.build();
                        frame.stroke(
                            &path,
                            Stroke {
                                style: canvas::Style::Solid(Color::from_rgba(0.9, 0.9, 0.9, 0.9)),
                                width: 1.0,
                                ..Stroke::default()
                            },
                        );
                    }
                });

        let draw_lines_cache = self
            .draw_lines_cache
            .draw(renderer, bounds.size(), |frame| {
                let mut builder = canvas::path::Builder::new();
                state.points.borrow().windows(2).for_each(|items| {
                    let start = items[0];
                    let end = items[1];
                    builder.move_to(start);
                    builder.line_to(end);
                });
                let path = builder.build();
                frame.stroke(
                    &path,
                    Stroke {
                        style: canvas::Style::Solid(Color::from_rgba(0.8, 0.8, 0.8, 0.8)),
                        width: 1.0,
                        ..Stroke::default()
                    },
                );

                state
                    .points
                    .borrow()
                    .windows(2)
                    .enumerate()
                    .for_each(|(index, items)| {
                        let end = items[1];
                        let location3d_a = *self.draw_lines.borrow().get(index).unwrap();
                        let location3d_b = *self.draw_lines.borrow().get(index + 1).unwrap();
                        let distance = (location3d_b - location3d_a).norm();

                        frame.fill_rectangle(
                            Point::new(end.x + 2.0, end.y + 2.0),
                            Size::new(150.0, 15.0),
                            Fill {
                                style: canvas::Style::Solid(Color::from_rgba(0.3, 0.3, 0.3, 0.9)),
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

                let mut builder = canvas::path::Builder::new();
                state
                    .reference_cub_2d
                    .borrow()
                    .iter()
                    .for_each(|&(start, end)| {
                        builder.move_to(start);
                        builder.line_to(end);
                    });
                let path = builder.build();
                frame.stroke(
                    &path,
                    Stroke {
                        style: canvas::Style::Solid(Color::from_rgba(0.9, 0.7, 0.7, 1.0)),
                        width: 1.0,
                        ..Stroke::default()
                    },
                );
            });

        match state.edit_state {
            Edit::None | Edit::VanishingPoint(_) | Edit::ControlPoint(_) => {
                vec![vanishing_lines_cache, draw_lines_cache, draw, axis_cache]
            }
            _ => vec![vanishing_lines_cache, draw_lines_cache, draw],
        }
    }

    fn compute_pose(&self) {
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
        self.compute_solution.borrow_mut().replace(
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
    fn extract_last_point_details_for_mode<'b>(
        &self,
        state: &'b State,
        bounds: Rectangle,
        cursor: &'b Vector,
    ) -> Option<(Vector3<f32>, Vector3<f32>, Color)> {
        let (axis, last_point_3d, color) = match &state.edit_state {
            Edit::Extrude(axis) => {
                let last_point_3d = *self.draw_lines.borrow().last()?;
                (axis, last_point_3d, Color::from_rgba(0.8, 0.8, 0.8, 0.8))
            }
            Edit::Scale(axis) => {
                let last_point_3d = *self.draw_lines.borrow().get(state.selected)?;
                (axis, last_point_3d, Color::from_rgba(0.2, 0.8, 0.2, 0.8))
            }
            _ => {
                return None;
            }
        };

        let new_point_3d = calculate_cursor_position_to_3d(
            axis,
            self.compute_solution.borrow().as_ref().unwrap(),
            self.image_size.width / self.image_size.height,
            &Vector2::new(cursor.x / bounds.width, cursor.y / bounds.height),
            last_point_3d,
        )?;

        let new_point_3d = match axis {
            EditAxis::EditX => Vector3::new(new_point_3d.x, last_point_3d.y, last_point_3d.z),
            EditAxis::EditY => Vector3::new(last_point_3d.x, new_point_3d.y, last_point_3d.z),
            EditAxis::EditZ => Vector3::new(last_point_3d.x, last_point_3d.y, new_point_3d.z),
            _ => new_point_3d,
        };
        Some((new_point_3d, last_point_3d, color))
    }

    fn draw_current_location_helpers(
        &self,
        bounds: Rectangle,
        frame: &mut geometry::Frame<Renderer>,
        new_point_3d: Vector3<f32>,
        new_point: Vector2<f32>,
    ) {
        let mut builder = canvas::path::Builder::new();

        //x
        let new_point_helper_positive = to_canvas(
            bounds.size(),
            &self
                .compute_solution
                .borrow()
                .as_ref()
                .unwrap()
                .calculate_location_position_to_2d(&(new_point_3d + Vector3::new(3.0, 0.0, 0.0)))
                .unwrap(),
        );

        builder.move_to(Point::new(new_point.x, new_point.y));
        builder.line_to(Point::new(
            new_point_helper_positive.x,
            new_point_helper_positive.y,
        ));
        let new_point_helper_negative = to_canvas(
            bounds.size(),
            &self
                .compute_solution
                .borrow()
                .as_ref()
                .unwrap()
                .calculate_location_position_to_2d(&(new_point_3d + Vector3::new(-3.0, 0.0, 0.0)))
                .unwrap(),
        );
        builder.move_to(Point::new(new_point.x, new_point.y));
        builder.line_to(Point::new(
            new_point_helper_negative.x,
            new_point_helper_negative.y,
        ));
        //y
        let new_point_helper_positive = to_canvas(
            bounds.size(),
            &self
                .compute_solution
                .borrow()
                .as_ref()
                .unwrap()
                .calculate_location_position_to_2d(&(new_point_3d + Vector3::new(0.0, 3.0, 0.0)))
                .unwrap(),
        );

        builder.move_to(Point::new(new_point.x, new_point.y));
        builder.line_to(Point::new(
            new_point_helper_positive.x,
            new_point_helper_positive.y,
        ));
        let new_point_helper_negative = to_canvas(
            bounds.size(),
            &self
                .compute_solution
                .borrow()
                .as_ref()
                .unwrap()
                .calculate_location_position_to_2d(&(new_point_3d + Vector3::new(0.0, -3.0, 0.0)))
                .unwrap(),
        );
        builder.move_to(Point::new(new_point.x, new_point.y));
        builder.line_to(Point::new(
            new_point_helper_negative.x,
            new_point_helper_negative.y,
        ));
        //z
        let new_point_helper_positive = to_canvas(
            bounds.size(),
            &self
                .compute_solution
                .borrow()
                .as_ref()
                .unwrap()
                .calculate_location_position_to_2d(&(new_point_3d + Vector3::new(0.0, 0.0, 3.0)))
                .unwrap(),
        );

        builder.move_to(Point::new(new_point.x, new_point.y));
        builder.line_to(Point::new(
            new_point_helper_positive.x,
            new_point_helper_positive.y,
        ));
        let new_point_helper_negative = to_canvas(
            bounds.size(),
            &self
                .compute_solution
                .borrow()
                .as_ref()
                .unwrap()
                .calculate_location_position_to_2d(&(new_point_3d + Vector3::new(0.0, 0.0, -3.0)))
                .unwrap(),
        );
        builder.move_to(Point::new(new_point.x, new_point.y));
        builder.line_to(Point::new(
            new_point_helper_negative.x,
            new_point_helper_negative.y,
        ));
        let path = builder.build();
        frame.stroke(
            &path,
            Stroke {
                style: canvas::Style::Solid(Color::WHITE),
                width: 0.5,
                ..Stroke::default()
            },
        );
    }
}

impl<Message, Theme, Renderer> Widget<Message, Theme, Renderer>
    for ComputeCameraPose<Message, Theme, Renderer>
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
        &mut self,
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
            self.cache.clear();
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
    pub selected: usize,
    pub highlight_axis_line: Option<usize>,
    pub edit: Option<Component>,
    pub image_path: String,
    pub edit_state: Edit,
    pub points: RefCell<Vec<Point>>,
    pub reference_cub_2d: RefCell<Vec<(Point, Point)>>,
    pub captured: Option<Vector>,
    pub captured_delta: f32,
    pub vanishing_points: RefCell<(Vector2<f32>, Vector2<f32>, Vector2<f32>)>,
    pub selected_match_point: Option<usize>,
}

impl<'a, Message, Theme, Renderer> From<ComputeCameraPose<Message, Theme, Renderer>>
    for Element<'a, Message, Theme, Renderer>
where
    Message: 'a,
    Theme: 'a,
    Renderer: 'a + geometry::Renderer,
{
    fn from(
        axis_decoration: ComputeCameraPose<Message, Theme, Renderer>,
    ) -> Element<'a, Message, Theme, Renderer> {
        Element::new(axis_decoration)
    }
}
