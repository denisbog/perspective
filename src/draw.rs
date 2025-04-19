use std::{cell::RefCell, f32, marker::PhantomData, rc::Rc};

use iced::{
    Color, Element, Length, Pixels, Point, Rectangle, Size, Vector,
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
    event::{self, Status},
    keyboard::{self, Key},
    widget::canvas::{self, Event, Stroke, Text},
};
use nalgebra::{Vector2, Vector3};

use crate::{
    Component, Edit, EditAxis, PointInformation,
    compute::ComputeSolution,
    utils::{
        calculate_cursor_position_to_3d, calculate_location_position_to_2d,
        check_if_point_is_from_line_new, to_canvas,
    },
};

pub struct DrawLine<'a, Message, Theme = iced::Theme, Renderer = iced::Renderer>
where
    Renderer: geometry::Renderer,
{
    width: Length,
    height: Length,
    message_: PhantomData<Message>,

    draw_cache: geometry::Cache<Renderer>,
    draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
    draw_lines_cache: geometry::Cache<Renderer>,

    compute_solution: &'a Option<ComputeSolution<f32>>,
    renderer_: PhantomData<Renderer>,
    theme_: PhantomData<Theme>,
    image_size: Size<f32>,
    custom_origin_translation: Rc<RefCell<Option<Vector3<f32>>>>,
    custom_scale_segment: Rc<RefCell<Option<usize>>>,
    custom_scale: Rc<RefCell<Option<PointInformation<f32>>>>,
    custom_error: Rc<RefCell<Option<PointInformation<f32>>>>,
}
impl<'a, Message, Theme, Renderer> DrawLine<'a, Message, Theme, Renderer>
where
    Renderer: geometry::Renderer,
{
    const DEFAULT_SIZE: f32 = 100.0;
    pub fn new(
        compute_solution: &'a Option<ComputeSolution<f32>>,
        draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
        custom_origin_translation: Rc<RefCell<Option<Vector3<f32>>>>,
        custom_scale_segment: Rc<RefCell<Option<usize>>>,
        custom_scale: Rc<RefCell<Option<PointInformation<f32>>>>,
        custom_error: Rc<RefCell<Option<PointInformation<f32>>>>,
    ) -> Self {
        Self {
            width: Length::Fixed(Self::DEFAULT_SIZE),
            height: Length::Fixed(Self::DEFAULT_SIZE),
            compute_solution,
            message_: PhantomData,
            renderer_: PhantomData,
            theme_: PhantomData,
            image_size: Size::default(),
            draw_cache: geometry::Cache::default(),
            draw_lines_cache: geometry::Cache::default(),
            draw_lines,
            custom_origin_translation,
            custom_scale_segment,
            custom_scale,
            custom_error,
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

    fn update_inner(
        &self,
        state: &mut State,
        event: Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> (Status, Option<Message>) {
        let Some(cursor) = cursor.position_over(bounds) else {
            return (Status::Ignored, None);
        };
        let adjusted_cursor = cursor - bounds.position();
        match event {
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right)) => {
                state.edit_state = Edit::Draw;
                (Status::Ignored, None)
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                if let Edit::Draw = state.edit_state {
                    let cursor = Point::new(adjusted_cursor.x, adjusted_cursor.y);
                    for (index, point) in state.points.borrow().iter().enumerate() {
                        if cursor.distance(*point) < 10.0 {
                            state.selected = index;
                            self.custom_origin_translation
                                .replace(self.draw_lines.borrow().get(index).copied());
                            self.draw_cache.clear();
                            return (Status::Ignored, None);
                        };
                    }

                    state
                        .points
                        .borrow()
                        .windows(2)
                        .enumerate()
                        .for_each(|(index, items)| {
                            let start = items[0];
                            let end = items[1];
                            if check_if_point_is_from_line_new(&start, &end, cursor) {
                                self.custom_scale_segment.borrow_mut().replace(index);
                                self.draw_cache.clear();
                            }
                        });
                }
                if let Edit::MarkError(_axis) = &state.edit_state {
                    let cursor = Point::new(adjusted_cursor.x, adjusted_cursor.y);
                    for (index, point) in state.points.borrow().iter().enumerate() {
                        if cursor.distance(*point) < 10.0 {
                            state.selected = index;
                            self.draw_cache.clear();
                            return (Status::Ignored, None);
                        };
                    }
                }
                (Status::Ignored, None)
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                let Some((new_point_3d, _last_point_3d, _color)) =
                    self.extract_last_point_details_for_mode(state, bounds, &adjusted_cursor)
                else {
                    return (Status::Ignored, None);
                };

                match &state.edit_state {
                    Edit::Extrude(_axis) => {
                        self.draw_lines.borrow_mut().push(new_point_3d);
                        self.draw_lines_cache.clear();
                        self.draw_cache.clear();
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
                        self.draw_cache.clear();
                        state.edit_state = Edit::Draw;
                    }
                    Edit::MarkError(axis) => {
                        self.custom_error.borrow_mut().replace(PointInformation {
                            vector: new_point_3d,
                            source_vector: *self.draw_lines.borrow().get(state.selected).unwrap(),
                            point: Vector2::new(
                                adjusted_cursor.x / bounds.width,
                                adjusted_cursor.y / bounds.height,
                            ),
                            axis: axis.clone(),
                        });
                        self.draw_lines_cache.clear();
                        self.draw_cache.clear();
                        state.edit_state = Edit::Draw;
                    }
                    _ => (),
                }
                (Status::Ignored, None)
            }
            Event::Mouse(mouse::Event::CursorMoved { position: _ }) => {
                match state.edit_state {
                    Edit::Extrude(_) | Edit::Scale(_) | Edit::MarkError(_) => {
                        self.draw_cache.clear()
                    }
                    _ => (),
                };
                (Status::Ignored, None)
            }
            Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => {
                if let Key::Character(c) = key {
                    let c = c.as_str();
                    match c {
                        "x" => state.edit_state = Edit::Scale(EditAxis::None),
                        "r" => match state.edit_state {
                            Edit::Extrude(_) => state.edit_state = Edit::Extrude(EditAxis::EditX),
                            Edit::Scale(_) => state.edit_state = Edit::Scale(EditAxis::EditX),
                            Edit::MarkError(_) => {
                                state.edit_state = Edit::MarkError(EditAxis::EditX)
                            }
                            _ => (),
                        },
                        "s" => match state.edit_state {
                            Edit::Extrude(_) => state.edit_state = Edit::Extrude(EditAxis::EditY),
                            Edit::Scale(_) => state.edit_state = Edit::Scale(EditAxis::EditY),
                            Edit::MarkError(_) => {
                                state.edit_state = Edit::MarkError(EditAxis::EditY)
                            }
                            _ => (),
                        },
                        "t" => match state.edit_state {
                            Edit::Extrude(_) => state.edit_state = Edit::Extrude(EditAxis::EditZ),
                            Edit::Scale(_) => state.edit_state = Edit::Scale(EditAxis::EditZ),
                            Edit::MarkError(_) => {
                                state.edit_state = Edit::MarkError(EditAxis::EditZ)
                            }
                            _ => (),
                        },
                        "c" => state.edit_state = Edit::Extrude(EditAxis::None),
                        "d" => {
                            if self.draw_lines.borrow().len() > 1 {
                                self.draw_lines.borrow_mut().pop();
                                self.draw_lines_cache.clear();
                                self.draw_cache.clear();
                            }
                            state.edit_state = Edit::Draw
                        }
                        "q" => state.edit_state = Edit::MarkError(EditAxis::None),
                        _ => state.edit_state = Edit::Draw,
                    }
                    self.draw_cache.clear();
                }
                (Status::Ignored, None)
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
        let draw_lines_cache = self
            .draw_lines_cache
            .draw(renderer, bounds.size(), |frame| {
                *state.points.borrow_mut() = self
                    .draw_lines
                    .borrow()
                    .iter()
                    .flat_map(|item| calculate_location_position_to_2d(self.compute_solution, item))
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
                        let location3d = *self.draw_lines.borrow().get(index + 1).unwrap();

                        frame.fill_text(Text {
                            content: format!(
                                "{:>7.2},{:>7.2},{:>7.2}",
                                location3d.x, location3d.y, location3d.z
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
                        width: 1.5,
                        ..Stroke::default()
                    },
                );
            });

        let draw_cache = self.draw_cache.draw(renderer, bounds.size(), |frame| {
            let selected_color = match &state.edit_state {
                Edit::MarkError(_) => Color::from_rgba(0.8, 0.2, 0.2, 0.8),
                Edit::ControlPoint => Color::from_rgba(0.8, 0.8, 0.2, 0.8),
                Edit::Draw => Color::from_rgba(0.8, 0.8, 0.2, 0.8),
                Edit::Extrude(_) => Color::from_rgba(0.8, 0.8, 0.8, 0.8),
                Edit::Scale(_) => Color::from_rgba(0.2, 0.8, 0.2, 0.8),
                Edit::None => Color::from_rgba(0.8, 0.8, 0.2, 0.8),
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
                                style: canvas::Style::Solid(Color::from_rgba(0.8, 0.8, 0.2, 0.8)),
                                width: 2.0,
                                ..Stroke::default()
                            },
                        );
                    });
            }
            if let Edit::Draw = state.edit_state {
                if let Some(scale) = self.custom_scale.borrow().as_ref() {
                    let start = to_canvas(
                        bounds.size(),
                        &calculate_location_position_to_2d(
                            self.compute_solution,
                            &scale.source_vector,
                        )
                        .unwrap(),
                    );
                    let end = to_canvas(
                        bounds.size(),
                        &calculate_location_position_to_2d(self.compute_solution, &scale.vector)
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
                if let Some(custom_error) = self.custom_error.borrow().as_ref() {
                    let start = to_canvas(
                        bounds.size(),
                        &calculate_location_position_to_2d(
                            self.compute_solution,
                            &custom_error.source_vector,
                        )
                        .unwrap(),
                    );
                    let end = to_canvas(
                        bounds.size(),
                        &calculate_location_position_to_2d(
                            self.compute_solution,
                            &custom_error.vector,
                        )
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
                            style: canvas::Style::Solid(Color::from_rgba(0.8, 0.2, 0.2, 0.8)),
                            width: 2.0,
                            ..Stroke::default()
                        },
                    );
                }
            }

            let Some(cursor) = cursor.position() else {
                return;
            };
            let cursor = cursor - bounds.position();

            let Some((new_point_3d, last_point_3d, color)) =
                self.extract_last_point_details_for_mode(state, bounds, &cursor)
            else {
                return;
            };

            let last_point = to_canvas(
                bounds.size(),
                &calculate_location_position_to_2d(self.compute_solution, &last_point_3d).unwrap(),
            );

            let new_point = to_canvas(
                bounds.size(),
                &calculate_location_position_to_2d(self.compute_solution, &new_point_3d).unwrap(),
            );

            let mut builder = canvas::path::Builder::new();
            frame.fill_text(Text {
                content: format!(
                    "{:>5.2},\n{:>5.2},\n{:>5.2}",
                    new_point_3d.x, new_point_3d.y, new_point_3d.z
                ),
                position: Point::new(new_point.x + 8.0, new_point.y + 8.0),
                color,
                size: Pixels(12.0),
                ..Default::default()
            });

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
        });
        vec![draw_lines_cache, draw_cache]
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
            Edit::MarkError(axis) => {
                let last_point_3d = *self.draw_lines.borrow().get(state.selected)?;
                (axis, last_point_3d, Color::from_rgba(0.8, 0.2, 0.2, 0.8))
            }
            _w => {
                return None;
            }
        };

        let new_point_3d = calculate_cursor_position_to_3d(
            axis,
            self.compute_solution.as_ref().unwrap(),
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
}

impl<'a, Message, Theme, Renderer> Widget<Message, Theme, Renderer>
    for DrawLine<'a, Message, Theme, Renderer>
where
    Renderer: geometry::Renderer,
{
    fn tag(&self) -> tree::Tag {
        struct Tag<T>(T);
        tree::Tag::of::<Tag<State>>()
    }

    fn state(&self) -> tree::State {
        tree::State::new(State {
            edit_state: Edit::Draw,
            ..Default::default()
        })
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

    fn on_event(
        &mut self,
        tree: &mut Tree,
        event: iced::Event,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        _renderer: &Renderer,
        _clipboard: &mut dyn Clipboard,
        _shell: &mut Shell<'_, Message>,
        _viewport: &Rectangle,
    ) -> event::Status {
        let bounds = layout.bounds();
        let canvas_event = match event {
            iced::Event::Mouse(mouse_event) => Some(Event::Mouse(mouse_event)),
            iced::Event::Touch(touch_event) => Some(Event::Touch(touch_event)),
            iced::Event::Keyboard(keyboard_event) => Some(Event::Keyboard(keyboard_event)),
            iced::Event::Window(_) => None,
        };

        if let Some(canvas_event) = canvas_event {
            let state = tree.state.downcast_mut::<State>();

            let (event_status, _message) = self.update_inner(state, canvas_event, bounds, cursor);
            //if let Some(message) = message {
            //    self.handle_internal_event(state, message);
            //}

            //if let Some(message) = message {
            //    shell.publish(message);
            //}

            return event_status;
        }

        event::Status::Ignored
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
        //match state.edit_state {
        //    Edit::Extrude(_) => mouse::Interaction::Crosshair,
        //    Edit::Scale(_) => mouse::Interaction::ZoomOut,
        //    _ => mouse::Interaction::default(),
        //}
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
    pub first_point: Point,
    pub selected: usize,
    pub is_second_point: bool,
    pub highlight: Option<usize>,
    pub edit: Option<Component>,
    pub image_path: String,
    pub mouse3d_position: Vector3<f32>,
    pub edit_state: Edit,
    pub points: RefCell<Vec<Point>>,
}

impl<'a, Message, Theme, Renderer> From<DrawLine<'a, Message, Theme, Renderer>>
    for Element<'a, Message, Theme, Renderer>
where
    Message: 'a,
    Theme: 'a,
    Renderer: 'a + geometry::Renderer,
{
    fn from(
        axis_decoration: DrawLine<'a, Message, Theme, Renderer>,
    ) -> Element<'a, Message, Theme, Renderer> {
        Element::new(axis_decoration)
    }
}
