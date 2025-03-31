use std::{cell::RefCell, f32, marker::PhantomData, rc::Rc};

use iced::{
    Color, Element, Length, Point, Rectangle, Size, Vector,
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
use nalgebra::{Matrix3, Perspective3, Point3, Vector2, Vector3};

use crate::{
    Component, Edit,
    compute::{ComputeSolution, line_insert_with_plane, relative_to_image_plane},
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

    compute_solution: &'a Option<ComputeSolution>,
    renderer_: PhantomData<Renderer>,
    theme_: PhantomData<Theme>,
    image_size: Size<f32>,
}
impl<'a, Message, Theme, Renderer> DrawLine<'a, Message, Theme, Renderer>
where
    Renderer: geometry::Renderer,
{
    const DEFAULT_SIZE: f32 = 100.0;
    pub fn new(
        compute_solution: &'a Option<ComputeSolution>,
        draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
    ) -> Self {
        DrawLine {
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
        let cursor = cursor - bounds.position();
        match event {
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right)) => {
                if self.draw_lines.borrow().len() > 1 {
                    self.draw_lines.borrow_mut().pop();
                    self.draw_lines_cache.clear();
                    self.draw_cache.clear();
                }
                state.draw = false;
                (Status::Ignored, None)
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                state.draw = true;
                (Status::Ignored, None)
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                let Some(new_point_3d) =
                    self.calculate_cursor_position_to_3d(state, bounds, &cursor)
                else {
                    return (Status::Ignored, None);
                };
                let last_point_3d = *self.draw_lines.borrow().last().unwrap();
                let location3d = match state.edit_state {
                    Edit::EditX => Vector3::new(new_point_3d.x, last_point_3d.y, last_point_3d.z),
                    Edit::EditY => Vector3::new(last_point_3d.x, new_point_3d.y, last_point_3d.z),
                    Edit::EditZ => Vector3::new(last_point_3d.x, last_point_3d.y, new_point_3d.z),
                    _ => new_point_3d,
                };
                self.draw_lines.borrow_mut().push(location3d);
                self.draw_lines_cache.clear();
                self.draw_cache.clear();
                state.draw = false;
                (Status::Ignored, None)
            }
            Event::Mouse(mouse::Event::CursorMoved { position: _ }) => {
                if state.draw {
                    self.draw_cache.clear();
                }
                (Status::Ignored, None)
            }
            Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => {
                if let Key::Character(c) = key {
                    let c = c.as_str();
                    match c {
                        "r" => state.edit_state = Edit::EditX,
                        "s" => state.edit_state = Edit::EditY,
                        "t" => state.edit_state = Edit::EditZ,
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
                let points: Vec<Point> = self
                    .draw_lines
                    .borrow()
                    .iter()
                    .flat_map(|item| self.calculate_location_position_to_2d(state, bounds, item))
                    .map(|item| Point::new(item.x, item.y))
                    .collect();

                let mut builder = canvas::path::Builder::new();
                points.windows(2).for_each(|items| {
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
                        width: 1.5,
                        ..Stroke::default()
                    },
                );
            });

        let draw_cache = self.draw_cache.draw(renderer, bounds.size(), |frame| {
            if !state.draw {
                return;
            }
            let Some(cursor) = cursor.position() else {
                return;
            };
            let cursor = cursor - bounds.position();

            let Some(new_point_3d) = self.calculate_cursor_position_to_3d(state, bounds, &cursor)
            else {
                return;
            };
            let last_point_3d = &self.draw_lines.borrow().last().unwrap().clone();
            let location3d = match state.edit_state {
                Edit::EditX => Vector3::new(new_point_3d.x, last_point_3d.y, last_point_3d.z),
                Edit::EditY => Vector3::new(last_point_3d.x, new_point_3d.y, last_point_3d.z),
                Edit::EditZ => Vector3::new(last_point_3d.x, last_point_3d.y, new_point_3d.z),
                _ => new_point_3d,
            };

            let Some(last_point) =
                self.calculate_location_position_to_2d(state, bounds, last_point_3d)
            else {
                return;
            };
            let Some(new_point) =
                self.calculate_location_position_to_2d(state, bounds, &location3d)
            else {
                return;
            };

            let mut builder = canvas::path::Builder::new();
            frame.fill_text(Text {
                content: format!(
                    "{:>5.2},\n{:>5.2},\n{:>5.2}",
                    location3d.x, location3d.y, location3d.z
                ),
                position: Point::new(cursor.x + 10.0, cursor.y + 10.0),
                color: Color::from_rgba(0.8, 0.8, 0.8, 0.8),
                ..Default::default()
            });

            builder.move_to(Point::new(last_point.x, last_point.y));
            builder.line_to(Point::new(new_point.x, new_point.y));
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
        vec![draw_lines_cache, draw_cache]
    }

    fn calculate_cursor_position_to_3d(
        &self,
        state: &State,
        bounds: Rectangle,
        cursor: &Vector,
    ) -> Option<Vector3<f32>> {
        let Some(compute_solution) = &self.compute_solution else {
            return None;
        };
        let click_location = relative_to_image_plane(
            self.image_size.width / self.image_size.height,
            &Vector2::new(cursor.x / bounds.width, cursor.y / bounds.height),
        );

        let perspective = Perspective3::new(1.0, compute_solution.field_of_view, 0.01, 10.0);

        let mut matrix = perspective.into_inner();
        *matrix.index_mut((0, 2)) = -compute_solution.ortho_center.x;
        *matrix.index_mut((1, 2)) = -compute_solution.ortho_center.y;

        let model_view_projection = matrix * compute_solution.view_transform;
        let model_view_projection = model_view_projection.try_inverse().unwrap();
        let last_point_axis = Vector3::zeros();
        let point = model_view_projection * Point3::from(last_point_axis).to_homogeneous();
        let point3d1 = Point3::from_homogeneous(point).unwrap();

        let point = Point3::new(click_location.x, click_location.y, 1.0).to_homogeneous();
        let point = model_view_projection * point;

        let point3d2 = Point3::from_homogeneous(point).unwrap();
        let last_point = self.draw_lines.borrow().last().cloned().unwrap();
        let axis = if let Edit::EditZ = state.edit_state {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        };
        let intersection1_3d =
            line_insert_with_plane(&last_point, &axis, &point3d1.coords, &point3d2.coords);
        Some(intersection1_3d)
    }

    fn calculate_location_position_to_2d(
        &self,
        _state: &State,
        bounds: Rectangle,
        location3d: &Vector3<f32>,
    ) -> Option<Vector2<f32>> {
        let Some(compute_solution) = &self.compute_solution else {
            return None;
        };
        let dc_to_image =
            Matrix3::new_nonuniform_scaling(&Vector2::new(bounds.width / 2.0, bounds.width / -2.0))
                .append_translation(&Vector2::new(bounds.width / 2.0, bounds.height / 2.0));

        let perspective = Perspective3::new(1.0, compute_solution.field_of_view, 0.01, 10.0);

        let mut matrix = perspective.into_inner();
        *matrix.index_mut((0, 2)) = -compute_solution.ortho_center.x;
        *matrix.index_mut((1, 2)) = -compute_solution.ortho_center.y;

        let transform = matrix * compute_solution.view_transform;
        let point = Point3::from(location3d.clone());

        let point = transform * point.to_homogeneous();
        let point = Point3::from_homogeneous(point).unwrap();

        Some(dc_to_image.transform_point(&point.xy()).coords)
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
    pub draw: bool,
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
