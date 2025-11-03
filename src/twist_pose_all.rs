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
    widget::canvas::{self, Event, Fill, Stroke, Text},
};
use nalgebra::{Point2, Point3, Vector2};

use crate::{
    Component,
    compute::data::ComputeSolution,
    utils::{scale_point, scale_point_to_canvas, to_canvas},
};

pub struct ComputeCameraPoseTwist<'a, Message, Theme = iced::Theme, Renderer = iced::Renderer>
where
    Renderer: geometry::Renderer,
{
    width: Length,
    height: Length,
    message_: PhantomData<Message>,
    referece_cub_cache: geometry::Cache<Renderer>,
    twist_points_cache: geometry::Cache<Renderer>,

    compute_solution: RefCell<Option<ComputeSolution<f32>>>,
    renderer_: PhantomData<Renderer>,
    theme_: PhantomData<Theme>,
    image_size: Size<f32>,
    reference_cub: Rc<RefCell<Vec<Point3<f32>>>>,
    twist_points: Rc<RefCell<Vec<Point3<f32>>>>,
    twist_points_2d: Rc<RefCell<Vec<Point2<f32>>>>,
    on_points_move: Box<dyn Fn() -> Message + 'a>,
}
impl<'a, M, Theme, Renderer> ComputeCameraPoseTwist<'a, M, Theme, Renderer>
where
    Renderer: geometry::Renderer,
{
    const DEFAULT_SIZE: f32 = 100.0;
    pub fn new(
        reference_cub: Rc<RefCell<Vec<Point3<f32>>>>,
        compute_solution: &'a Option<ComputeSolution<f32>>,
        twist_points: Rc<RefCell<Vec<Point3<f32>>>>,
        twist_points_2d: Rc<RefCell<Vec<Point2<f32>>>>,
        on_points_move: impl Fn() -> M + 'a,
    ) -> Self {
        ComputeCameraPoseTwist {
            width: Length::Fixed(Self::DEFAULT_SIZE),
            height: Length::Fixed(Self::DEFAULT_SIZE),
            compute_solution: RefCell::new(compute_solution.clone()),
            message_: PhantomData,
            renderer_: PhantomData,
            theme_: PhantomData,
            referece_cub_cache: geometry::Cache::default(),
            twist_points_cache: geometry::Cache::default(),
            reference_cub,
            image_size: Size::default(),
            twist_points,
            twist_points_2d,
            on_points_move: Box::new(on_points_move),
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
        event: &Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Status {
        let Some(cursor) = cursor.position_over(bounds) else {
            return Status::Ignored;
        };
        let adjusted_cursor = cursor - bounds.position();
        let scale_cursor = scale_point(adjusted_cursor, bounds.size());
        match event {
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let clicked_position = scale_cursor;
                state.captured = Some(Vector::new(clicked_position.x, clicked_position.y));
                let cursor = Point::new(adjusted_cursor.x, adjusted_cursor.y);
                self.twist_points_2d
                    .borrow()
                    .iter()
                    .enumerate()
                    .for_each(|(index, item)| {
                        let item =
                            scale_point_to_canvas(&Point::new(item.x, item.y), bounds.size());
                        if cursor.distance(item) < 10.0 {
                            state.selected_twist_point = Some(index);
                        }
                    });
                Status::Captured
            }

            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                state.selected_twist_point = None;
                self.twist_points_cache.clear();
                Status::Captured
            }

            Event::Mouse(mouse::Event::CursorMoved { position: _ }) => {
                if let Some(selected_twist_point) = state.selected_twist_point {
                    *self
                        .twist_points_2d
                        .borrow_mut()
                        .get_mut(selected_twist_point)
                        .unwrap() = Point2::new(scale_cursor.x, scale_cursor.y);
                    self.twist_points_cache.clear();
                    Status::Captured
                } else {
                    Status::Ignored
                }
            }
            _ => Status::Ignored,
        }
    }

    fn draw_inner(
        &self,
        state: &State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Renderer::Geometry> {
        let referece_cub_cache = self
            .referece_cub_cache
            .draw(renderer, bounds.size(), |frame| {
                if self.compute_solution.borrow().as_ref().is_none() {
                    return;
                }

                let mut builder = canvas::path::Builder::new();

                self.reference_cub
                    .as_ref()
                    .borrow()
                    .chunks(2)
                    .for_each(|points| {
                        self.compute_solution
                            .borrow()
                            .as_ref()
                            .unwrap()
                            .calculate_location_position_to_2d_frustum(points)
                            .iter()
                            .for_each(|&(start, end)| {
                                let start = to_canvas(bounds.size(), &start.coords.xy());
                                let end = to_canvas(bounds.size(), &end.coords.xy());
                                builder.move_to(Point::new(start.x, start.y));
                                builder.line_to(Point::new(end.x, end.y));
                            });
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

        let twist_point = self
            .twist_points_cache
            .draw(renderer, bounds.size(), |frame| {
                let selected_color = Color::from_rgba(0.8, 0.8, 0.2, 0.8);
                if let Some(selected) = state.selected_twist_point {
                    if let Some(item) = self.twist_points_2d.borrow().get(selected) {
                        let item =
                            scale_point_to_canvas(&Point::new(item.x, item.y), bounds.size());
                        let mut builder = canvas::path::Builder::new();
                        builder.circle(item, 5.0);
                        let path = builder.build();
                        frame.fill_rectangle(
                            Point::new(item.x + 2.0, item.y + 2.0),
                            Size::new(100.0, 15.0),
                            Fill {
                                style: canvas::Style::Solid(Color::from_rgba(0.3, 0.3, 0.3, 0.9)),
                                ..Fill::default()
                            },
                        );

                        if let Some(twist_point) = self.twist_points.borrow().get(selected) {
                            frame.fill_text(Text {
                                content: format!(
                                    "{:>7.2},{:>7.2},{:>7.2}",
                                    twist_point.x, twist_point.y, twist_point.z
                                ),
                                position: Point::new(item.x + 4.0, item.y + 4.0),
                                color: Color::from_rgba(0.8, 0.8, 0.8, 0.8),
                                size: Pixels(10.0),
                                ..Default::default()
                            });
                            frame.stroke(
                                &path,
                                Stroke {
                                    style: canvas::Style::Solid(selected_color),
                                    width: 2.0,
                                    ..Stroke::default()
                                },
                            );
                        }
                    }
                } else {
                    self.twist_points_2d
                        .borrow()
                        .iter()
                        .enumerate()
                        .for_each(|(selected, item)| {
                            let item =
                                scale_point_to_canvas(&Point::new(item.x, item.y), bounds.size());
                            let mut builder = canvas::path::Builder::new();
                            builder.circle(item, 5.0);
                            let path = builder.build();
                            frame.fill_rectangle(
                                Point::new(item.x + 2.0, item.y + 2.0),
                                Size::new(100.0, 15.0),
                                Fill {
                                    style: canvas::Style::Solid(Color::from_rgba(
                                        0.3, 0.3, 0.3, 0.9,
                                    )),
                                    ..Fill::default()
                                },
                            );

                            if let Some(twist_point) = self.twist_points.borrow().get(selected) {
                                frame.fill_text(Text {
                                    content: format!(
                                        "{:>7.2},{:>7.2},{:>7.2}",
                                        twist_point.x, twist_point.y, twist_point.z
                                    ),
                                    position: Point::new(item.x + 4.0, item.y + 4.0),
                                    color: Color::from_rgba(0.8, 0.8, 0.8, 0.8),
                                    size: Pixels(10.0),
                                    ..Default::default()
                                });
                                frame.stroke(
                                    &path,
                                    Stroke {
                                        style: canvas::Style::Solid(selected_color),
                                        width: 2.0,
                                        ..Stroke::default()
                                    },
                                );
                            }
                        })
                };
            });

        vec![twist_point, referece_cub_cache]
    }
}

impl<Message, Theme, Renderer> Widget<Message, Theme, Renderer>
    for ComputeCameraPoseTwist<'_, Message, Theme, Renderer>
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

        let event_status = self.update_inner(state, event, bounds, cursor);
        if let Status::Captured = event_status {
            shell.capture_event();
            if let Event::Mouse(mouse::Event::CursorMoved { position: _ }) = event {
                shell.publish((self.on_points_move)());
            } else {
                shell.request_redraw();
            }
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
    pub edit: Option<Component>,
    pub image_path: String,
    pub captured: Option<Vector>,
    pub vanishing_points: RefCell<(Vector2<f32>, Vector2<f32>, Vector2<f32>)>,
    pub selected_twist_point: Option<usize>,
}

impl<'a, Message, Theme, Renderer> From<ComputeCameraPoseTwist<'a, Message, Theme, Renderer>>
    for Element<'a, Message, Theme, Renderer>
where
    Message: 'a,
    Theme: 'a,
    Renderer: 'a + geometry::Renderer,
{
    fn from(
        axis_decoration: ComputeCameraPoseTwist<'a, Message, Theme, Renderer>,
    ) -> Element<'a, Message, Theme, Renderer> {
        Element::new(axis_decoration)
    }
}
