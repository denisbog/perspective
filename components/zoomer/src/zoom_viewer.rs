use iced::{
    ContentFit, Element, Event, Length, Point, Radians, Rectangle, Size, Vector,
    advanced::{
        Clipboard, Layout, Shell, Widget, image, layout, mouse, renderer,
        widget::{
            Tree,
            tree::{self, Tag},
        },
    },
    widget::image::FilterMethod,
};

pub fn zoomer<Handle>(handle: impl Into<Handle>) -> ZoomViewer<Handle> {
    ZoomViewer::new(handle)
}

pub struct ZoomViewer<Handle> {
    width: Length,
    height: Length,
    zoomer_width: f32,
    zoomer_height: f32,
    scale: f32,
    handle: Handle,
    filter_method: FilterMethod,
    content_fit: ContentFit,
}

impl<Handle> ZoomViewer<Handle> {
    pub fn new<T: Into<Handle>>(handle: T) -> Self {
        ZoomViewer {
            handle: handle.into(),
            width: Length::Shrink,
            height: Length::Shrink,
            zoomer_width: 100.0,
            zoomer_height: 100.0,
            scale: 3.0,
            filter_method: FilterMethod::default(),
            content_fit: ContentFit::default(),
        }
    }

    pub fn filter_method(mut self, filter_method: FilterMethod) -> Self {
        self.filter_method = filter_method;
        self
    }

    /// Sets the [`ContentFit`] of the [`Viewer`].
    pub fn content_fit(mut self, content_fit: ContentFit) -> Self {
        self.content_fit = content_fit;
        self
    }

    /// Sets the width of the [`Viewer`].
    pub fn width(mut self, width: impl Into<Length>) -> Self {
        self.width = width.into();
        self
    }

    /// Sets the height of the [`Viewer`].
    pub fn height(mut self, height: impl Into<Length>) -> Self {
        self.height = height.into();
        self
    }

    pub fn scale(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }
}

impl<Message, Theme, Renderer, Handle> Widget<Message, Theme, Renderer> for ZoomViewer<Handle>
where
    Renderer: image::Renderer<Handle = Handle>,
    Handle: Clone,
{
    fn tag(&self) -> Tag {
        tree::Tag::of::<State>()
    }

    fn state(&self) -> tree::State {
        tree::State::new(State::new())
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
        renderer: &Renderer,
        limits: &layout::Limits,
    ) -> layout::Node {
        // The raw w/h of the underlying image
        let image_size = renderer.measure_image(&self.handle);
        let image_size = Size::new(image_size.width as f32, image_size.height as f32);

        // The size to be available to the widget prior to `Shrink`ing
        let raw_size = limits.resolve(self.width, self.height, image_size);

        // The uncropped size of the image when fit to the bounds above
        let full_size = self.content_fit.fit(image_size, raw_size);

        // Shrink the widget to fit the resized image, if requested
        let final_size = Size {
            width: match self.width {
                Length::Shrink => f32::min(raw_size.width, full_size.width),
                _ => raw_size.width,
            },
            height: match self.height {
                Length::Shrink => f32::min(raw_size.height, full_size.height),
                _ => raw_size.height,
            },
        };

        layout::Node::new(final_size)
    }

    fn update(
        &mut self,
        tree: &mut Tree,
        event: &Event,
        _layout: Layout<'_>,
        _cursor: mouse::Cursor,
        _renderer: &Renderer,
        _clipboard: &mut dyn Clipboard,
        _shell: &mut Shell<'_, Message>,
        _viewport: &Rectangle,
    ) {
        let state = tree.state.downcast_mut::<State>();
        if let Event::Keyboard(iced::keyboard::Event::ModifiersChanged(modifiers)) = event {
            state.zoom = modifiers.shift()
        };
    }

    fn mouse_interaction(
        &self,
        tree: &Tree,
        _layout: Layout<'_>,
        _cursor: mouse::Cursor,
        _viewport: &Rectangle,
        _renderer: &Renderer,
    ) -> mouse::Interaction {
        let state = tree.state.downcast_ref::<State>();
        if state.is_zoom() {
            mouse::Interaction::Crosshair
        } else {
            mouse::Interaction::None
        }
    }

    fn draw(
        &self,
        tree: &Tree,
        renderer: &mut Renderer,
        _theme: &Theme,
        _style: &renderer::Style,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        _viewport: &Rectangle,
    ) {
        let bounds = layout.bounds();
        //render origial image
        renderer.draw_image(
            image::Image {
                handle: self.handle.clone(),
                filter_method: self.filter_method,
                rotation: Radians(0.0),
                opacity: 0.9,
                snap: true,
            },
            bounds,
        );

        let state = tree.state.downcast_ref::<State>();
        if state.is_zoom()
            && let Some(cursor) = cursor.position_over(bounds)
        {
            let final_size = bounds.size() * self.scale;
            let drawing_bounds = Rectangle::new(
                Point::new(
                    bounds.center_x() - final_size.width / 2.0,
                    bounds.center_y() - final_size.height / 2.0,
                ),
                final_size,
            );

            let cursor_image_coordinates = bounds.center() - cursor;
            let translation = Vector::new(
                (final_size.width * ((self.scale - 1.0) / self.scale)) * cursor_image_coordinates.x
                    / bounds.width,
                (final_size.height * ((self.scale - 1.0) / self.scale))
                    * cursor_image_coordinates.y
                    / bounds.height,
            );
            let render = |renderer: &mut Renderer| {
                // translation
                renderer.with_translation(translation, |renderer| {
                    renderer.draw_image(
                        image::Image {
                            handle: self.handle.clone(),
                            filter_method: self.filter_method,
                            rotation: Radians(0.0),
                            opacity: 1.0,
                            snap: true,
                        },
                        drawing_bounds, //zooming
                    );
                });
            };
            // clipping
            renderer.with_layer(
                Rectangle::new(
                    Point::new(
                        cursor.x - self.zoomer_width / 2.0,
                        cursor.y - self.zoomer_height / 2.0,
                    ),
                    Size::new(self.zoomer_width, self.zoomer_height),
                ),
                render,
            );
        };
    }
}

/// The local state of a [`Viewer`].
#[derive(Default, Debug, Clone, Copy)]
pub struct State {
    zoom: bool,
}

impl State {
    /// Creates a new [`State`].
    pub fn new() -> Self {
        State::default()
    }

    /// Returns if the cursor is currently grabbed by the [`Viewer`].
    pub fn is_zoom(&self) -> bool {
        self.zoom
    }
}

impl<'a, Message, Theme, Renderer, Handle> From<ZoomViewer<Handle>>
    for Element<'a, Message, Theme, Renderer>
where
    Renderer: 'a + image::Renderer<Handle = Handle>,
    Message: 'a,
    Handle: Clone + 'a,
{
    fn from(viewer: ZoomViewer<Handle>) -> Element<'a, Message, Theme, Renderer> {
        Element::new(viewer)
    }
}

/// Returns the bounds of the underlying image, given the bounds of
/// the [`Viewer`]. Scaling will be applied and original aspect ratio
/// will be respected.
pub fn scaled_image_size<Renderer>(
    renderer: &Renderer,
    handle: &<Renderer as image::Renderer>::Handle,
    _state: &State,
    bounds: Size,
    content_fit: ContentFit,
) -> Size
where
    Renderer: image::Renderer,
{
    let Size { width, height } = renderer.measure_image(handle);
    let image_size = Size::new(width as f32, height as f32);

    let adjusted_fit = content_fit.fit(image_size, bounds);

    Size::new(adjusted_fit.width, adjusted_fit.height)
}
