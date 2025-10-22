use iced::{
    Element, Event, Length, Point, Rectangle, Vector,
    advanced::{
        Clipboard, Layout, Shell, Widget,
        layout::{Limits, Node},
        overlay, renderer,
        widget::{Operation, Tree, tree},
    },
    mouse::{self, Button, Cursor},
    overlay::menu::Catalog,
};

use crate::context_menu_overlay::ContextMenuOverlay;

pub struct ContextMenu<'a, Overlay, Message, Theme = iced::Theme, Renderer = iced::Renderer>
where
    Overlay: Fn() -> Element<'a, Message, Theme, Renderer>,
    Message: Clone,
    Renderer: renderer::Renderer,
{
    content: Element<'a, Message, Theme, Renderer>,
    overlay: Overlay,
}

impl<'a, Overlay, Message, Theme, Renderer> ContextMenu<'a, Overlay, Message, Theme, Renderer>
where
    Overlay: Fn() -> Element<'a, Message, Theme, Renderer>,
    Message: Clone,
    Renderer: renderer::Renderer,
{
    pub fn new(
        content: impl Into<Element<'a, Message, Theme, Renderer>>,
        overlay: Overlay,
    ) -> Self {
        ContextMenu {
            content: content.into(),
            overlay,
        }
    }
}

impl<'a, Content, Message, Theme, Renderer> Widget<Message, Theme, Renderer>
    for ContextMenu<'a, Content, Message, Theme, Renderer>
where
    Content: 'a + Fn() -> Element<'a, Message, Theme, Renderer>,
    Message: 'a + Clone,
    Renderer: 'a + renderer::Renderer,
{
    fn size(&self) -> iced::Size<Length> {
        self.content.as_widget().size()
    }

    fn layout(&mut self, tree: &mut Tree, renderer: &Renderer, limits: &Limits) -> Node {
        self.content
            .as_widget_mut()
            .layout(&mut tree.children[0], renderer, limits)
    }

    fn draw(
        &self,
        state: &Tree,
        renderer: &mut Renderer,
        theme: &Theme,
        style: &renderer::Style,
        layout: Layout<'_>,
        cursor: Cursor,
        viewport: &Rectangle,
    ) {
        self.content.as_widget().draw(
            &state.children[0],
            renderer,
            theme,
            style,
            layout,
            cursor,
            viewport,
        );
    }

    fn tag(&self) -> tree::Tag {
        tree::Tag::of::<State>()
    }

    fn state(&self) -> tree::State {
        tree::State::new(State::new())
    }

    fn children(&self) -> Vec<Tree> {
        vec![Tree::new(&self.content), Tree::new((self.overlay)())]
    }

    fn diff(&self, tree: &mut Tree) {
        tree.diff_children(&[&self.content, &(self.overlay)()]);
    }

    fn operate<'b>(
        &'b mut self,
        state: &'b mut Tree,
        layout: Layout<'_>,
        renderer: &Renderer,
        operation: &mut dyn Operation<()>,
    ) {
        let s: &mut State = state.state.downcast_mut();

        if s.show {
            let mut content = (self.overlay)();
            content.as_widget().diff(&mut state.children[1]);

            content
                .as_widget_mut()
                .operate(&mut state.children[1], layout, renderer, operation);
        } else {
            self.content.as_widget_mut().operate(
                &mut state.children[0],
                layout,
                renderer,
                operation,
            );
        }
    }

    fn update(
        &mut self,
        state: &mut Tree,
        event: &Event,
        layout: Layout<'_>,
        cursor: Cursor,
        renderer: &Renderer,
        clipboard: &mut dyn Clipboard,
        shell: &mut Shell<'_, Message>,
        viewport: &Rectangle,
    ) {
        self.content.as_widget_mut().update(
            &mut state.children[0],
            event,
            layout,
            cursor,
            renderer,
            clipboard,
            shell,
            viewport,
        );

        if !shell.is_event_captured()
            && *event == Event::Mouse(mouse::Event::ButtonPressed(Button::Right))
        {
            let bounds = layout.bounds();

            if cursor.is_over(bounds) {
                let s: &mut State = state.state.downcast_mut();
                s.cursor_position = cursor.position().unwrap_or_default();
                s.show = !s.show;
                shell.capture_event();
                shell.request_redraw();
            }
        }
    }

    fn mouse_interaction(
        &self,
        state: &Tree,
        layout: Layout<'_>,
        cursor: Cursor,
        viewport: &Rectangle,
        renderer: &Renderer,
    ) -> mouse::Interaction {
        self.content.as_widget().mouse_interaction(
            &state.children[0],
            layout,
            cursor,
            viewport,
            renderer,
        )
    }

    fn overlay<'b>(
        &'b mut self,
        state: &'b mut Tree,
        layout: Layout<'b>,
        renderer: &Renderer,
        viewport: &Rectangle,
        translation: Vector,
    ) -> Option<overlay::Element<'b, Message, Theme, Renderer>> {
        let s: &mut State = state.state.downcast_mut();

        if !s.show {
            return self.content.as_widget_mut().overlay(
                &mut state.children[0],
                layout,
                renderer,
                viewport,
                translation,
            );
        }

        let position = s.cursor_position;
        let content = (self.overlay)();
        content.as_widget().diff(&mut state.children[1]);
        Some(
            ContextMenuOverlay::new(position + translation, &mut state.children[1], content, s)
                .overlay(),
        )
    }
}

impl<'a, Content, Message, Theme, Renderer> From<ContextMenu<'a, Content, Message, Theme, Renderer>>
    for Element<'a, Message, Theme, Renderer>
where
    Content: 'a + Fn() -> Self,
    Message: 'a + Clone,
    Renderer: 'a + renderer::Renderer,
    Theme: 'a + Catalog,
{
    fn from(modal: ContextMenu<'a, Content, Message, Theme, Renderer>) -> Self {
        Element::new(modal)
    }
}

#[derive(Debug, Default)]
pub(crate) struct State {
    pub show: bool,
    pub cursor_position: Point,
}

impl State {
    pub const fn new() -> Self {
        Self {
            show: false,
            cursor_position: Point::ORIGIN,
        }
    }
}
