use iced::{
    Element, Event, Length, Rectangle, Size,
    advanced::{
        Clipboard, Layout, Shell, Widget, layout, mouse, renderer,
        widget::{
            Tree,
            tree::{self},
        },
    },
    alignment::Vertical,
    widget::{
        column, row, text, text_editor,
        text_editor::{Action, Content},
    },
};

pub fn twist_editor<'a, Message, Theme, Renderer>(
    content: &'a Content<Renderer>,
    on_edit: impl Fn(Action) -> Message + 'a,
) -> TwistEditor<'a, Message, Theme, Renderer>
where
    Message: 'a + Clone,
    Theme:
        'a + iced::widget::text::Catalog + text_editor::Catalog + iced::widget::container::Catalog,
    Renderer: iced::advanced::Renderer + iced::advanced::text::Renderer + 'a,
{
    TwistEditor::new(content, on_edit)
}

pub struct TwistEditor<'a, Message, Theme, Renderer>
where
    Theme: iced::widget::text::Catalog,
    Renderer: iced::advanced::text::Renderer,
    Message: 'a + Clone,
    Theme: 'a + iced::widget::text::Catalog + text_editor::Catalog,
{
    content: Element<'a, Message, Theme, Renderer>,
}
#[derive(Debug, Clone)]
enum Message {
    Edit(char),
}

impl<'a, Message, Theme, Renderer> TwistEditor<'a, Message, Theme, Renderer>
where
    Message: 'a + Clone,
    Theme:
        'a + iced::widget::text::Catalog + text_editor::Catalog + iced::widget::container::Catalog,
    Renderer: 'a + iced::advanced::text::Renderer,
{
    pub fn new(edit: &'a Content<Renderer>, on_edit: impl Fn(Action) -> Message + 'a) -> Self {
        Self {
            content: column!(
                row!(
                    text!("Point #1"),
                    text_editor(edit),
                    text_editor(edit),
                    text_editor(edit)
                )
                .align_y(Vertical::Center)
                .padding(5.0)
                .spacing(10.0),
                row!(
                    text!("Point #2"),
                    text_editor(edit),
                    text_editor(edit),
                    text_editor(edit)
                )
                .align_y(Vertical::Center)
                .padding(5.0)
                .spacing(10.0),
                row!(
                    text!("Point #3"),
                    text_editor(edit),
                    text_editor(edit),
                    text_editor(edit)
                )
                .align_y(Vertical::Center)
                .padding(5.0)
                .spacing(10.0)
            )
            .into(),
        }
    }
}

impl<'a, Message, Theme, Renderer> Widget<Message, Theme, Renderer>
    for TwistEditor<'a, Message, Theme, Renderer>
where
    Message: 'a + Clone,
    Theme: 'a + iced::widget::text::Catalog + text_editor::Catalog,
    Renderer: 'a + iced::advanced::text::Renderer,
{
    fn tag(&self) -> tree::Tag {
        tree::Tag::of::<State>()
    }

    fn state(&self) -> tree::State {
        tree::State::new(State::default())
    }

    fn children(&self) -> Vec<Tree> {
        vec![Tree::new(&self.content)]
    }

    fn diff(&self, tree: &mut Tree) {
        tree.diff_children(std::slice::from_ref(&self.content));
    }

    fn size(&self) -> Size<Length> {
        self.content.as_widget().size()
    }

    fn layout(
        &mut self,
        tree: &mut Tree,
        renderer: &Renderer,
        limits: &layout::Limits,
    ) -> layout::Node {
        self.content
            .as_widget_mut()
            .layout(&mut tree.children[0], renderer, limits)
    }

    fn update(
        &mut self,
        tree: &mut Tree,
        event: &Event,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        renderer: &Renderer,
        clipboard: &mut dyn Clipboard,
        shell: &mut Shell<'_, Message>,
        viewport: &Rectangle,
    ) {
        self.content.as_widget_mut().update(
            &mut tree.children[0],
            event,
            layout,
            cursor,
            renderer,
            clipboard,
            shell,
            viewport,
        );
    }

    fn mouse_interaction(
        &self,
        tree: &Tree,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        viewport: &Rectangle,
        renderer: &Renderer,
    ) -> mouse::Interaction {
        self.content.as_widget().mouse_interaction(
            &tree.children[0],
            layout,
            cursor,
            viewport,
            renderer,
        )
    }

    fn draw(
        &self,
        tree: &Tree,
        renderer: &mut Renderer,
        theme: &Theme,
        style: &renderer::Style,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        viewport: &Rectangle,
    ) {
        self.content.as_widget().draw(
            &tree.children[0],
            renderer,
            theme,
            style,
            layout,
            cursor,
            viewport,
        );
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

impl<'a, Message, Theme, Renderer> From<TwistEditor<'a, Message, Theme, Renderer>>
    for Element<'a, Message, Theme, Renderer>
where
    Message: 'a + Clone,
    Theme: iced::widget::text_editor::Catalog + iced::widget::text::Catalog + 'a,
    Renderer: iced::advanced::Renderer + iced::advanced::text::Renderer + 'a,
{
    fn from(
        viewer: TwistEditor<'a, Message, Theme, Renderer>,
    ) -> Element<'a, Message, Theme, Renderer>
    where
        Message: 'a + Clone,
    {
        Element::new(viewer)
    }
}
