pub(crate) use iced::widget::image::Handle;
use iced::widget::{center, column};
use iced::{Center, Element};
use zoomer::zoom_viewer::zoomer;
#[derive(Debug, Clone, Copy)]
enum Message {}

pub fn main() -> iced::Result {
    iced::run(Example::update, Example::view)
}

struct Example {}

impl Example {
    fn update(&mut self, message: Message) {
        match message {}
    }

    fn view<'a>(&'a self) -> Element<'a, Message> {
        let content = column!(zoomer(Handle::from_path("perspective.jpg")))
            .padding(20)
            .spacing(20)
            .max_width(500)
            .align_x(Center);

        center(content).into()
    }
}

impl Default for Example {
    fn default() -> Self {
        Example {}
    }
}
