use ::zoomer::zoom_viewer;
use iced::widget::image::Handle;
use iced::widget::{center, column};
use iced::{Center, Element};

#[derive(Debug, Clone, Copy)]
enum Message {}

pub fn main() -> iced::Result {
    iced::run("Zoom Widget - Iced", Example::update, Example::view)
}

struct Example {}

impl Example {
    fn update(&mut self, message: Message) {
        match message {}
    }

    fn view(&self) -> Element<Message> {
        let content = column!(zoom_viewer(Handle::from_path("perspective.jpg")))
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
