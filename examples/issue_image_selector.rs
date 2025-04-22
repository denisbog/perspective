use iced::{
    Element,
    widget::{column, image, scrollable},
};
use tracing::trace;
use tracing_subscriber::EnvFilter;

pub fn main() -> iced::Result {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    iced::application(
        ImageSelector::default,
        ImageSelector::update,
        ImageSelector::view,
    )
    .antialiasing(true)
    .centered()
    .run()
}

struct ImageSelector {
    images: Vec<String>,
}

impl Default for ImageSelector {
    fn default() -> Self {
        Self {
            images: (0..10).map(|_image| format!("perspective.jpg")).collect(),
        }
    }
}

#[derive(Debug, Clone)]
enum Message {}
impl ImageSelector {
    fn update(&mut self, _message: Message) {}
    fn view(&self) -> Element<Message> {
        trace!("images {:?}", self.images);
        scrollable(column(self.images.iter().map(|item| {
            image(item)
                .content_fit(iced::ContentFit::Cover)
                .width(280)
                .height(200)
                .into()
        })))
        .into()
    }
}
