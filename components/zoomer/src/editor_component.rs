use iced::{
    Background, Border, Element, Theme,
    alignment::Vertical,
    widget::{column, row, text_input},
};
use nalgebra::Point3;

#[derive(Default)]
pub struct EditorComponent {
    label: &'static str,
    value_x: String,
    value_y: String,
    value_z: String,
}

#[derive(Debug, Clone)]
pub enum Message {
    InternalEdit(usize, String),
}

#[derive(Debug, Clone)]
pub enum Action {
    Valid(Point3<f32>),
    Invalid,
}
impl<'a> EditorComponent {
    pub fn new(label: &'static str, twist_point: &Point3<f32>) -> Self {
        Self {
            label,
            value_x: EditorComponent::edit_string(twist_point.x),
            value_y: EditorComponent::edit_string(twist_point.y),
            value_z: EditorComponent::edit_string(twist_point.z),
        }
    }

    #[must_use]
    pub fn update(&mut self, message: Message) -> Action {
        match message {
            Message::InternalEdit(component, input) => self.handle_update(component, input),
        }
    }

    fn handle_update(&mut self, component: usize, input: String) -> Action {
        match component {
            1 => {
                self.value_x = input.clone();
            }
            2 => {
                self.value_y = input.clone();
            }
            3 => {
                self.value_z = input.clone();
            }
            _ => {}
        };
        if let Ok(new) = input.parse::<f32>()
            && self.value_x.parse::<f32>().is_ok()
            && self.value_y.parse::<f32>().is_ok()
            && self.value_z.parse::<f32>().is_ok()
        {
            match component {
                1 => Action::Valid(Point3::new(
                    new,
                    self.value_y.parse().unwrap(),
                    self.value_z.parse().unwrap(),
                )),
                2 => Action::Valid(Point3::new(
                    self.value_x.parse().unwrap(),
                    new,
                    self.value_z.parse().unwrap(),
                )),
                3 => Action::Valid(Point3::new(
                    self.value_x.parse().unwrap(),
                    self.value_y.parse().unwrap(),
                    new,
                )),
                _ => Action::Invalid,
            }
        } else {
            Action::Invalid
        }
    }

    pub fn view<M>(&'a self, on_edit: &'a (impl Fn(Message) -> M + 'a)) -> Element<'a, M>
    where
        M: Clone + 'a,
    {
        column!(
            row!(
                self.label,
                text_input("x", &self.value_x)
                    .on_input(|input| on_edit(Message::InternalEdit(1, input)))
                    .style(|theme, status| EditorComponent::get_style(
                        &self.value_x,
                        theme,
                        status
                    )),
                text_input("y", &self.value_y)
                    .on_input(|input| on_edit(Message::InternalEdit(2, input)))
                    .style(|theme, status| EditorComponent::get_style(
                        &self.value_y,
                        theme,
                        status
                    )),
                text_input("z", &self.value_z)
                    .on_input(|input| on_edit(Message::InternalEdit(3, input)))
                    .style(|theme, status| EditorComponent::get_style(
                        &self.value_z,
                        theme,
                        status
                    )),
            )
            .align_y(Vertical::Center)
            .padding(5.0)
            .spacing(5.0),
        )
        .into()
    }

    fn edit_string(value: f32) -> String {
        if value == 0.0 {
            "0.0".to_string()
        } else {
            format!("{value:.2}")
        }
    }

    fn get_style(input: &str, theme: &Theme, _status: text_input::Status) -> text_input::Style {
        let palette = theme.extended_palette();
        let border_color = if input.parse::<f32>().is_ok() {
            palette.background.strong.color
        } else {
            palette.danger.strong.color
        };
        text_input::Style {
            background: Background::Color(palette.background.base.color),
            border: Border {
                radius: 2.0.into(),
                width: 1.0,
                color: border_color,
            },
            icon: palette.background.weak.text,
            placeholder: palette.secondary.base.color,
            value: palette.background.base.text,
            selection: palette.primary.weak.color,
        }
    }
}
