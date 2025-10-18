use std::{cell::RefCell, rc::Rc};

use iced::{
    Element,
    alignment::Vertical,
    widget::{
        column, row, text,
        text_editor::{self, Action},
        text_input,
    },
};
use nalgebra::Point3;

#[derive(Default)]
pub struct EditorComponent {
    pub twist_points: Rc<RefCell<Vec<Point3<f32>>>>,
}

#[derive(Debug)]
pub enum EditComponentMessage {
    Edit1(Point3<f32>),
    Edit2(Point3<f32>),
    Edit3(Point3<f32>),
    None,
}

impl<'a> EditorComponent {
    pub fn new(twist_points: Rc<RefCell<Vec<Point3<f32>>>>) -> Self {
        Self { twist_points }
    }
    pub fn update<Message>(&mut self) -> Option<Message> {
        None
    }

    fn handle_update<Message>(
        &self,
        index: usize,
        component: usize,
        input: String,
        on_edit: &'a (impl Fn(EditComponentMessage) -> Message + 'a),
    ) -> Message {
        let input = if input.len() == 0 {
            "0".to_string()
        } else {
            input
        };
        let input = if input.chars().last().unwrap() == '.' {
            format!("{input}.0")
        } else {
            input
        };
        if let Ok(new) = input.parse::<f32>() {
            match component {
                1 => {
                    self.twist_points.borrow_mut()[index].x = new;
                    on_edit(EditComponentMessage::Edit1(
                        self.twist_points.borrow()[index],
                    ))
                }
                2 => {
                    self.twist_points.borrow_mut()[index].y = new;
                    on_edit(EditComponentMessage::Edit2(
                        self.twist_points.borrow()[index],
                    ))
                }
                3 => {
                    self.twist_points.borrow_mut()[index].z = new;
                    on_edit(EditComponentMessage::Edit3(
                        self.twist_points.borrow()[index],
                    ))
                }
                _ => on_edit(EditComponentMessage::None),
            }
        } else {
            on_edit(EditComponentMessage::None)
        }
    }

    pub fn view<Message>(
        &'a self,
        on_edit: &'a (impl Fn(EditComponentMessage) -> Message + 'a),
    ) -> Element<'a, Message>
    where
        Message: Clone + 'a,
    {
        println!("rerender");

        column!(
            row!(
                text!("Point #1"),
                text_input("x", &self.twist_points.borrow()[0].x.to_string())
                    .on_input(|action| { self.handle_update(0, 1, action, on_edit) }),
                text_input("y", &self.twist_points.borrow()[0].y.to_string())
                    .on_input(|action| { self.handle_update(0, 2, action, on_edit) }),
                text_input("z", &self.twist_points.borrow()[0].z.to_string())
                    .on_input(|action| { self.handle_update(0, 3, action, on_edit) }),
            )
            .align_y(Vertical::Center)
            .padding(5.0)
            .spacing(5.0),
            row!(
                text!("Point #2"),
                text_input("x", &self.twist_points.borrow()[1].x.to_string())
                    .on_input(|action| { self.handle_update(1, 1, action, on_edit) }),
                text_input("y", &self.twist_points.borrow()[1].y.to_string())
                    .on_input(|action| { self.handle_update(1, 2, action, on_edit) }),
                text_input("z", &self.twist_points.borrow()[1].z.to_string())
                    .on_input(|action| { self.handle_update(1, 3, action, on_edit) }),
            )
            .align_y(Vertical::Center)
            .padding(5.0)
            .spacing(5.0),
            row!(
                text!("Point #3"),
                text_input("x", &self.twist_points.borrow()[2].x.to_string())
                    .on_input(|action| { self.handle_update(2, 1, action, on_edit) }),
                text_input("y", &self.twist_points.borrow()[2].y.to_string())
                    .on_input(|action| { self.handle_update(2, 2, action, on_edit) }),
                text_input("z", &self.twist_points.borrow()[2].z.to_string())
                    .on_input(|action| { self.handle_update(2, 3, action, on_edit) }),
            )
            .align_y(Vertical::Center)
            .padding(5.0)
            .spacing(5.0),
        )
        .into()
    }
}
