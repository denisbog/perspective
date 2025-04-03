use ::image::ImageReader;
use clap::{Parser, command};
use iced::alignment::Horizontal;
use iced::futures::executor::block_on;
use nalgebra::Vector3;
use perspective::AxisData;
use perspective::camera_pose::ComputeCameraPose;
use perspective::draw::DrawLine;
use std::cell::RefCell;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::rc::Rc;
use zoomer::ZoomViewer;

use iced::Alignment::Center;
use iced::Length::Fill;
use iced::widget::{button, center, column, image, row, slider, stack, text};
use iced::{Element, Length, Size, Task, Theme};
use perspective::compute::{
    ComputeSolution, Lines, StoreLine, StorePoint, StorePoint3d, compute_adapter,
    read_points_from_file, store_scene_data_to_file,
};
use tracing::{trace, warn};
use tracing_subscriber::EnvFilter;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    points: Option<String>,
    #[arg(short, long, value_delimiter = ' ', num_args = 1.., default_value = "perspective.jpg")]
    images: Vec<String>,
}

pub fn main() -> iced::Result {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    iced::application("Perspective", Perspective::update, Perspective::view)
        .theme(Perspective::theme)
        .antialiasing(true)
        .centered()
        .run_with(Perspective::new)
}

#[derive(Default, Clone, Debug)]
enum UiMod {
    #[default]
    Pose,
    Draw,
    Scale,
}

#[derive(Debug, Clone)]
enum Message {
    Save,
    CalculateAndSaveToFile,
    LoadApplicationState {
        image_data: Option<ImageData>,
        image_size: Size<u32>,
    },
    SelectImage(u8),
    Flip(bool, bool, bool),
    AddTranslation,
    ResetTranslation,
    ChangeMode(UiMod),
}

#[derive(Default)]
struct Perspective {
    axis_data: Option<Rc<RefCell<AxisData>>>,
    image_path: String,
    points_file_name: String,
    export_file_name: String,
    compute_solution: Option<ComputeSolution>,
    image_width: u32,
    image_height: u32,
    draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
    selected_image: u8,
    images: Vec<String>,
    traslate_origin: Rc<RefCell<Vector3<f32>>>,
    mode: UiMod,
}
#[derive(Debug, Clone)]
struct ImageData {
    axis_data: AxisData,
    lines: Option<Vec<Vector3<f32>>>,
}
async fn load(
    image: String,
    points_file_name: String,
    load_lines: bool,
) -> (Option<ImageData>, Size<u32>) {
    let extracted_data = if Path::new(&points_file_name).exists() {
        let read_from_file = read_points_from_file(&points_file_name);
        let lines = if load_lines { read_from_file.1 } else { None };
        Some(ImageData {
            axis_data: read_from_file.0,
            lines,
        })
    } else {
        warn!("could not read data for {}", points_file_name);
        None
    };

    let decoded_image = ImageReader::open(&image).unwrap().decode().unwrap();
    (
        extracted_data,
        Size::new(decoded_image.width(), decoded_image.height()),
    )
}

fn extract_state(state: (Option<ImageData>, Size<u32>)) -> Message {
    Message::LoadApplicationState {
        image_data: state.0,
        image_size: state.1,
    }
}

impl Perspective {
    fn new() -> (Self, Task<Message>) {
        let args = Args::parse();
        trace!("args {:?}", args);
        let draw_lines = Rc::new(RefCell::new(vec![Vector3::<f32>::zeros()]));
        let first_image = args.images.first().unwrap().clone();
        let image_name = Path::new(&first_image)
            .file_stem()
            .unwrap()
            .to_str()
            .unwrap();
        let points = if args.points.is_none() {
            format!("{}.points", image_name)
        } else {
            args.points.unwrap()
        };
        let export_file_name = format!("{}.fspy", image_name);
        let init = Perspective {
            image_path: first_image.clone(),
            draw_lines,
            images: args.images,
            export_file_name,
            points_file_name: points.clone(),
            ..Self::default()
        };
        (
            init,
            Task::perform(load(first_image, points, true), extract_state),
        )
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::Save => {
                let mut file = File::create(self.points_file_name.clone()).unwrap();
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                let lines = axis_data
                    .borrow()
                    .axis_lines
                    .iter()
                    .map(Into::into)
                    .collect::<Vec<StoreLine>>();

                let custom_origin_tanslation =
                    if let Some(item) = axis_data.borrow().custom_origin_tanslation {
                        Some(StorePoint3d {
                            x: item.x,
                            y: item.y,
                            z: item.z,
                        })
                    } else {
                        None
                    };

                let store = Lines {
                    lines,
                    control_point: StorePoint {
                        x: axis_data.borrow().control_point.x,
                        y: axis_data.borrow().control_point.y,
                    },
                    scale: StoreLine::from(&axis_data.borrow().scale),
                    points: Some(
                        self.draw_lines
                            .borrow()
                            .iter()
                            .map(|item| StorePoint3d {
                                x: item.x,
                                y: item.y,
                                z: item.z,
                            })
                            .collect(),
                    ),
                    flip: Some([
                        axis_data.borrow().flip.0,
                        axis_data.borrow().flip.1,
                        axis_data.borrow().flip.2,
                    ]),
                    custom_origin_tanslation,
                };
                file.write_all(&serde_json::to_vec(&store).unwrap())
                    .unwrap();
            }
            Message::CalculateAndSaveToFile => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                let lines_x = [
                    axis_data.borrow().axis_lines[0],
                    axis_data.borrow().axis_lines[1],
                ];
                let lines_y = [
                    axis_data.borrow().axis_lines[2],
                    axis_data.borrow().axis_lines[3],
                ];
                let lines_z = [
                    axis_data.borrow().axis_lines[4],
                    axis_data.borrow().axis_lines[5],
                ];

                let control_point = &axis_data.borrow().control_point;
                let scale = &axis_data.borrow().scale;
                self.compute_solution = Some(block_on(async {
                    let compute_solution = compute_adapter(
                        lines_x,
                        lines_y,
                        lines_z,
                        self.image_width,
                        self.image_height,
                        control_point,
                        scale,
                        axis_data.borrow().flip,
                        &axis_data.borrow().custom_origin_tanslation,
                    )
                    .await
                    .unwrap();

                    let data = store_scene_data_to_file(
                        &compute_solution,
                        self.image_width,
                        self.image_height,
                        self.image_path.clone(),
                        self.export_file_name.clone(),
                    )
                    .await;
                    trace!("scene data: {:?}", data);
                    compute_solution
                }));
            }
            Message::LoadApplicationState {
                image_data,
                image_size,
            } => {
                self.image_width = image_size.width;
                self.image_height = image_size.height;
                if let Some(image_data) = image_data {
                    self.axis_data = Some(Rc::new(RefCell::new(image_data.axis_data)));
                    if let Some(lines) = image_data.lines {
                        self.traslate_origin = Rc::new(RefCell::new(lines.last().unwrap().clone()));
                        self.draw_lines = Rc::new(RefCell::new(lines));
                    }
                } else {
                    self.axis_data = Some(Rc::new(RefCell::new(AxisData::default())));
                }
            }
            Message::ChangeMode(mode) => {
                self.mode = mode;
            }
            Message::SelectImage(selected) => {
                self.selected_image = selected;
                let selected_image_name = self
                    .images
                    .get(self.selected_image as usize)
                    .unwrap()
                    .clone();
                self.image_path = selected_image_name.clone();
                let name_without_extension = Path::new(&selected_image_name)
                    .file_stem()
                    .unwrap()
                    .to_str()
                    .unwrap();
                self.points_file_name = format!("{}.points", name_without_extension);
                self.export_file_name = format!("{}.fspy", name_without_extension);

                self.update(extract_state(block_on(async {
                    load(selected_image_name, self.points_file_name.clone(), false).await
                })));
                self.update(Message::CalculateAndSaveToFile);
            }
            Message::Flip(flip_x, flip_y, flip_z) => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                axis_data.borrow_mut().flip = (flip_x, flip_y, flip_z);
            }
            Message::AddTranslation => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                axis_data.borrow_mut().custom_origin_tanslation =
                    Some(self.traslate_origin.borrow().clone());
                self.update(Message::CalculateAndSaveToFile);
            }
            Message::ResetTranslation => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                axis_data.borrow_mut().custom_origin_tanslation = None;
                self.update(Message::CalculateAndSaveToFile);
            }
        }
    }
    fn view(&self) -> Element<Message> {
        if self.axis_data.is_none() {
            return center(text("Loading...").width(Fill).align_x(Center).size(50)).into();
        };
        let component: Element<Message> = if let UiMod::Draw = self.mode {
            DrawLine::new(
                &self.compute_solution,
                Rc::clone(&self.draw_lines),
                self.traslate_origin.clone(),
            )
            .image_size(Size::new(self.image_width as f32, self.image_height as f32))
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
        } else {
            ComputeCameraPose::new(
                self.axis_data.as_ref().unwrap().clone(),
                &self.compute_solution,
            )
            .image_size(Size::new(self.image_width as f32, self.image_height as f32))
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
        };

        let mut buttons = Vec::new();
        match self.mode {
            UiMod::Draw => {
                buttons.push(
                    button("Pose")
                        .on_press(Message::ChangeMode(UiMod::Pose))
                        .into(),
                );
                buttons.push(
                    button("Apply Translation")
                        .on_press(Message::AddTranslation)
                        .into(),
                );
                buttons.push(
                    button("Reset Translation")
                        .on_press(Message::ResetTranslation)
                        .into(),
                );
                buttons.push(button("Save lines").on_press(Message::Save).into());
            }
            UiMod::Pose => {
                buttons.push(
                    button("Draw lines")
                        .on_press(Message::ChangeMode(UiMod::Draw))
                        .into(),
                );
                buttons.push(
                    button("Perform calculations")
                        .on_press(Message::CalculateAndSaveToFile)
                        .into(),
                );
                buttons.push(
                    button("Flip X")
                        .on_press(Message::Flip(
                            !self.axis_data.as_ref().unwrap().borrow().flip.0,
                            self.axis_data.as_ref().unwrap().borrow().flip.1,
                            self.axis_data.as_ref().unwrap().borrow().flip.2,
                        ))
                        .into(),
                );
                buttons.push(
                    button("Flip Y")
                        .on_press(Message::Flip(
                            self.axis_data.as_ref().unwrap().borrow().flip.0,
                            !self.axis_data.as_ref().unwrap().borrow().flip.1,
                            self.axis_data.as_ref().unwrap().borrow().flip.2,
                        ))
                        .into(),
                );
                buttons.push(
                    button("Flip Z")
                        .on_press(Message::Flip(
                            self.axis_data.as_ref().unwrap().borrow().flip.0,
                            self.axis_data.as_ref().unwrap().borrow().flip.1,
                            !self.axis_data.as_ref().unwrap().borrow().flip.2,
                        ))
                        .into(),
                );
            }
            UiMod::Scale => todo!(),
        }

        column!(
            row!(
                column!(stack!(
                    ZoomViewer::new(self.images.get(self.selected_image as usize).unwrap())
                        .scale(2.0),
                    component,
                ),)
                .width(Length::Fill),
                column!(
                    slider(
                        0u8..=(self.images.len() - 1) as u8,
                        self.selected_image,
                        Message::SelectImage
                    )
                    .width(280),
                    column(self.images.iter().enumerate().map(|(index, item)| {
                        button(
                            image(item)
                                .content_fit(iced::ContentFit::Cover)
                                .width(280)
                                .height(200),
                        )
                        .on_press_with(move || Message::SelectImage(index as u8))
                        .into()
                    }))
                    .spacing(10)
                )
                .width(300)
                .spacing(10)
                .align_x(Horizontal::Right),
            )
            .height(Length::Fill)
            .padding(20),
            row(buttons).width(Length::Fill).padding(10).spacing(5)
        )
        .into()
    }
    fn theme(&self) -> Theme {
        Theme::TokyoNight
    }
}

#[cfg(test)]
mod tests {
    use anyhow::Result;
    use nalgebra::{Matrix3, RowVector3, Vector2};
    use perspective::compute::{compute_camera_pose, store_scene_data_to_file};
    use tracing::trace;
    use tracing_subscriber::EnvFilter;

    #[tokio::test]
    async fn compute_test_new() -> Result<()> {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .init();
        let points = [
            Vector2::new(0.6746836, 0.5918425),
            Vector2::new(0.8013924, 0.5004782),
            Vector2::new(0.50898737, 0.11926863),
            Vector2::new(0.64367086, 0.078312226),
            Vector2::new(0.6596202, 0.5918425),
            Vector2::new(0.52405065, 0.5130802),
            Vector2::new(0.65607595, 0.08146272),
            Vector2::new(0.7748101, 0.11139241),
            Vector2::new(0.66759497, 0.5571871),
            Vector2::new(0.67556965, 0.19330521),
            Vector2::new(0.5001266, 0.365007),
            Vector2::new(0.5001266, 0.13344586),
        ];
        let image_width = 1920.0;
        let image_height = 1080.0;
        let ratio = image_width / image_height;

        let user_selected_origin = Vector2::new(0.66607594, 0.5972433);

        let handle_position = [
            Vector2::new(0.666962, 0.5956681),
            Vector2::new(0.80341774, 0.49957806),
        ];

        let axis = Matrix3::from_rows(&[
            RowVector3::new(1.0, 0.0, 0.0),
            RowVector3::new(0.0, -1.0, 0.0),
            RowVector3::new(0.0, 0.0, -1.0),
        ]);

        let compute_solution = compute_camera_pose(
            &points,
            ratio,
            &user_selected_origin,
            &Some(handle_position),
            axis,
            &None,
        )
        .await
        .unwrap();

        trace!("out {:#?}", compute_solution.view_transform);

        store_scene_data_to_file(
            &compute_solution,
            image_width as u32,
            image_height as u32,
            "newperspective.jpg".into(),
            "newperspective.jpg.test.fspy".into(),
        )
        .await
        .unwrap();
        Ok(())
    }
}
