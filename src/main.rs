use ::image::ImageReader;
use clap::{Parser, command};
use iced::Alignment::Center;
use iced::Length::Fill;
use iced::alignment::{Horizontal, Vertical};
use iced::futures::executor::block_on;
use iced::widget::scrollable::{Direction, Scrollbar};
use iced::widget::{
    center, column, container, image, mouse_area, row, scrollable, slider, stack, text,
};
use iced::{Element, Length, Point, Size, Task, Theme};
use nalgebra::{Vector2, Vector3};
use perspective::camera_pose::ComputeCameraPose;
use perspective::compute::{
    ComputeSolution, Lines, StoreLine, StorePoint, StorePoint3d, compute_camera_pose_scale,
    compute_ui_adapter, read_points_from_file, store_scene_data_to_file,
};
use perspective::draw::DrawLine;
use perspective::optimize::{ortho_center_optimize, pose_optimize};
use perspective::{AxisData, PointInformation};
use std::cell::RefCell;
use std::fmt::Debug;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::rc::Rc;
use tracing::{trace, warn};
use tracing_subscriber::EnvFilter;
use zoomer::context_menu::ContextMenu;

use anyhow::Result;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    points: Option<String>,
    #[arg(short, long)]
    dimension: Option<f32>,
    #[arg(short, long, value_delimiter = ' ', num_args = 1.., default_value = "perspective.jpg")]
    images: Vec<String>,
}

pub fn main() -> iced::Result {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    iced::application(Perspective::new, Perspective::update, Perspective::view)
        .theme(Perspective::theme)
        .antialiasing(true)
        .centered()
        .run()
}

#[derive(Default, Clone, Debug)]
enum UiMod {
    Pose,
    #[default]
    Draw,
    Scale,
    Try,
}

#[derive(Debug, Clone)]
enum Message {
    Save,
    LoadLines,
    CalculatePose,
    LoadApplicationState {
        image_data: Option<ImageData>,
        image_size: Size<u32>,
    },
    SelectImage(u8),
    Flip(bool, bool, bool),
    ApplyScale,
    ResetScale,
    ApplyTranslation,
    ResetTranslation,
    ChangeMode(UiMod),
    ExportToFSpy,
    Optimize,
    OptimizeForError,
    ZoomChanged(f32),
    ScaleToDimension,
}

#[derive(Default)]
struct Perspective {
    axis_data: Option<Rc<RefCell<AxisData>>>,
    image_path: String,
    points_file_name: String,
    export_file_name: String,
    compute_solution: Option<ComputeSolution<f32>>,
    image_size: Size<f32>,
    draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
    selected_image: u8,
    images: Vec<String>,
    mode: UiMod,
    custom_origin_translation: Rc<RefCell<Option<Vector3<f32>>>>,
    custom_scale_segment: Rc<RefCell<Option<usize>>>,
    custom_scale: Rc<RefCell<Option<PointInformation<f32>>>>,
    custom_error: Rc<RefCell<Option<PointInformation<f32>>>>,
    zoom: f32,
    dimension: Option<f32>,
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
) -> Result<(Option<ImageData>, Size<u32>)> {
    let extracted_data = if Path::new(&points_file_name).exists() {
        let read_from_file = read_points_from_file(&points_file_name)?;
        let lines = if load_lines { read_from_file.1 } else { None };
        Some(ImageData {
            axis_data: read_from_file.0,
            lines,
        })
    } else {
        warn!("could not read data for {}", points_file_name);
        None
    };

    let decoded_image = ImageReader::open(&image)?.decode()?;
    Ok((
        extracted_data,
        Size::new(decoded_image.width(), decoded_image.height()),
    ))
}

fn extract_state(state: Result<(Option<ImageData>, Size<u32>)>) -> Message {
    let state = state.unwrap();
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
        let dimension = args.dimension;
        let init = Perspective {
            image_path: first_image.clone(),
            draw_lines,
            images: args.images,
            export_file_name,
            points_file_name: points.clone(),
            zoom: 0.5,
            dimension,
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
                if self.axis_data.is_none() {
                    return;
                };
                let out = <Lines as From<&Perspective>>::from(self);
                file.write_all(&serde_json::to_vec(&out).unwrap()).unwrap();
            }
            Message::LoadLines => {
                if self.axis_data.is_none() {
                    return;
                };
                if let Ok((_, Some(lines))) = read_points_from_file(&self.points_file_name.clone())
                {
                    *self.draw_lines.borrow_mut() = lines;
                };
            }
            Message::CalculatePose => {
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
                self.compute_solution = Some(
                    compute_ui_adapter(
                        lines_x,
                        lines_y,
                        lines_z,
                        self.image_size,
                        control_point,
                        axis_data.borrow().flip,
                        &axis_data.borrow().custom_origin_translation,
                        &axis_data.borrow().custom_scale,
                    )
                    .unwrap(),
                );
            }
            Message::ScaleToDimension => {
                if self.compute_solution.is_some() {
                    let Some(custom_scale) = self.custom_scale.borrow().clone() else {
                        return;
                    };
                    let solution = self.compute_solution.clone().unwrap();
                    self.compute_solution = if let Some(scale) = self.dimension {
                        let scale =
                            (custom_scale.source_vector - custom_scale.vector).norm() / scale;
                        self.axis_data
                            .as_mut()
                            .unwrap()
                            .borrow_mut()
                            .custom_scale
                            .replace(scale);
                        compute_camera_pose_scale(solution, scale).ok()
                    } else {
                        Some(solution)
                    };
                };
            }
            Message::LoadApplicationState {
                image_data,
                image_size,
            } => {
                self.image_size = Size::new(image_size.width as f32, image_size.height as f32);
                if let Some(image_data) = image_data {
                    self.axis_data = Some(Rc::new(RefCell::new(image_data.axis_data)));
                    if let Some(lines) = image_data.lines {
                        self.draw_lines = Rc::new(RefCell::new(lines));
                    }
                } else {
                    self.axis_data = Some(Rc::new(RefCell::new(AxisData::default())));
                }
                self.update(Message::CalculatePose);
            }
            Message::ChangeMode(mode) => {
                self.mode = mode;
            }
            Message::SelectImage(selected) => {
                self.update(Message::Save);
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
                self.update(Message::CalculatePose);
            }
            Message::Flip(flip_x, flip_y, flip_z) => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                axis_data.borrow_mut().flip = (flip_x, flip_y, flip_z);
                self.update(Message::CalculatePose);
            }
            Message::ApplyTranslation => {
                let Some(custom_origin_translation) = *self.custom_origin_translation.borrow()
                else {
                    return;
                };
                self.axis_data
                    .as_ref()
                    .unwrap()
                    .borrow_mut()
                    .custom_origin_translation = Some(custom_origin_translation);
                self.update(Message::CalculatePose);
            }
            Message::ResetTranslation => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                axis_data.borrow_mut().custom_origin_translation = None;
                self.update(Message::CalculatePose);
            }
            Message::ApplyScale => {
                let Some(custom_scale) = self.custom_scale.borrow().clone() else {
                    return;
                };
                let custom_scale = custom_scale.vector - custom_scale.source_vector;

                let scale = if let Some(custom_scale_segment) = *self.custom_scale_segment.borrow()
                {
                    let start = *self.draw_lines.borrow().get(custom_scale_segment).unwrap();
                    let end = *self
                        .draw_lines
                        .borrow()
                        .get(custom_scale_segment + 1)
                        .unwrap();
                    let length = start - end;
                    length.norm()
                } else {
                    1.0
                };
                let scale = custom_scale.norm() / scale;
                let scale = if let Some(prev_scale) =
                    self.axis_data.as_ref().unwrap().borrow().custom_scale
                {
                    prev_scale * scale
                } else {
                    scale
                };
                self.axis_data.as_ref().unwrap().borrow_mut().custom_scale = Some(scale);
                self.custom_scale.replace(None);
                self.custom_scale_segment.replace(None);
                self.update(Message::CalculatePose);
            }
            Message::ResetScale => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                axis_data.borrow_mut().custom_scale = None;
                self.update(Message::CalculatePose);
            }
            Message::ExportToFSpy => {
                let Some(compute_solution) = &self.compute_solution else {
                    return;
                };
                block_on(async {
                    let data = store_scene_data_to_file(
                        compute_solution,
                        self.image_size.width as u32,
                        self.image_size.height as u32,
                        self.image_path.clone(),
                        self.export_file_name.clone(),
                    )
                    .await;
                    trace!("scene data: {:?}", data);
                });
            }
            Message::Optimize => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                let lines = axis_data
                    .borrow()
                    .axis_lines
                    .iter()
                    .cloned()
                    .flat_map(|(a, b)| [Vector2::new(a.x, a.y), Vector2::new(b.x, b.y)])
                    .collect();
                if let Ok(lines) =
                    ortho_center_optimize(self.image_size.width / self.image_size.height, lines)
                {
                    axis_data.borrow_mut().axis_lines = lines
                        .chunks(2)
                        .map(|items| {
                            (
                                Point::new(items[0].x, items[0].y),
                                Point::new(items[1].x, items[1].y),
                            )
                        })
                        .collect();
                    self.update(Message::CalculatePose);
                };
            }
            Message::OptimizeForError => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                let ratio = self.image_size.width / self.image_size.height;
                let axis_lines = axis_data
                    .borrow()
                    .axis_lines
                    .iter()
                    .cloned()
                    .flat_map(|(a, b)| [Vector2::new(a.x, a.y), Vector2::new(b.x, b.y)])
                    .collect();

                let control_point = Vector2::new(
                    axis_data.borrow().control_point.x,
                    axis_data.borrow().control_point.y,
                );

                let flip = axis_data.borrow().flip;
                let custom_translation = axis_data
                    .borrow()
                    .custom_origin_translation
                    .unwrap_or_default();
                //let draw_lines = self.draw_lines.borrow().to_vec();
                let scale = axis_data.borrow().custom_scale.unwrap_or(1.0) as f64;
                if let Ok(lines) = pose_optimize(
                    ratio,
                    axis_lines,
                    //draw_lines,
                    control_point,
                    flip,
                    custom_translation,
                    //*self.custom_scale_segment.borrow(),
                    //self.custom_scale.borrow().clone(),
                    self.custom_error.borrow().clone(),
                    scale,
                ) {
                    axis_data.borrow_mut().axis_lines = lines
                        .chunks(2)
                        .map(|items| {
                            (
                                Point::new(items[0].x, items[0].y),
                                Point::new(items[1].x, items[1].y),
                            )
                        })
                        .collect();
                };
                self.update(Message::CalculatePose);
            }
            Message::ZoomChanged(zoom) => self.zoom = zoom,
        }
    }
    fn view(&self) -> Element<Message> {
        let Some(axis_data) = &self.axis_data else {
            return center(text("Loading...").width(Fill).align_x(Center).size(50)).into();
        };
        let component: Element<Message> = match self.mode {
            UiMod::Pose => ComputeCameraPose::new(Rc::clone(axis_data), &self.compute_solution)
                .image_size(self.image_size)
                .width(Length::Fill)
                .height(Length::Fill)
                .into(),
            UiMod::Scale => DrawLine::new(
                &self.compute_solution,
                Rc::clone(&self.draw_lines),
                Rc::clone(&self.custom_origin_translation),
                Rc::clone(&self.custom_scale_segment),
                Rc::clone(&self.custom_scale),
                Rc::clone(&self.custom_error),
            )
            .image_size(self.image_size)
            .width(Length::Fill)
            .height(Length::Fill)
            .into(),
            UiMod::Draw => DrawLine::new(
                &self.compute_solution,
                Rc::clone(&self.draw_lines),
                Rc::clone(&self.custom_origin_translation),
                Rc::clone(&self.custom_scale_segment),
                Rc::clone(&self.custom_scale),
                Rc::clone(&self.custom_error),
            )
            .image_size(self.image_size)
            .width(Length::Fill)
            .height(Length::Fill)
            .into(),
            UiMod::Try => DrawLine::new(
                &self.compute_solution,
                Rc::new(RefCell::new(vec![
                    self.custom_origin_translation.borrow().unwrap_or_default(),
                ])),
                Rc::clone(&self.custom_origin_translation),
                Rc::clone(&self.custom_scale_segment),
                Rc::clone(&self.custom_scale),
                Rc::clone(&self.custom_error),
            )
            .image_size(self.image_size)
            .width(Length::Fill)
            .height(Length::Fill)
            .into(),
        };

        let canvas = scrollable(stack!(
            image(self.images.get(self.selected_image as usize).unwrap())
                .width(self.image_size.width * self.zoom)
                .height(self.image_size.height * self.zoom),
            component,
        ))
        .direction(Direction::Both {
            vertical: Scrollbar::default(),
            horizontal: Scrollbar::default(),
        });

        let canvas_with_context_menu = ContextMenu::new(canvas, move || {
            let mut buttons = Vec::new();
            match self.mode {
                UiMod::Pose => {
                    buttons.push(
                        mouse_area(container("Try").width(Length::Fill))
                            .on_press(Message::ChangeMode(UiMod::Try))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Draw lines").width(Length::Fill))
                            .on_press(Message::ChangeMode(UiMod::Draw))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Scale/Translation").width(Length::Fill))
                            .on_press(Message::ChangeMode(UiMod::Scale))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Perform calculations").width(Length::Fill))
                            .on_press(Message::CalculatePose)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Flip X").width(Length::Fill))
                            .on_press(Message::Flip(
                                !axis_data.as_ref().borrow().flip.0,
                                axis_data.as_ref().borrow().flip.1,
                                axis_data.as_ref().borrow().flip.2,
                            ))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Flip Y").width(Length::Fill))
                            .on_press(Message::Flip(
                                axis_data.as_ref().borrow().flip.0,
                                !axis_data.as_ref().borrow().flip.1,
                                axis_data.as_ref().borrow().flip.2,
                            ))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Flip Z").width(Length::Fill))
                            .on_press(Message::Flip(
                                axis_data.as_ref().borrow().flip.0,
                                axis_data.as_ref().borrow().flip.1,
                                !axis_data.as_ref().borrow().flip.2,
                            ))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Export Pose To FSpy").width(Length::Fill))
                            .on_press(Message::ExportToFSpy)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Save lines").width(Length::Fill))
                            .on_press(Message::Save)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Optimize").width(Length::Fill))
                            .on_press(Message::Optimize)
                            .into(),
                    );
                }
                UiMod::Scale => {
                    buttons.push(
                        mouse_area(container("Pose").width(Length::Fill))
                            .on_press(Message::ChangeMode(UiMod::Pose))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Draw lines").width(Length::Fill))
                            .on_press(Message::ChangeMode(UiMod::Draw))
                            .into(),
                    );
                    //if self.custom_scale.borrow().is_some() {
                    buttons.push(
                        mouse_area(container("Apply Scale").width(Length::Fill))
                            .on_press(Message::ApplyScale)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Apply Scale to Dimension").width(Length::Fill))
                            .on_press(Message::ScaleToDimension)
                            .into(),
                    );
                    //} else {
                    buttons.push(
                        mouse_area(container("Reset Scale").width(Length::Fill))
                            .on_press(Message::ResetScale)
                            .into(),
                    );
                    //}

                    //if self.custom_origin_translation.borrow().is_some() {
                    buttons.push(
                        mouse_area(container("Apply Translation").width(Length::Fill))
                            .on_press(Message::ApplyTranslation)
                            .into(),
                    );
                    //} else {
                    buttons.push(
                        mouse_area(container("Reset Translation").width(Length::Fill))
                            .on_press(Message::ResetTranslation)
                            .into(),
                    );
                    //}
                    buttons.push(
                        mouse_area(container("Export Pose To FSpy").width(Length::Fill))
                            .on_press(Message::ExportToFSpy)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Save lines").width(Length::Fill))
                            .on_press(Message::Save)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Load lines").width(Length::Fill))
                            .on_press(Message::LoadLines)
                            .into(),
                    );
                }
                UiMod::Draw => {
                    buttons.push(
                        mouse_area(container("Pose").width(Length::Fill))
                            .on_press(Message::ChangeMode(UiMod::Pose))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Scale/Translation").width(Length::Fill))
                            .on_press(Message::ChangeMode(UiMod::Scale))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Save lines").width(Length::Fill))
                            .on_press(Message::Save)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Load lines").width(Length::Fill))
                            .on_press(Message::LoadLines)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Optimize Error").width(Length::Fill))
                            .on_press(Message::OptimizeForError)
                            .into(),
                    );
                }
                UiMod::Try => {
                    buttons.push(
                        mouse_area(container("Pose").width(Length::Fill))
                            .on_press(Message::ChangeMode(UiMod::Pose))
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Scale/Translation").width(Length::Fill))
                            .on_press(Message::ChangeMode(UiMod::Scale))
                            .into(),
                    );
                }
            }
            column(buttons).width(300).padding(5).spacing(7).into()
        });

        column!(
            row!(
                container(canvas_with_context_menu)
                    .width(Length::Fill)
                    .height(Length::Fill)
                    .align_x(Horizontal::Center)
                    .align_y(Vertical::Center),
                column!(
                    container(slider(0.25f32..=1.0f32, self.zoom, Message::ZoomChanged).step(0.05))
                        .padding(20),
                    scrollable(
                        column(self.images.iter().enumerate().map(|(index, item)| {
                            let opacity = if index as u8 == self.selected_image {
                                1.0
                            } else {
                                0.4
                            };
                            mouse_area(
                                image(item)
                                    .content_fit(iced::ContentFit::Cover)
                                    .width(280)
                                    .height(200)
                                    .opacity(opacity),
                            )
                            .on_press(Message::SelectImage(index as u8))
                            .into()
                        }))
                        .spacing(20)
                        .padding(20)
                    )
                )
                .width(300)
            )
            .height(Length::Fill)
            .padding(10),
        )
        .into()
    }
    fn theme(&self) -> Theme {
        Theme::TokyoNight
    }
}

impl From<&Perspective> for Lines {
    fn from(value: &Perspective) -> Self {
        let axis_data = value.axis_data.as_ref().unwrap();
        let lines = axis_data
            .borrow()
            .axis_lines
            .iter()
            .map(Into::into)
            .collect::<Vec<StoreLine>>();

        let custom_origin_tanslation =
            axis_data
                .borrow()
                .custom_origin_translation
                .map(|item| StorePoint3d {
                    x: item.x,
                    y: item.y,
                    z: item.z,
                });

        let custom_scale = axis_data.borrow().custom_scale;
        Lines {
            lines,
            control_point: StorePoint {
                x: axis_data.borrow().control_point.x,
                y: axis_data.borrow().control_point.y,
            },
            points: Some(
                value
                    .draw_lines
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
            custom_scale,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use anyhow::Result;
    use nalgebra::{Matrix3, Perspective3, Point3, RowVector3, Vector2};
    use perspective::{
        compute::{
            compute_camera_pose, compute_camera_pose_scale, find_vanishing_point_for_lines,
            store_scene_data_to_file,
        },
        optimize::ortho_center_optimize,
        utils::relative_to_image_plane,
    };
    use tracing::trace;
    use tracing_subscriber::EnvFilter;

    #[tokio::test]
    async fn compute_test_new() -> Result<()> {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .init();
        let points = vec![
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

        let axis = Matrix3::from_rows(&[
            RowVector3::new(1.0, 0.0, 0.0),
            RowVector3::new(0.0, -1.0, 0.0),
            RowVector3::new(0.0, 0.0, -1.0),
        ]);

        let user_selected_origin = relative_to_image_plane(ratio, &user_selected_origin);

        let vanishing_points = points
            .chunks(4)
            .map(|lines| find_vanishing_point_for_lines(&lines[0], &lines[1], &lines[2], &lines[3]))
            .collect::<Vec<Vector2<f32>>>();

        let vanishing_points = vanishing_points
            .iter()
            .map(|point| relative_to_image_plane(ratio, point))
            .collect::<Vec<Vector2<f32>>>();

        let compute_solution =
            compute_camera_pose(&vanishing_points, &user_selected_origin, axis).unwrap();

        let compute_solution = compute_camera_pose_scale(compute_solution, 1.75).unwrap();

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
    #[tokio::test]
    async fn optimize() -> Result<()> {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .init();

        let points = vec![
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
        let points = ortho_center_optimize(ratio, points);
        trace!("solution: {:?}", points);
        Ok(())
    }
    #[tokio::test]
    async fn space_convertion() -> Result<()> {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .init();
        let point = relative_to_image_plane(1.33, &Vector2::new(0.0, 0.0));
        trace!("point {point}");
        let point = relative_to_image_plane(1.33, &Vector2::new(0.5, 0.5));
        trace!("point {point}");
        let point = relative_to_image_plane(1.33, &Vector2::new(1.0, 1.0));
        trace!("point {point}");
        Ok(())
    }
    #[tokio::test]
    async fn matrix_test() -> Result<()> {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .init();

        let perspective = Perspective3::new(1.0, PI / 2.0, 0.01, 10000.0);
        let point = Point3::new(100.0, 100.0, 100.0);
        let point = perspective.into_inner() * point.to_homogeneous();
        let point = Point3::from_homogeneous(point);
        trace!("point {:?}", point);
        let perspective = Perspective3::new(1.0f64, (PI as f64) / 2.0f64, 0.01f64, 10000.0f64);
        let point = Point3::new(100.0f64, 100.0f64, 100.0f64);
        let point = perspective.into_inner() * point.to_homogeneous();
        let point = Point3::from_homogeneous(point);
        trace!("point {:?}", point);
        Ok(())
    }
}
