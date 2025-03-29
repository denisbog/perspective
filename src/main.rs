use clap::{Parser, command};
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
use iced::widget::{Image, button, center, column, row, slider, stack, text};
use iced::{Element, Length, Point, Size, Task, Theme};
use perspective::compute::{
    ComputeSolution, Lines, StoreLine, StorePoint, StorePoint3d, compute_adapter,
    read_points_from_file, store_scene_data_to_file,
};
use tracing::trace;
use tracing_subscriber::EnvFilter;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    points: Option<String>,
    #[arg(short, long, value_delimiter = ' ', num_args = 1..,default_value = "perspective.jpg")]
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
#[derive(Debug, Clone)]
enum Message {
    Draw,
    Save,
    CalculateAndSaveToFile,
    LoadApplicationState {
        control_point: Option<Point>,
        scale: Option<(Point, Point)>,
        lines: Option<Vec<(Point, Point)>>,
        export_file_name: String,
        points_file_name: String,
        image_size: (u32, u32),
        points: Option<Vec<Vector3<f32>>>,
        images: Option<Vec<String>>,
    },
    SelectImage(u8),
}

#[derive(Default)]
struct Perspective {
    axis_data: Rc<RefCell<AxisData>>,
    image_path: String,
    points_file_name: String,
    export_file_name: String,
    compute_solution: Option<ComputeSolution>,
    image_width: u32,
    image_height: u32,
    draw: bool,
    draw_lines: Rc<RefCell<Vec<Vector3<f32>>>>,
    selected_image: u8,
    images: Option<Vec<String>>,
}

async fn load(
    images: Vec<String>,
    points: Option<String>,
    image_to_load: usize,
) -> (
    Vec<String>,
    Option<(
        Point,
        (Point, Point),
        Vec<(Point, Point)>,
        Option<Vec<Vector3<f32>>>,
    )>,
    String,
    String,
    (u32, u32),
) {
    let image = images.get(image_to_load).unwrap();
    let image_name = Path::new(image).file_stem().unwrap();
    let export_file_name = format!("{}.fspy", image_name.to_str().unwrap());
    let lines = if let Some(points) = points {
        (Some(read_points_from_file(&points)), points)
    } else {
        let points = format!("{}.points", image_name.to_str().unwrap());

        let out = if Path::new(&points).exists() {
            Some(read_points_from_file(&points))
        } else {
            None
        };
        (out, points)
    };
    let decoded_image = image::ImageReader::open(image).unwrap().decode().unwrap();
    (
        images,
        lines.0,
        lines.1,
        export_file_name,
        (decoded_image.width(), decoded_image.height()),
    )
}

fn extract_state(
    state: (
        Vec<String>,
        Option<(
            Point,
            (Point, Point),
            Vec<(Point, Point)>,
            Option<Vec<Vector3<f32>>>,
        )>,
        String,
        String,
        (u32, u32),
    ),
) -> Message {
    match state.1 {
        Some((control_point, scale, lines, points)) => Message::LoadApplicationState {
            images: Some(state.0),
            control_point: Some(control_point),
            scale: Some(scale),
            lines: Some(lines),
            points_file_name: state.2,
            export_file_name: state.3,
            image_size: state.4,
            points,
        },
        _ => Message::LoadApplicationState {
            images: None,
            control_point: None,
            scale: None,
            lines: None,
            points_file_name: state.2,
            export_file_name: state.3,
            image_size: state.4,
            points: None,
        },
    }
}

impl Perspective {
    fn new() -> (Self, Task<Message>) {
        let args = Args::parse();
        trace!("args {:?}", args);

        let draw_lines = Rc::new(RefCell::new(vec![Vector3::<f32>::zeros()]));
        let init = Perspective {
            axis_data: Rc::new(RefCell::new(AxisData {
                control_point: Point::new(0.5, 0.5),
                scale: (Point::new(0.5, 0.5), Point::new(0.75, 0.75)),
                ..AxisData::default()
            })),
            image_path: args.images.first().unwrap().clone(),
            draw_lines,
            ..Self::default()
        };
        (
            init,
            Task::perform(load(args.images, args.points, 0), extract_state),
        )
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::Save => {
                let mut file = File::create(self.points_file_name.clone()).unwrap();

                let lines = self
                    .axis_data
                    .borrow()
                    .axis_lines
                    .iter()
                    .map(Into::into)
                    .collect::<Vec<StoreLine>>();
                let store = Lines {
                    lines,
                    control_point: StorePoint {
                        x: self.axis_data.borrow().control_point.x,
                        y: self.axis_data.borrow().control_point.y,
                    },
                    scale: StoreLine::from(&self.axis_data.borrow().scale),
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
                };
                file.write_all(&serde_json::to_vec(&store).unwrap())
                    .unwrap();
            }
            Message::CalculateAndSaveToFile => {
                let lines_x = [
                    self.axis_data.borrow().axis_lines[0],
                    self.axis_data.borrow().axis_lines[1],
                ];
                let lines_y = [
                    self.axis_data.borrow().axis_lines[2],
                    self.axis_data.borrow().axis_lines[3],
                ];
                let lines_z = [
                    self.axis_data.borrow().axis_lines[4],
                    self.axis_data.borrow().axis_lines[5],
                ];

                let control_point = &self.axis_data.borrow().control_point;
                let scale = &self.axis_data.borrow().scale;
                self.compute_solution = Some(block_on(async {
                    let compute_solution = compute_adapter(
                        lines_x,
                        lines_y,
                        lines_z,
                        self.image_width,
                        self.image_height,
                        control_point,
                        scale,
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
                images: additional_images,
                control_point,
                scale,
                lines,
                points_file_name,
                export_file_name,
                image_size,
                points,
            } => {
                self.points_file_name = points_file_name;
                if let Some(control_point) = control_point {
                    self.axis_data.borrow_mut().control_point = control_point;
                }
                if let Some(scale) = scale {
                    self.axis_data.borrow_mut().scale = scale;
                }
                if let Some(lines) = lines {
                    self.axis_data.borrow_mut().axis_lines = lines;
                } else {
                    self.axis_data.borrow_mut().axis_lines = vec![
                        (
                            Point::new(0.49291667, 0.8496296),
                            Point::new(0.66791666, 0.6798148),
                        ),
                        (
                            Point::new(0.315, 0.27925926),
                            Point::new(0.50166667, 0.17685185),
                        ),
                        (
                            Point::new(0.47104168, 0.8211111),
                            Point::new(0.27052084, 0.6020371),
                        ),
                        (
                            Point::new(0.5264583, 0.18981482),
                            Point::new(0.81083333, 0.3622222),
                        ),
                        (
                            Point::new(0.6715625, 0.5838889),
                            Point::new(0.68833333, 0.11722221),
                        ),
                        (
                            Point::new(0.32958332, 0.58518517),
                            Point::new(0.30770832, 0.05111111),
                        ),
                    ];
                }
                self.export_file_name = export_file_name;
                self.image_width = image_size.0;
                self.image_height = image_size.1;
                if let Some(points) = points {
                    self.draw_lines = Rc::new(RefCell::new(points));
                }
                if additional_images.is_some() {
                    self.images = additional_images;
                }
            }
            Message::Draw => {
                self.draw = !self.draw;
            }
            Message::SelectImage(selected) => {
                self.selected_image = selected;
            }
        }
    }
    fn view(&self) -> Element<Message> {
        let Some(images) = &self.images else {
            return center(text("Loading...").width(Fill).align_x(Center).size(50)).into();
        };
        let component: Element<Message> = if self.draw {
            DrawLine::new(&self.compute_solution, Rc::clone(&self.draw_lines))
                .image_size(Size::new(self.image_width as f32, self.image_height as f32))
                .width(Length::Fill)
                .height(Length::Fill)
                .into()
        } else {
            ComputeCameraPose::new(Rc::clone(&self.axis_data), &self.compute_solution)
                .image_size(Size::new(self.image_width as f32, self.image_height as f32))
                .width(Length::Fill)
                .height(Length::Fill)
                .into()
        };

        let additional_images: u8 = if let Some(additional_images) = &self.images {
            additional_images.len() as u8
        } else {
            1
        };

        column!(
            row!(
                column!(stack!(
                    ZoomViewer::new(images.get(self.selected_image as usize).unwrap()).scale(2.0),
                    component,
                ),)
                .width(Length::Fill),
                column!(
                    slider(
                        0..=additional_images - 1,
                        self.selected_image,
                        Message::SelectImage
                    )
                    .width(300),
                    Image::new(images.get(self.selected_image as usize).unwrap())
                        .content_fit(iced::ContentFit::Cover)
                        .width(300)
                        .height(200)
                )
            )
            .height(Length::Fill)
            .padding(20),
            row!(
                button("Perform calculations").on_press(Message::CalculateAndSaveToFile),
                button("Draw lines").on_press(Message::Draw),
                button("Save lines").on_press(Message::Save),
            )
            .width(Length::Fill)
            .padding(10)
            .spacing(5)
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
    use nalgebra::Vector2;
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
        let compute_solution =
            compute_camera_pose(&points, ratio, &user_selected_origin, &handle_position)
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
