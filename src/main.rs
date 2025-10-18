use clap::{Args, Parser};
use cv::FeatureWorldMatch;
use iced::Alignment::Center;
use iced::Length::Fill;
use iced::alignment::{Horizontal, Vertical};
use iced::futures::executor::block_on;
use iced::widget::scrollable::{Direction, Scrollbar};
use iced::widget::{
    center, column, container, image, mouse_area, row, scrollable, slider, stack, text,
};
use iced::{Element, Length, Point, Size, Task, Theme};
use nalgebra::{Matrix4, Point2, Point3, Vector2, Vector3};
use perspective::compute::data::ComputeSolution;
use perspective::compute::{
    Lines, StoreLine, StorePoint, StorePoint3d, compute_camera_pose_scale, compute_ui_adapter,
    read_points_from_file, store_scene_data_to_file,
};
use perspective::optimize::{
    ortho_center_optimize, ortho_center_optimize_x, ortho_center_optimize_y, pose_optimize,
};
use perspective::read_state::{ImageData, load};
use perspective::twist::LambdaTwist;
use perspective::twist_pose_all::ComputeCameraPoseTwist;
use perspective::{AxisData, PointInformation};
use std::cell::RefCell;
use std::fmt::Debug;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::rc::Rc;
use tracing::{info, trace};
use tracing_subscriber::EnvFilter;
use zoomer::context_menu::ContextMenu;
use zoomer::editor_component::{EditComponentMessage, EditorComponent};

use anyhow::Result;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Cli {
    #[arg(short, long)]
    points: Option<String>,
    #[arg(short, long)]
    dimension: Option<f32>,
    #[command(flatten)]
    reference: Option<ReferenceCub>,
    #[arg(short, long, value_delimiter = ' ', num_args = 1.., default_value = "perspective.jpg")]
    images: Vec<String>,
}

#[derive(Debug, Args)]
struct ReferenceCub {
    x: f32,
    y: f32,
    z: f32,
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
    #[default]
    Pose,
}

#[derive(Debug, Clone)]
enum TwistPointEdit {
    Point1(Point3<f32>),
    Point2(Point3<f32>),
    Point3(Point3<f32>),
    None,
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
    OptimizeX,
    PoseLambdaTwist,
    OptimizeY,
    CalculatePoseUsingVanishingPoint,
    Edit(TwistPointEdit),
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
    reference_cub: Rc<RefCell<Vec<Point3<f32>>>>,
    selected_image: u8,
    images: Vec<String>,
    mode: UiMod,
    custom_origin_translation: Rc<RefCell<Option<Vector3<f32>>>>,
    custom_scale_segment: Rc<RefCell<Option<usize>>>,
    custom_scale: Rc<RefCell<Option<PointInformation<f32>>>>,
    custom_error: Rc<RefCell<Option<PointInformation<f32>>>>,
    zoom: f32,
    dimension: Option<f32>,
    twist_points: Rc<RefCell<Vec<Point3<f32>>>>,
    twist_points_2d: Rc<RefCell<Vec<Point2<f32>>>>,
    editor_component: EditorComponent,
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
        let args = Cli::parse();
        trace!("args {:?}", args);
        let draw_lines = Rc::new(RefCell::new(vec![Vector3::<f32>::zeros()]));
        let first_image = args.images.first().unwrap().clone();
        let image_name = Path::new(&first_image)
            .file_stem()
            .unwrap()
            .to_str()
            .unwrap();
        let points = if args.points.is_none() {
            let parent = Path::new(&first_image).parent().unwrap().to_str().unwrap();
            format!("{parent}/{image_name}.points")
        } else {
            args.points.unwrap()
        };
        let export_file_name = format!("{}.fspy", image_name);
        let dimension = args.dimension;
        let reference_cub = if let Some(reference) = args.reference {
            match reference {
                ReferenceCub { x, y, z } => {
                    let draw_lines = Rc::new(RefCell::new(vec![
                        Point3::<f32>::new(0.0, 0.0, 0.0),
                        Point3::<f32>::new(x, 0.0, 0.0),
                        Point3::<f32>::new(x, y, 0.0),
                        Point3::<f32>::new(0.0, y, 0.0),
                        Point3::<f32>::new(0.0, 0.0, 0.0),
                        // z
                        Point3::<f32>::new(0.0, 0.0, z),
                        Point3::<f32>::new(x, 0.0, z),
                        Point3::<f32>::new(x, y, z),
                        Point3::<f32>::new(0.0, y, z),
                        Point3::<f32>::new(0.0, 0.0, z),
                    ]));
                    draw_lines
                }
            }
        } else {
            Rc::new(RefCell::new(vec![Point3::<f32>::new(0.0, 0.0, 0.0)]))
        };

        let twist_points = Rc::new(RefCell::new(vec![
            Point3::new(7.54, 0.0, 0.0),
            Point3::new(3.14, 0.0, 2.4),
            Point3::new(3.57, 3.61, 0.0),
        ]));

        let twist_points_2d = Rc::new(RefCell::new(vec![
            Point2::new(0.5, 0.5),
            Point2::new(0.5, 0.5),
            Point2::new(0.5, 0.5),
        ]));

        let editor_component = EditorComponent::new(twist_points.clone());
        let init = Perspective {
            image_path: first_image.clone(),
            draw_lines,
            reference_cub,
            images: args.images,
            export_file_name,
            points_file_name: points.clone(),
            zoom: 0.5,
            dimension,
            twist_points,
            twist_points_2d,
            editor_component,
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
                if !Path::new(&self.points_file_name).exists() {
                    trace!("create file {}", self.points_file_name);
                }
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
                info!("does nothing");
                // let Some(axis_data) = &self.axis_data else {
                //     return;
                // };
                // let lines_x = [
                //     axis_data.borrow().axis_lines[0],
                //     axis_data.borrow().axis_lines[1],
                // ];
                // let lines_y = [
                //     axis_data.borrow().axis_lines[2],
                //     axis_data.borrow().axis_lines[3],
                // ];
                // let lines_z = [
                //     axis_data.borrow().axis_lines[4],
                //     axis_data.borrow().axis_lines[5],
                // ];
                // let control_point = &axis_data.borrow().control_point;
                // self.compute_solution = Some(
                //     compute_ui_adapter(
                //         lines_x,
                //         lines_y,
                //         lines_z,
                //         self.image_size,
                //         control_point,
                //         axis_data.borrow().flip,
                //         &axis_data.borrow().custom_origin_translation,
                //         &axis_data.borrow().custom_scale,
                //     )
                //     .unwrap(),
                // );
            }
            Message::CalculatePoseUsingVanishingPoint => {
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
                        *self.custom_scale.borrow_mut() = None;
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
                self.twist_points = Rc::new(RefCell::new(
                    self.axis_data
                        .as_ref()
                        .unwrap()
                        .borrow()
                        .twist_points
                        .as_ref()
                        .unwrap()
                        .clone(),
                ));
                self.twist_points_2d = Rc::new(RefCell::new(
                    self.axis_data
                        .as_ref()
                        .unwrap()
                        .borrow()
                        .twist_points_2d
                        .as_ref()
                        .unwrap()
                        .clone(),
                ));
                self.editor_component = EditorComponent::new(self.twist_points.clone());
                self.update(Message::CalculatePoseUsingVanishingPoint);
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
                let parent = Path::new(&selected_image_name)
                    .parent()
                    .unwrap()
                    .to_str()
                    .unwrap();
                self.points_file_name = format!("{parent}/{name_without_extension}.points");
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
                self.update(Message::CalculatePoseUsingVanishingPoint);
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
                self.update(Message::CalculatePoseUsingVanishingPoint);
            }
            Message::ResetTranslation => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                axis_data.borrow_mut().custom_origin_translation = None;
                self.update(Message::CalculatePoseUsingVanishingPoint);
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
                self.update(Message::CalculatePoseUsingVanishingPoint);
            }
            Message::ResetScale => {
                let Some(axis_data) = &self.axis_data else {
                    return;
                };
                axis_data.borrow_mut().custom_scale = None;
                self.update(Message::CalculatePoseUsingVanishingPoint);
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
                    self.update(Message::CalculatePoseUsingVanishingPoint);
                };
            }
            Message::OptimizeX => {
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
                    ortho_center_optimize_x(self.image_size.width / self.image_size.height, lines)
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
                    self.update(Message::CalculatePoseUsingVanishingPoint);
                };
            }
            Message::OptimizeY => {
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
                    ortho_center_optimize_y(self.image_size.width / self.image_size.height, lines)
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
                    self.update(Message::CalculatePoseUsingVanishingPoint);
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
                self.update(Message::CalculatePoseUsingVanishingPoint);
            }
            Message::ZoomChanged(zoom) => self.zoom = zoom,
            Message::PoseLambdaTwist => {
                let fx = self.image_size.width as f64;
                let fy = self.image_size.height as f64;
                let cx = self.image_size.width as f64 / 2.0;
                let cy = self.image_size.height as f64 / 2.0;
                let field_of_view = 101f64.to_radians();
                let unprojection =
                    cv::nalgebra::Perspective3::new(1.0, field_of_view, 0.1, 1000.0).inverse();
                let to_device_coord_transform = cv::nalgebra::Matrix3::new_nonuniform_scaling(
                    &cv::nalgebra::Vector2::new(fx / 2.0, -fx / 2.0),
                )
                .append_translation(&cv::nalgebra::Vector2::new(cx, cy))
                .try_inverse()
                .unwrap();
                info!("3d: {:?}", self.twist_points.borrow());
                info!("2d: {:?}", self.twist_points_2d.borrow());
                info!(
                    "2d: {:?}",
                    self.twist_points_2d
                        .borrow()
                        .iter()
                        .map(|item| {
                            cv::nalgebra::Point2::new(item.x as f64 * fx, item.y as f64 * fy)
                        })
                        .collect::<Vec<_>>()
                );
                let bearings: Vec<cv::nalgebra::Point3<f64>> = self
                    .twist_points_2d
                    .borrow()
                    .iter()
                    .map(|item| {
                        let item =
                            cv::nalgebra::Point2::new(item.x as f64 * fx, item.y as f64 * fy);
                        cv::nalgebra::Point3::from(
                            (unprojection
                                * cv::nalgebra::Point3::from(
                                    to_device_coord_transform * item.to_homogeneous(),
                                )
                                .to_homogeneous())
                            .xyz(),
                        )
                    })
                    .map(|item| item / item.z)
                    .collect();
                info!("bearings: {:?}", bearings);
                let features: Vec<FeatureWorldMatch<_>> = self
                    .twist_points
                    .borrow()
                    .iter()
                    .zip(&bearings)
                    .map(|(&world, &image)| {
                        let world = cv::nalgebra::Point3::new(
                            world.x as f64,
                            world.y as f64,
                            world.z as f64,
                        );
                        let bearing = cv::nalgebra::Unit::new_normalize(
                            cv::nalgebra::Vector3::new(image.x, image.y, 1.0),
                        );
                        FeatureWorldMatch(bearing, cv::WorldPoint(world.to_homogeneous()))
                    })
                    .collect();

                let solver = LambdaTwist::new();
                use cv::Estimator;
                let candidates = solver.estimate(features.iter().cloned());
                candidates
                    .iter()
                    .for_each(|item| info!("solution: {}", item.0.to_homogeneous()));
                let item = candidates.iter().last().unwrap();
                let solution = item.0.to_homogeneous();
                info!("using the last solution {solution}");
                self.compute_solution = Some(ComputeSolution::new(
                    Matrix4::new(
                        solution.m11 as f32,
                        solution.m12 as f32,
                        solution.m13 as f32,
                        solution.m14 as f32,
                        solution.m21 as f32,
                        solution.m22 as f32,
                        solution.m23 as f32,
                        solution.m24 as f32,
                        solution.m31 as f32,
                        solution.m32 as f32,
                        solution.m33 as f32,
                        solution.m34 as f32,
                        solution.m41 as f32,
                        solution.m42 as f32,
                        solution.m43 as f32,
                        solution.m44 as f32,
                    ),
                    Vector2::new(0.0, 0.0),
                    field_of_view as f32,
                ));

                // use cv::Consensus;
                // // Estimate potential poses with P3P.
                // // Arrsac should use the fourth point to filter and find only one model from the 4 generated.
                // let mut arrsac = Arrsac::new(1e-6, SmallRng::seed_from_u64(0));
                // if let Some(pose) = arrsac.model(&LambdaTwist::new(), features.iter().cloned()) {
                //     info!("pose: {:?}", pose.0);
                //     info!(
                //         "pose: rotation: {} {} {}",
                //         pose.0.rotation.transpose().euler_angles().0.to_degrees(),
                //         pose.0.rotation.transpose().euler_angles().1.to_degrees(),
                //         pose.0.rotation.transpose().euler_angles().2.to_degrees()
                //     );
                // } else {
                //     info!("no solution found");
                // }
            }
            Message::Edit(point) => {
                info!("insert {point:?}")
            }
        }
    }
    fn view(&self) -> Element<'_, Message> {
        let Some(axis_data) = &self.axis_data else {
            return center(text("Loading...").width(Fill).align_x(Center).size(50)).into();
        };

        let component: Element<Message> = match self.mode {
            UiMod::Pose => ComputeCameraPoseTwist::new(
                Rc::clone(axis_data),
                Rc::clone(&self.draw_lines),
                Rc::clone(&self.reference_cub),
                &self.compute_solution,
                Rc::clone(&self.custom_origin_translation),
                Rc::clone(&self.custom_scale_segment),
                Rc::clone(&self.custom_scale),
                Rc::clone(&self.custom_error),
                Rc::clone(&self.twist_points),
                Rc::clone(&self.twist_points_2d),
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
                        mouse_area(container("Perform calculations").width(Length::Fill))
                            .on_press(Message::CalculatePoseUsingVanishingPoint)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Apply Translation").width(Length::Fill))
                            .on_press(Message::ApplyTranslation)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Reset Translation").width(Length::Fill))
                            .on_press(Message::ResetTranslation)
                            .into(),
                    );
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
                    buttons.push(
                        mouse_area(container("Reset Scale").width(Length::Fill))
                            .on_press(Message::ResetScale)
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
                    buttons.push(
                        mouse_area(container("Optimize X axis").width(Length::Fill))
                            .on_press(Message::OptimizeX)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Optimize Y axis").width(Length::Fill))
                            .on_press(Message::OptimizeY)
                            .into(),
                    );
                    buttons.push(
                        mouse_area(container("Optimize Error").width(Length::Fill))
                            .on_press(Message::OptimizeForError)
                            .into(),
                    );
                }
            }
            buttons.push(
                mouse_area(container("Pose Lambda Twist").width(Length::Fill))
                    .on_press(Message::PoseLambdaTwist)
                    .into(),
            );
            column(buttons).width(300).padding(5).spacing(7).into()
        });

        let focal_length = if let Some(compute_solution) = &self.compute_solution {
            format!(
                "Focal lenght: {:.2} degrees",
                compute_solution.field_of_view().to_degrees(),
            )
        } else {
            "Focal length not avaliable. Compute the solution".into()
        };

        column!(
            row!(
                container(canvas_with_context_menu)
                    .width(Length::Fill)
                    .height(Length::Fill)
                    .align_x(Horizontal::Center)
                    .align_y(Vertical::Center),
                column!(
                    container(column!(
                        slider(0.25f32..=1.0f32, self.zoom, Message::ZoomChanged).step(0.05),
                        text(focal_length),
                        self.editor_component.view(
                            &(|action| {
                                match action {
                                    EditComponentMessage::Edit1(edit) => {
                                        Message::Edit(TwistPointEdit::Point1(edit))
                                    }

                                    EditComponentMessage::Edit2(edit) => {
                                        Message::Edit(TwistPointEdit::Point2(edit))
                                    }

                                    EditComponentMessage::Edit3(edit) => {
                                        Message::Edit(TwistPointEdit::Point3(edit))
                                    }

                                    EditComponentMessage::None => {
                                        Message::Edit(TwistPointEdit::None)
                                    }
                                }
                            })
                        ),
                        //  twist_editor(&self.edit, |action| { Message::Edit('a') }),
                    ))
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

        let twist_points = value
            .twist_points
            .borrow()
            .iter()
            .map(|item| StorePoint3d {
                x: item.x,
                y: item.y,
                z: item.z,
            })
            .collect();

        let twist_points_2d = value
            .twist_points_2d
            .borrow()
            .iter()
            .map(|item| StorePoint {
                x: item.x,
                y: item.y,
            })
            .collect();

        let custom_scale = axis_data.borrow().custom_scale;
        Lines {
            lines,
            control_point: StorePoint {
                x: axis_data.borrow().control_point.x,
                y: axis_data.borrow().control_point.y,
            },
            twist_points: Some(twist_points),
            twist_points_2d: Some(twist_points_2d),
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
    use nalgebra::{
        IsometryMatrix3, Matrix3, Perspective3, Point3, Rotation3, RowVector3, Translation,
        Vector2, Vector3,
    };
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
            RowVector3::new(-1.0, 0.0, 0.0),
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

        let rot = Rotation3::from_euler_angles(
            72.8799f64.to_radians(),
            0.048048f64.to_radians(),
            135.469f64.to_radians(),
        );

        let trans = Translation::from(Vector3::new(4.25837, 3.30374, 2.07094));
        let initial_pose = IsometryMatrix3::from_parts(trans, rot);
        info!("transform matrix: {}", initial_pose.to_matrix());

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
