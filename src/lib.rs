pub mod compute;
pub mod decoder;
pub mod encoder;
pub mod frustum;
pub mod fspy;
pub mod read_state;
pub mod twist_pose_all;
pub mod utils;
use std::fmt::Debug;

use iced::Point;
use nalgebra::{Point2, Point3, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrincipalPoint {
    pub x: f32,
    pub y: f32,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraTransform {
    pub rows: [[f32; 4]; 4],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SceneSettings {
    pub camera_parameters: CameraParameters,
    pub calibration_settings_base: CalibrationSettingsBase,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CameraParameters {
    pub principal_point: PrincipalPoint,
    pub camera_transform: CameraTransform,
    pub horizontal_field_of_view: f32,
    pub image_width: u32,
    pub image_height: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CalibrationSettingsBase {
    pub reference_distance_unit: String,
}
#[derive(Debug)]
pub struct FSpyData {
    pub data: SceneSettings,
    pub image: Vec<u8>,
}
#[derive(Debug)]
pub enum Reading {
    Header,
    Data,
    Image,
}

#[derive(Debug, Clone)]
pub enum Component {
    A,
    B,
}
#[derive(Default, Clone, Debug)]
pub enum Edit {
    ControlPoint(EditAxis),
    Draw,
    Extrude(EditAxis),
    Scale(EditAxis),
    VanishingPoint(EditAxis),
    VanishingLines(EditAxis),
    #[default]
    None,
}

#[derive(Default, Clone, Debug)]
pub enum EditAxis {
    EditX,
    EditY,
    EditZ,
    #[default]
    None,
}
#[derive(Default)]
pub struct PerspectiveState {
    pub edit: Edit,
}

#[derive(Debug, Clone)]
pub struct AxisData {
    pub axis_lines: Vec<(Point, Point)>,
    pub control_point: Point,
    pub flip: (bool, bool, bool),
    pub custom_origin_translation: Option<Vector3<f32>>,
    pub custom_scale: Option<f32>,
    pub twist_points: Option<Vec<Point3<f32>>>,
    pub twist_points_2d: Option<Vec<Point2<f32>>>,
    pub field_of_view: Option<f32>,
}

impl Default for AxisData {
    fn default() -> Self {
        Self {
            control_point: Point::new(0.5, 0.5),
            axis_lines: vec![
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
            ],
            flip: (false, false, false),
            custom_origin_translation: None,
            custom_scale: None,
            twist_points: Some(vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
            ]),

            twist_points_2d: Some(vec![
                Point2::new(0.4, 0.6),
                Point2::new(0.6, 0.6),
                Point2::new(0.5, 0.4),
            ]),
            field_of_view: Some(102.0),
        }
    }
}
