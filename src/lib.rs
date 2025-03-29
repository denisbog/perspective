pub mod camera_pose;
pub mod compute;
pub mod decoder;
pub mod draw;
pub mod draw_decoration;
pub mod encoder;
pub mod fspy;
pub mod utils;
use iced::Point;
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
    ControlPoint,
    Scale {
        component: Component,
    },
    Draw,
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

#[derive(Default)]
pub struct AxisData {
    pub axis_lines: Vec<(Point, Point)>,
    pub control_point: Point,
    pub scale: (Point, Point),
}
