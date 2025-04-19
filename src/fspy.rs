use nalgebra::{ComplexField, Scalar};
use num_traits::Float;
use tracing::trace;

use crate::{
    CalibrationSettingsBase, CameraParameters, CameraTransform, PrincipalPoint, SceneSettings,
    compute::ComputeSolution,
};
use anyhow::Result;

macro_rules! matrix_to_row_vec {
    ($name:ident, $row:tt) => {
        [
            Into::<f32>::into(*$name.index(($row, 0))),
            Into::<f32>::into(*$name.index(($row, 1))),
            Into::<f32>::into(*$name.index(($row, 2))),
            Into::<f32>::into(*$name.index(($row, 3))),
        ]
    };
}
pub fn compute_solution_to_scene_settings<T: Float + Scalar + ComplexField + Into<f32>>(
    image_width: u32,
    image_height: u32,
    compute_solution: &ComputeSolution<T>,
) -> Result<SceneSettings> {
    let ComputeSolution {
        view_transform,
        ortho_center,
        focal_length: _,
        field_of_view,
    } = compute_solution;
    let view_transform = view_transform.try_inverse().unwrap();
    trace!("view transform inverse: {view_transform}");

    let data = SceneSettings {
        camera_parameters: CameraParameters {
            principal_point: PrincipalPoint {
                x: ortho_center.x.into(),
                y: ortho_center.y.into(),
            },
            camera_transform: CameraTransform {
                rows: [
                    matrix_to_row_vec!(view_transform, 0),
                    matrix_to_row_vec!(view_transform, 1),
                    matrix_to_row_vec!(view_transform, 2),
                    matrix_to_row_vec!(view_transform, 3),
                ],
            },
            horizontal_field_of_view: Into::<f32>::into(*field_of_view),

            image_width,
            image_height,
        },

        calibration_settings_base: CalibrationSettingsBase {
            reference_distance_unit: "Meters".to_string(),
        },
    };

    Ok(data)
}
