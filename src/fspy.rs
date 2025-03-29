use tracing::trace;

use crate::{
    compute::ComputeSolution, CalibrationSettingsBase, CameraParameters, CameraTransform,
    PrincipalPoint, SceneSettings,
};
use anyhow::Result;

macro_rules! matrix_to_row_vec {
    ($name:ident, $row:tt) => {
        [
            *$name.index(($row, 0)),
            *$name.index(($row, 1)),
            *$name.index(($row, 2)),
            *$name.index(($row, 3)),
        ]
    };
}
pub fn compute_solution_to_scene_settings(
    image_width: u32,
    image_height: u32,
    compute_solution: &ComputeSolution,
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
                x: ortho_center.x,
                y: ortho_center.y,
            },
            camera_transform: CameraTransform {
                rows: [
                    matrix_to_row_vec!(view_transform, 0),
                    matrix_to_row_vec!(view_transform, 1),
                    matrix_to_row_vec!(view_transform, 2),
                    matrix_to_row_vec!(view_transform, 3),
                ],
            },
            horizontal_field_of_view: *field_of_view,

            image_width,
            image_height,
        },

        calibration_settings_base: CalibrationSettingsBase {
            reference_distance_unit: "Meters".to_string(),
        },
    };

    Ok(data)
}
