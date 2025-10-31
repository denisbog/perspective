#[cfg(test)]
mod local_tests {
    use crate::{compute::compute_ui_adapter, read_state::load, utils::to_canvas};
    use anyhow::Result;
    use cv::{FeatureWorldMatch, WorldPoint, consensus::Arrsac, nalgebra::Unit};
    use iced::{Point, Size};
    use lambda_twist::LambdaTwist;
    use nalgebra::{Matrix4, Point3, Vector4};
    use rand::{SeedableRng, rngs::SmallRng};
    use std::{cell::RefCell, rc::Rc};
    use tracing::{info, trace};
    use tracing_subscriber::EnvFilter;

    #[tokio::test]
    async fn frustum_test() -> anyhow::Result<()> {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .init();

        let image = "room/20250713_154954.jpg".to_string();
        let points = "room/20250713_154954.points".to_string();
        if let Ok((Some(image_data), image_size)) = load(image, points, false).await {
            let axis_data = Rc::new(RefCell::new(image_data.axis_data));
            let image_size = Size::new(image_size.width as f32, image_size.height as f32);

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
            let compute_solution = compute_ui_adapter(
                lines_x,
                lines_y,
                lines_z,
                image_size,
                control_point,
                axis_data.borrow().flip,
                &axis_data.borrow().custom_origin_translation,
                &axis_data.borrow().custom_scale,
            )
            .unwrap();

            let (x, y, z) = (7.54, 3.77, 2.75);
            // let (x, y, z) = (7.54, 1.92, 2.75);
            // let (x, y, z) = (7.54, 1.0, 2.75);
            // let (x, y, z) = (0.0, 3.77, 0.0);
            let reference_cub = Rc::new(RefCell::new(vec![
                Point3::<f32>::new(0.0, 0.0, 0.0),
                Point3::<f32>::new(x, 0.0, 0.0),
                // Vector3::<f32>::new(x, y, 0.0),
                // Vector3::<f32>::new(0.0, y, 0.0),
                // Vector3::<f32>::new(0.0, 0.0, 0.0),
                // z
                // Vector3::<f32>::new(0.0, 0.0, z),
                // Vector3::<f32>::new(x, 0.0, z),
                // Vector3::<f32>::new(x, y, z),
                // Vector3::<f32>::new(0.0, y, z),
                // Vector3::<f32>::new(0.0, 0.0, z),
            ]));
            info!("compute_solution {}", compute_solution.view_transform());
            info!(
                "compute_solution transform {}",
                compute_solution.transform()
            );
            let _ = reference_cub
                .borrow()
                .iter()
                .map(|item| {
                    info!("item {item}");
                    let out = compute_solution
                        .calculate_location_position_to_2d(&item.coords.xyz())
                        .unwrap();
                    out
                })
                .map(|item| {
                    info!("intermediate {item}");
                    let out = to_canvas(image_size, &item);
                    info!("out {out}");
                    out
                })
                .map(|item| Point::new(item.x, item.y))
                .collect::<Vec<Point>>();

            let out = compute_solution.calculate_location_position_to_2d_frustum(
                &reference_cub
                    .borrow()
                    .iter()
                    .map(|item| Point3::new(item.x, item.y, item.z))
                    .collect(),
            );
            out.iter().for_each(|item| info!("vector {:?}", item));
        };
        Ok(())
    }

    #[tokio::test]
    async fn matrix_multiplication_test1() -> Result<()> {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .init();

        #[rustfmt::skip]
        let matrix = Matrix4::new(
            -0.284683,  -0.76492697,  0.03867203,  2.6360335,
            0.016588874, 0.035086438, 0.81620276, -1.1947881,
            0.93902093, -0.34943834, -0.011853154, 0.6489991,
            0.93714476, -0.34874016, -0.011829471, 0.6676824,
        );

        info!("matrix {matrix}");
        let vector = Vector4::new(0.0, 1.91, 0.0, 1.0);
        info!("vector {vector}");
        let temp = matrix * vector;
        info!("temp {temp}");
        info!(
            "vector {}",
            Point3::from_homogeneous(temp).unwrap().xy().coords
        );
        Ok(())
    }
    #[tokio::test]
    async fn twist_test() -> Result<()> {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .init();
        let first_3_points: Vec<cv::nalgebra::Point3<f64>> = vec![
            // cv::nalgebra::Point3::new(0.0, 0.0, 0.0),
            cv::nalgebra::Point3::new(0.0, 2.75, 0.0),
            cv::nalgebra::Point3::new(0.0, 0.0, 0.84),
            cv::nalgebra::Point3::new(3.77, 0.0, 0.0),
            cv::nalgebra::Point3::new(3.77, 2.75, 0.0),
        ];
        let first_3_points_2d: Vec<cv::nalgebra::Point2<f64>> = vec![
            // cv::nalgebra::Point2::new(2145.0, 1808.0),
            cv::nalgebra::Point2::new(2174.0, 1232.0),
            cv::nalgebra::Point2::new(2231.0, 1840.0),
            cv::nalgebra::Point2::new(1278.0, 1821.0),
            cv::nalgebra::Point2::new(1311.0, 1121.0),
        ];
        let first_3_points_2d: Vec<cv::nalgebra::Point2<f64>> = first_3_points_2d
            .into_iter()
            .map(|item| {
                cv::nalgebra::Point2::<f64>::new(
                    item.x / 4000.0 * 2.0 - 1.0,
                    -item.y / 3000.0 * 2.0 + 1.0,
                )
            })
            .collect();
        trace!("debug {first_3_points_2d:?}");
        let samples: Vec<FeatureWorldMatch<_>> = first_3_points
            .iter()
            .zip(&first_3_points_2d)
            .map(|(&world, &image)| {
                let image = cv::nalgebra::Point2::new(image.x, image.y);
                let world =
                    cv::nalgebra::Point3::new(world.x as f64, world.y as f64, world.z as f64);
                let image = Unit::new_normalize(image.to_homogeneous());
                let world = WorldPoint(world.to_homogeneous());
                FeatureWorldMatch(image, world)
            })
            .collect();

        use cv::Consensus;

        // Estimate potential poses with P3P.
        // Arrsac should use the fourth point to filter and find only one model from the 4 generated.
        let mut arrsac = Arrsac::new(0.1, SmallRng::seed_from_u64(0));
        if let Some(pose) = arrsac.model(&LambdaTwist::new(), samples.iter().cloned()) {
            println!("pose: {:?}", pose.0);
            println!(
                "pose: rotation: {} {} {}",
                pose.0.rotation.euler_angles().0.to_degrees(),
                pose.0.rotation.euler_angles().1.to_degrees(),
                pose.0.rotation.euler_angles().2.to_degrees()
            );
            println!("pose: rotation: {}", pose.0.translation);
        } else {
            println!("no solution found");
        }
        Ok(())
    }
    #[tokio::test]
    async fn compute_test_check() -> Result<()> {
        tracing_subscriber::fmt()
            .with_env_filter(EnvFilter::from_default_env())
            .init();

        let image = "room/20250713_154954.jpg".to_string();
        let points = "room/20250713_154954.points".to_string();
        if let Ok((Some(image_data), image_size)) = load(image, points, false).await {
            let axis_data = Rc::new(RefCell::new(image_data.axis_data));
            let image_size = Size::new(image_size.width as f32, image_size.height as f32);

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
            let compute_solution = compute_ui_adapter(
                lines_x,
                lines_y,
                lines_z,
                image_size,
                control_point,
                axis_data.borrow().flip,
                &axis_data.borrow().custom_origin_translation,
                &axis_data.borrow().custom_scale,
            )
            .unwrap();

            let (x, y, z) = (7.54, 3.77, 2.75);
            // let (x, y, z) = (7.54, 1.92, 2.75);
            // let (x, y, z) = (7.54, 1.0, 2.75);
            // let (x, y, z) = (0.0, 3.77, 0.0);
            let reference_cub = Rc::new(RefCell::new(vec![
                // Point3::<f32>::new(0.0, 0.0, 0.0),
                Point3::<f32>::new(x, 0.0, 0.0),
                // Point3::<f32>::new(x, y, 0.0),
                // Point3::<f32>::new(0.0, y, 0.0),
                // Point3::<f32>::new(0.0, 0.0, 0.0),
                // // z
                // Point3::<f32>::new(0.0, 0.0, z),
                // Point3::<f32>::new(x, 0.0, z),
                // Point3::<f32>::new(x, y, z),
                // Point3::<f32>::new(0.0, y, z),
                // Point3::<f32>::new(0.0, 0.0, z),
            ]));
            info!("compute_solution {}", compute_solution.view_transform());
            info!(
                "compute_solution transform {}",
                compute_solution.transform()
            );
            let _ = reference_cub
                .borrow()
                .iter()
                .map(|item| {
                    info!("item {item}");
                    let out = compute_solution
                        .calculate_location_position_to_2d(&item.coords.xyz())
                        .unwrap();
                    out
                })
                .map(|item| {
                    info!("intermediate {item}");
                    let out = to_canvas(image_size, &item);
                    info!("out {out}");
                    out
                })
                .map(|item| Point::new(item.x, item.y))
                .collect::<Vec<Point>>();

            let out = compute_solution.calculate_location_position_to_2d_frustum(
                &reference_cub
                    .borrow()
                    .iter()
                    .map(|item| Point3::new(item.x, item.y, item.z))
                    .collect(),
            );
            out.iter().for_each(|item| info!("vector {:?}", item));
        };
        Ok(())
    }

    #[tokio::test]
    async fn twist_plain_test() -> Result<()> {
        let world_points = vec![
            cv::nalgebra::Point3::new(7.54, 0.0, 0.0),
            cv::nalgebra::Point3::new(3.14, 0.0, 2.4),
            cv::nalgebra::Point3::new(3.57, 3.61, 0.0),
        ];

        let bearings = vec![
            cv::nalgebra::Point3::new(-0.0723005761222013, 0.1853612146477087, 1.0),
            cv::nalgebra::Point3::new(-0.6225612920309557, -0.24650130825266783, 1.0),
            cv::nalgebra::Point3::new(0.4983402252178681, 0.48548139622249836, 1.0),
        ];
        let features: Vec<FeatureWorldMatch<_>> = world_points
            .into_iter()
            .zip(bearings.into_iter())
            .map(|(coords, bearing)| {
                let bearing =
                    Unit::new_normalize(cv::nalgebra::Vector3::new(bearing.x, bearing.y, 1.0));
                FeatureWorldMatch(bearing, WorldPoint(coords.to_homogeneous()))
            })
            .collect();

        println!("------ Find solution ------");
        let solver = LambdaTwist::new();
        use cv::Estimator;
        let candidates = solver.estimate(features.iter().cloned());
        candidates.iter().for_each(|item| {
            println!("test {}", item.0.to_homogeneous().try_inverse().unwrap());
        });

        // use cv::Consensus;
        // let mut arrsac = Arrsac::new(1e0, SmallRng::seed_from_u64(0));
        // if let Some(pose) = arrsac.model(&LambdaTwist::new(), features.iter().cloned()) {
        //     println!("pose: {:?}", pose.0);
        //     println!(
        //         "pose: rotation: {} {} {}",
        //         pose.0.rotation.transpose().euler_angles().0.to_degrees(),
        //         pose.0.rotation.transpose().euler_angles().1.to_degrees(),
        //         pose.0.rotation.transpose().euler_angles().2.to_degrees()
        //     );
        // } else {
        //     println!("no solution found");
        // }
        Ok(())
    }
}
