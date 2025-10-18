// #[cfg(test)]
// mod local_tests {
//     use std::{cell::RefCell, rc::Rc};
//
//     use crate::{
//         arrsac::Arrsac,
//         compute::{
//             compute_camera_pose, compute_camera_pose_scale, compute_ui_adapter,
//             find_vanishing_point_for_lines, store_scene_data_to_file,
//         },
//         optimize::ortho_center_optimize,
//         read_state::load,
//         twist::LambdaTwist,
//         utils::{relative_to_image_plane, to_canvas},
//     };
//     use anyhow::Result;
//     use cv::{FeatureWorldMatch, Projective};
//     use iced::{Point, Size};
//     use nalgebra::{
//         Matrix3, Matrix4, Perspective3, Point3, Rotation3, RowVector3, Translation, Vector2,
//         Vector3, Vector4,
//     };
//     use rand::{SeedableRng, rngs::SmallRng};
//     use tracing::info;
//     use tracing_subscriber::EnvFilter;
//
//     use cv::nalgebra::UnitVector3;
//     #[tokio::test]
//     async fn frustum_test() -> anyhow::Result<()> {
//         tracing_subscriber::fmt()
//             .with_env_filter(EnvFilter::from_default_env())
//             .init();
//
//         let image = "/home/denis/projects/rust/perspective/room/20250713_154954.jpg".to_string();
//         let points =
//             "/home/denis/projects/rust/perspective/room/20250713_154954.points".to_string();
//         if let Ok((Some(image_data), image_size)) = load(image, points, false).await {
//             let axis_data = Rc::new(RefCell::new(image_data.axis_data));
//             let image_size = Size::new(image_size.width as f32, image_size.height as f32);
//
//             let lines_x = [
//                 axis_data.borrow().axis_lines[0],
//                 axis_data.borrow().axis_lines[1],
//             ];
//             let lines_y = [
//                 axis_data.borrow().axis_lines[2],
//                 axis_data.borrow().axis_lines[3],
//             ];
//             let lines_z = [
//                 axis_data.borrow().axis_lines[4],
//                 axis_data.borrow().axis_lines[5],
//             ];
//             let control_point = &axis_data.borrow().control_point;
//             let compute_solution = compute_ui_adapter(
//                 lines_x,
//                 lines_y,
//                 lines_z,
//                 image_size,
//                 control_point,
//                 axis_data.borrow().flip,
//                 &axis_data.borrow().custom_origin_translation,
//                 &axis_data.borrow().custom_scale,
//             )
//             .unwrap();
//
//             let (x, y, z) = (7.54, 3.77, 2.75);
//             // let (x, y, z) = (7.54, 1.92, 2.75);
//             // let (x, y, z) = (7.54, 1.0, 2.75);
//             // let (x, y, z) = (0.0, 3.77, 0.0);
//             let reference_cub = Rc::new(RefCell::new(vec![
//                 Point3::<f32>::new(0.0, 0.0, 0.0),
//                 Point3::<f32>::new(x, 0.0, 0.0),
//                 // Vector3::<f32>::new(x, y, 0.0),
//                 // Vector3::<f32>::new(0.0, y, 0.0),
//                 // Vector3::<f32>::new(0.0, 0.0, 0.0),
//                 // z
//                 // Vector3::<f32>::new(0.0, 0.0, z),
//                 // Vector3::<f32>::new(x, 0.0, z),
//                 // Vector3::<f32>::new(x, y, z),
//                 // Vector3::<f32>::new(0.0, y, z),
//                 // Vector3::<f32>::new(0.0, 0.0, z),
//             ]));
//             info!("compute_solution {}", compute_solution.view_transform());
//             info!(
//                 "compute_solution transform {}",
//                 compute_solution.transform()
//             );
//             let _ = reference_cub
//                 .borrow()
//                 .iter()
//                 .map(|item| {
//                     info!("item {item}");
//                     let out = compute_solution
//                         .calculate_location_position_to_2d(&item.coords.xyz())
//                         .unwrap();
//                     out
//                 })
//                 .map(|item| {
//                     info!("intermediate {item}");
//                     let out = to_canvas(image_size, &item);
//                     info!("out {out}");
//                     out
//                 })
//                 .map(|item| Point::new(item.x, item.y))
//                 .collect::<Vec<Point>>();
//
//             let out = compute_solution.calculate_location_position_to_2d_frustum(
//                 &reference_cub
//                     .borrow()
//                     .iter()
//                     .map(|item| Point3::new(item.x, item.y, item.z))
//                     .collect(),
//             );
//             out.iter().for_each(|item| info!("vector {:?}", item));
//         };
//         Ok(())
//     }
//
//     #[tokio::test]
//     async fn twist_test() -> Result<()> {
//         tracing_subscriber::fmt()
//             .with_env_filter(EnvFilter::from_default_env())
//             .init();
//
//         let image = "/home/denis/projects/rust/perspective/twist/20250713_154954.jpg".to_string();
//         let points =
//             "/home/denis/projects/rust/perspective/twist/20250713_154954.points".to_string();
//         if let Ok((Some(image_data), image_size)) = load(image, points, true).await {
//             let axis_data = Rc::new(RefCell::new(image_data.axis_data));
//             let image_size = Size::new(image_size.width as f32, image_size.height as f32);
//
//             let lines_x = [
//                 axis_data.borrow().axis_lines[0],
//                 axis_data.borrow().axis_lines[1],
//             ];
//             let lines_y = [
//                 axis_data.borrow().axis_lines[2],
//                 axis_data.borrow().axis_lines[3],
//             ];
//             let lines_z = [
//                 axis_data.borrow().axis_lines[4],
//                 axis_data.borrow().axis_lines[5],
//             ];
//             let control_point = &axis_data.borrow().control_point;
//             let compute_solution = compute_ui_adapter(
//                 lines_x,
//                 lines_y,
//                 lines_z,
//                 image_size,
//                 control_point,
//                 axis_data.borrow().flip,
//                 &axis_data.borrow().custom_origin_translation,
//                 &axis_data.borrow().custom_scale,
//             )
//             .unwrap();
//
//             let first_3_points = image_data
//                 .lines
//                 .unwrap()
//                 .iter()
//                 .skip(1)
//                 .take(3)
//                 .cloned()
//                 .collect::<Vec<Vector3<f32>>>();
//             println!("points {:?}", first_3_points);
//
//             println!("transform matrix: {}", compute_solution.view_transform());
//
//             first_3_points.iter().for_each(|point| {
//                 let point2d = compute_solution
//                     .calculate_location_position_to_2d(point)
//                     .unwrap();
//
//                 println!(
//                     "tranform from {} point to 2d via compute {}",
//                     point, point2d
//                 );
//             });
//             let first_3_points_2d = first_3_points
//                 .iter()
//                 .map(|point| {
//                     let point =
//                         nalgebra::Point3::new(point.x as f32, point.y as f32, point.z as f32);
//                     let point = compute_solution.view_transform().try_inverse().unwrap()
//                         * point.to_homogeneous();
//
//                     println!("point {}", point);
//                     let point = nalgebra::Point3::from_homogeneous(point).unwrap();
//                     point
//                     //
//                     //self.compute_solution
//                     //    .as_ref()
//                     //    .unwrap()
//                     //    .calculate_location_position_to_2d(point)
//                     //    .unwrap()
//                 })
//                 .map(|point| {
//                     Vector2::new(
//                         point.x as f64 / point.z as f64,
//                         point.y as f64 / point.z as f64,
//                     )
//                 })
//                 //       .map(|item| to_canvas(self.image_size, &item))
//                 //.map(|item| item.normalize())
//                 .collect::<Vec<Vector2<f64>>>();
//             println!("points: {:?}", first_3_points_2d);
//             let samples: Vec<FeatureWorldMatch> = first_3_points
//                 .iter()
//                 .zip(&first_3_points_2d)
//                 .map(|(&world, &image)| {
//                     let image = cv::nalgebra::Point2::new(image.x, image.y);
//                     let world =
//                         cv::nalgebra::Point3::new(world.x as f64, world.z as f64, world.y as f64);
//                     let image = UnitVector3::new_normalize(image.to_homogeneous());
//                     let world = Projective::from_homogeneous(world.to_homogeneous());
//                     FeatureWorldMatch(image, world)
//                 })
//                 .collect();
//
//             use cv::Consensus;
//
//             // Estimate potential poses with P3P.
//             // Arrsac should use the fourth point to filter and find only one model from the 4 generated.
//             let mut arrsac = Arrsac::new(0.01, SmallRng::seed_from_u64(0));
//             if let Some(pose) = arrsac.model(&LambdaTwist::new(), samples.iter().cloned()) {
//                 println!("pose: {:?}", pose.0);
//                 println!(
//                     "pose: rotation: {} {} {}",
//                     pose.0.rotation.euler_angles().0.to_degrees() - 90.0,
//                     pose.0.rotation.euler_angles().1.to_degrees(),
//                     pose.0.rotation.euler_angles().2.to_degrees()
//                 );
//             } else {
//                 println!("no solution found");
//             }
//         };
//         Ok(())
//     }
//     #[tokio::test]
//     async fn matrix_multiplication_test() -> Result<()> {
//         tracing_subscriber::fmt()
//             .with_env_filter(EnvFilter::from_default_env())
//             .init();
//         let matrix = Matrix4::new(
//             -1.34784904,
//             -0.93635553,
//             0.04732134,
//             3.22648,
//             0.027579434,
//             0.040232074,
//             0.99880964,
//             -1.4570445,
//             -0.93714476,
//             0.34874016,
//             0.011829471,
//             -0.6676824,
//             0.0,
//             0.0,
//             0.0,
//             1.0f32,
//         );
//
//         let vector = Vector4::new(0.0f32, 1.92, 0.0, 1.0);
//         info!(
//             "vector {}",
//             Point3::from_homogeneous(matrix * vector)
//                 .unwrap()
//                 .xy()
//                 .coords
//         );
//         Ok(())
//     }
//
//     #[tokio::test]
//     async fn matrix_multiplication_test1() -> Result<()> {
//         tracing_subscriber::fmt()
//             .with_env_filter(EnvFilter::from_default_env())
//             .init();
//
//         #[rustfmt::skip]
//         let matrix = Matrix4::new(
//             -0.284683,  -0.76492697,  0.03867203,  2.6360335,
//             0.016588874, 0.035086438, 0.81620276, -1.1947881,
//             0.93902093, -0.34943834, -0.011853154, 0.6489991,
//             0.93714476, -0.34874016, -0.011829471, 0.6676824,
//         );
//
//         info!("matrix {matrix}");
//         let vector = Vector4::new(0.0, 1.91, 0.0, 1.0);
//         info!("vector {vector}");
//         let temp = matrix * vector;
//         info!("temp {temp}");
//         info!(
//             "vector {}",
//             Point3::from_homogeneous(temp).unwrap().xy().coords
//         );
//         Ok(())
//     }
// }
