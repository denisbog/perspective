use arrsac::Arrsac;
use cv::Consensus;
use cv::FeatureWorldMatch;
use cv::Projective;
use cv::estimate::LambdaTwist;
use cv::nalgebra::UnitVector3;
use cv::nalgebra::{IsometryMatrix3, Point3, Rotation3, Translation, Vector3};
use perspective::utils::line_insert_with_plane;
use perspective::utils::line_insert_with_yz_plane;
use rand::{SeedableRng, rngs::SmallRng};

fn main() {
    // Define some points in camera coordinates (with z > 0).

    let world_points = [
        [0.0f64, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, -1.0, 1.0],
        [0.0, 0.0, 1.0],
    ];

    let world_points = [
        [-0.228_125, -0.061_458_334, 1.0],
        [0.418_75, -0.581_25, 2.0],
        [1.128_125, 0.878_125, 3.0],
        [-0.528_125, 0.178_125, 2.5],
        [-0.923_424, -0.235_125, 2.8],
    ];

    let world_points = world_points.map(|p| Point3::from(p));
    // Define the camera pose.
    let rot = Rotation3::from_euler_angles(
        63.5593f64.to_radians(),
        0.0f64.to_radians(),
        46.6919f64.to_radians(),
    );
    let trans = Translation::from(Vector3::new(7.35889, -6.92579, 4.95831));
    let initial_pose = IsometryMatrix3::from_parts(trans, rot);

    // Compute world coordinates.
    let camera_depth_points = world_points.map(|p| initial_pose.transform_point(&p));

    // Compute normalized image coordinates.
    let normalized_image_coordinates = camera_depth_points.map(|p| (p / p.z).xy());

    println!(
        "normalized_image_coordinates {:?}",
        normalized_image_coordinates
    );
    normalized_image_coordinates.iter().for_each(|p| {
        let new_point = initial_pose
            .inverse_transform_point(&Point3::from(cv::nalgebra::Vector3::new(p.x, p.y, 1.0)));
        println!("new point {new_point}");
        let intersection1_3d = line_insert_with_plane(
            &nalgebra::Vector3::new(0.0, 0.0, 0.0),
            &nalgebra::Vector3::new(1.0, 0.0, 0.0),
            &nalgebra::Vector3::new(0.0, 0.0, 0.0),
            &nalgebra::Vector3::new(new_point.x, new_point.y, new_point.z),
        );

        println!("{}", intersection1_3d)
    });

    let samples: Vec<FeatureWorldMatch> = world_points
        .iter()
        .zip(&normalized_image_coordinates)
        .map(|(&world, &image)| {
            println!("before: {:?} {:?}", world, image);
            let image = UnitVector3::new_normalize(image.to_homogeneous());
            let world = Projective::from_homogeneous(world.to_homogeneous());
            println!("{:?} {:?}", world, image);
            let out = FeatureWorldMatch(image, world);
            println!("out {:?}", out);
            out
        })
        .collect();

    println!("samples {:?}", samples);
    // Estimate potential poses with P3P.
    // Arrsac should use the fourth point to filter and find only one model from the 4 generated.
    let mut arrsac = Arrsac::new(0.01, SmallRng::seed_from_u64(0));
    if let Some(pose) = arrsac.model(&LambdaTwist::new(), samples.iter().cloned()) {
        println!("current pose {:?}", pose.0);
        let rotation = pose.0.rotation.euler_angles();
        let rotation = (
            rotation.0.to_degrees(),
            rotation.1.to_degrees(),
            rotation.2.to_degrees(),
        );
        println!("rotation {:?}", rotation);
        let translation = pose.0.translation;
        println!("translation {:?}", pose.0.translation);
        let rot = Rotation3::from_euler_angles(rotation.0, rotation.1, rotation.2);
        let trans = Translation::from(translation);
        let pose = IsometryMatrix3::from_parts(trans, rot);
        normalized_image_coordinates.iter().for_each(|p| {
            println!(
                "{}",
                pose.inverse_transform_point(&Point3::from(p.to_homogeneous()))
            )
        });

        println!("initial pose: {:?}", initial_pose);
        println!("pose: {:?}", pose);
    } else {
        println!("no solution found");
    }
}
