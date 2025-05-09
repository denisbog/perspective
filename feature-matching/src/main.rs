use cv::{
    camera::pinhole::CameraIntrinsics,
    estimate::EightPoint,
    feature::akaze::Akaze,
    image::{
        image::{self, DynamicImage, GenericImageView, Rgba, RgbaImage},
        imageproc::drawing,
    },
    CameraModel, Consensus, FeatureMatch,
};
use imageproc::pixelops;
use itertools::Itertools;
use palette::{FromColor, Hsv, RgbHue, Srgb};

use arrsac::Arrsac;
use bitarray::{BitArray, Hamming};
use rand::SeedableRng;
use rand_pcg::Pcg64;
use space::{Knn, LinearKnn};
use tracing::info;
// to check https://github.com/rust-cv/akaze/blob/master/tests/estimate_pose.rs
//
const LOWES_RATIO: f32 = 0.5;
fn main() {
    // Load the image.
    let src_image_a = image::open("perspective.jpg").expect("failed to open image file");
    let src_image_b = image::open("newperspective.jpg").expect("failed to open image file");

    // Create an instance of `Akaze` with the default settings.
    let akaze = Akaze::dense();

    // Extract the features from the image using akaze.
    let (key_points_a, descriptors_a) = akaze.extract(&src_image_a);
    let (key_points_b, descriptors_b) = akaze.extract(&src_image_b);
    let matches = symmetric_matching(&descriptors_a, &descriptors_b);

    // Make a canvas with the `imageproc::drawing` module.
    // We use the blend mode so that we can draw with translucency on the image.
    // We convert the image to rgba8 during this process.
    let canvas_width = src_image_a.dimensions().0 + src_image_b.dimensions().0;
    let canvas_height = std::cmp::max(src_image_a.dimensions().1, src_image_b.dimensions().1);
    let rgba_image_a = src_image_a.to_rgba8();
    let rgba_image_b = src_image_b.to_rgba8();
    let mut canvas = RgbaImage::from_pixel(canvas_width, canvas_height, Rgba([0, 0, 0, 255]));

    // let intrinsics = CameraIntrinsics {
    //     focals: Vector2::new(9.842_439e2, 9.808_141e2),
    //     principal_point: Point2::new(6.9e2, 2.331_966e2),
    //     skew: 0.0,
    // };
    let intrinsics = CameraIntrinsics::identity();
    let matches_pose: Vec<FeatureMatch> = match_descriptors(&descriptors_a, &descriptors_b)
        .into_iter()
        .map(|(ix1, ix2)| {
            let a = intrinsics.calibrate(key_points_a[ix1]);
            let b = intrinsics.calibrate(key_points_b[ix2]);
            FeatureMatch(a, b)
        })
        .collect();

    // Run ARRSAC with the eight-point algorithm.
    let mut arrsac = Arrsac::new(0.1, Pcg64::from_seed([1; 32]));
    let eight_point = EightPoint::new();
    if let Some((_, inliers)) = arrsac.model_inliers(&eight_point, matches_pose.iter().copied()) {
        info!("inliers: {}", inliers.len());
        info!(
            "inlier ratio: {}",
            inliers.len() as f32 / matches.len() as f32
        );
    }

    // Create closure to render an image at an x offset in a canvas.
    let mut render_image_onto_canvas_x_offset = |image: &RgbaImage, x_offset: u32| {
        let (width, height) = image.dimensions();
        for (x, y) in (0..width).cartesian_product(0..height) {
            canvas.put_pixel(x + x_offset, y, *image.get_pixel(x, y));
        }
    };
    // Render image a in the top left.
    render_image_onto_canvas_x_offset(&rgba_image_a, 0);
    // Render image b just to the right of image a (in the top right).
    render_image_onto_canvas_x_offset(&rgba_image_b, rgba_image_a.dimensions().0);

    // Draw a translucent line for every match.
    for (ix, &[kpa, kpb]) in matches.iter().enumerate() {
        // Compute a color by rotating through a color wheel on only the most saturated colors.
        let ix = ix as f64;
        let hsv = Hsv::new(RgbHue::from_radians(ix * 0.1), 1.0, 1.0);
        let rgb = Srgb::from_color(hsv);

        // Draw the line between the keypoints in the two images.
        let point_to_i32_tup =
            |point: (f32, f32), off: u32| (point.0 as i32 + off as i32, point.1 as i32);
        drawing::draw_antialiased_line_segment_mut(
            &mut canvas,
            point_to_i32_tup(key_points_a[kpa].point, 0),
            point_to_i32_tup(key_points_b[kpb].point, rgba_image_a.dimensions().0),
            Rgba([
                (rgb.red * 255.0) as u8,
                (rgb.green * 255.0) as u8,
                (rgb.blue * 255.0) as u8,
                255,
            ]),
            pixelops::interpolate,
        );
    }

    // Get the resulting image.
    let out_image = DynamicImage::ImageRgba8(canvas);

    // Save the image to a temporary file.
    let image_file_path = tempfile::Builder::new()
        .suffix(".png")
        .tempfile()
        .unwrap()
        .into_temp_path();
    out_image.save(&image_file_path).unwrap();

    // Open the image with the system's default application.
    open::that(&image_file_path).unwrap();
    // Some applications may spawn in the background and take a while to begin opening the image,
    // and it isn't clear if its possible to always detect whether the child process has been closed.
    std::thread::sleep(std::time::Duration::from_secs(5));
}

/// This function performs non-symmetric matching from a to b.
fn matching(a_descriptors: &[BitArray<64>], b_descriptors: &[BitArray<64>]) -> Vec<Option<usize>> {
    let knn_b = LinearKnn {
        metric: Hamming,
        iter: b_descriptors.iter(),
    };
    (0..a_descriptors.len())
        .map(|a_feature| {
            let knn = knn_b.knn(&a_descriptors[a_feature], 2);
            if knn[0].distance + 24 < knn[1].distance {
                Some(knn[0].index)
            } else {
                None
            }
        })
        .collect()
}

/// This function performs symmetric matching between `a` and `b`.
///
/// Symmetric matching requires a feature in `b` to be the best match for a feature in `a`
/// and for the same feature in `a` to be the best match for the same feature in `b`.
/// The feature that a feature matches to in one direction might not be reciprocated.
/// Consider a 1d line. Three features are in a line `X`, `Y`, and `Z` like `X---Y-Z`.
/// `Y` is closer to `Z` than to `X`. The closest match to `X` is `Y`, but the closest
/// match to `Y` is `Z`. Therefore `X` and `Y` do not match symmetrically. However,
/// `Y` and `Z` do form a symmetric match, because the closest point to `Y` is `Z`
/// and the closest point to `Z` is `Y`.
///
/// Symmetric matching is very important for our purposes and gives stronger matches.
fn symmetric_matching(a: &[BitArray<64>], b: &[BitArray<64>]) -> Vec<[usize; 2]> {
    // The best match for each feature in frame a to frame b's features.
    let forward_matches = matching(a, b);
    // The best match for each feature in frame b to frame a's features.
    let reverse_matches = matching(b, a);
    forward_matches
        .into_iter()
        .enumerate()
        .filter_map(move |(aix, bix)| {
            // First we only proceed if there was a sufficient bix match.
            // Filter out matches which are not symmetric.
            // Symmetric is defined as the best and sufficient match of a being b,
            // and likewise the best and sufficient match of b being a.
            bix.map(|bix| [aix, bix])
                .filter(|&[aix, bix]| reverse_matches[bix] == Some(aix))
        })
        .collect()
}

fn match_descriptors(ds1: &[BitArray<64>], ds2: &[BitArray<64>]) -> Vec<(usize, usize)> {
    let two_neighbors = ds1
        .iter()
        .map(|d1| {
            let neighbors = space::LinearKnn {
                metric: Hamming,
                iter: ds2.iter(),
            }
            .knn(d1, 2);
            assert_eq!(neighbors.len(), 2, "there should be at least two matches");
            neighbors
        })
        .enumerate();
    let satisfies_lowes_ratio = two_neighbors.filter(|(_, neighbors)| {
        (neighbors[0].distance as f32) < neighbors[1].distance as f32 * LOWES_RATIO
    });
    satisfies_lowes_ratio
        .map(|(ix1, neighbors)| (ix1, neighbors[0].index))
        .collect()
}
