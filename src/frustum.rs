use nalgebra::{Matrix4, RealField, Vector3, Vector4};
use num_traits::Float;

#[derive(Clone, Copy)]
struct Plane<T> {
    normal: Vector3<T>,
    d: T,
}

impl<T: RealField + Float> Plane<T> {
    fn distance(&self, p: &Vector3<T>) -> T {
        self.normal.dot(p) + self.d
    }
}

pub struct Frustum<T> {
    planes: [Plane<T>; 6],
}

impl<T: Float + RealField> Frustum<T> {
    pub fn from_matrix(m: &Matrix4<T>) -> Self {
        let m = m.transpose(); // easier column access (nalgebra is column-major)

        let planes = [
            extract_plane(&(m.column(3) + m.column(0))), // left
            extract_plane(&(m.column(3) - m.column(0))), // right
            extract_plane(&(m.column(3) + m.column(1))), // bottom
            extract_plane(&(m.column(3) - m.column(1))), // top
            extract_plane(&(m.column(3) + m.column(2))), // near
            extract_plane(&(m.column(3) - m.column(2))), // far
        ];

        Self { planes }
    }
}

fn extract_plane<T: RealField + Float>(v: &Vector4<T>) -> Plane<T> {
    let normal = Vector3::new(v.x, v.y, v.z);
    let len = normal.norm();
    Plane {
        normal: normal / len,
        d: v.w / len,
    }
}
fn clip_line_against_plane<T: Float + RealField>(
    p0: Vector3<T>,
    p1: Vector3<T>,
    d0: T,
    d1: T,
) -> Option<(Vector3<T>, Vector3<T>)> {
    if d0 >= T::from(0.0).unwrap() && d1 >= T::from(0.0).unwrap() {
        // both inside
        Some((p0, p1))
    } else if d0 < T::from(0.0).unwrap() && d1 < T::from(0.0).unwrap() {
        // both outside
        None
    } else {
        // one inside, one outside
        let t = d0 / (d0 - d1);
        let intersection = p0 + (p1 - p0) * t;

        if d0 < T::from(0.0).unwrap() {
            // p0 is outside → move it to intersection
            Some((intersection, p1))
        } else {
            // p1 is outside → move it to intersection
            Some((p0, intersection))
        }
    }
}

pub fn clip_line_frustum<T: RealField + Float>(
    frustum: &Frustum<T>,
    mut p0: Vector3<T>,
    mut p1: Vector3<T>,
) -> Option<(Vector3<T>, Vector3<T>)> {
    for plane in &frustum.planes {
        let d0 = plane.distance(&p0);
        let d1 = plane.distance(&p1);

        if let Some((np0, np1)) = clip_line_against_plane(p0, p1, d0, d1) {
            p0 = np0;
            p1 = np1;
        } else {
            return None; // completely clipped away
        }
    }
    Some((p0, p1))
}
