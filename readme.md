# Notes

## implemetation

starting point for the implementation `https://github.com/stuffmatic/fSpy-Blender`

## documentation

- `https://annals-csis.org/proceedings/2012/pliks/110.pdf`

## todo

allow the user to view the zoomed region of the image under cursor
to allow the user more pricese control when editing control lines

- simple widget to display fragement of a scaled image
- when mouse is over the main image draw the zoomer widget centered
on the pixel of the image under cusor

## order of operations

According to the derivation above, the standard procedure of the three
vanishing point-based camera calibration starts from the:

1) rotation angle estimation (Equations (9)–(11)), followed by
2) principle point calculation, and finally, the
3) focal length from Equation (16).

Six unknowns thus can be solved with a unique solution.

altenative:

1) calulate focal point
2) calculate camera rotation matrix
3) calculate camera translation

## items

- paralle lines
- vanishting points (orthogonal)
- orthocenter of the triangle (vanishing points)
- principal point
- focal length
- camera rotation
- camera origin/translation

vector operation

- length (sqrt(sum (xi*xi)))
- normalization (xi/vector.lenght)
- dot product (xi*yi)

vanising points
`https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection`

ortho center:
`https://www.wolframalpha.com/input?i=triange+ortho+center`

```math
(x,y) = (
c/2, (c (a^2 + b^2 - c^2))/
(2 sqrt((a + b - c) (a - b + c) (-a + b + c) (a + b + c)))
)
```

distance to a line
`https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line`

```ts

    result.cameraParameters = this.computeCameraParameters(
      result,
      controlPointsBase,
      settingsBase,
      principalPoint,
      inputVanishingPoints[0],
      inputVanishingPoints[1],
      fRelative,
      imageWidth,
      imageHeight
    )
```

```ts

    if (settings2VP.quadModeEnabled) {
      secondVanishingPointControlState = {
        lineSegments: [
          [
            firstVanishingPointControlState.lineSegments[0][0],
            firstVanishingPointControlState.lineSegments[1][0]
          ],
          [
            firstVanishingPointControlState.lineSegments[0][1],
            firstVanishingPointControlState.lineSegments[1][1]
          ]
        ]
      }
    }

    // Compute the two input vanishing points from the provided control points
    let inputVanishingPoints = this.computeVanishingPointsFromControlPoints(
      image,
      [controlPointsBase.firstVanishingPoint, secondVanishingPointControlState],
      errors
    )

          principalPoint = MathUtil.triangleOrthoCenter(
            inputVanishingPoints[0], inputVanishingPoints[1], thirdVanishingPoint
          )


    let fRelative = this.computeFocalLength(
      inputVanishingPoints[0], inputVanishingPoints[1], principalPoint
    )



```

```ts

  static lineIntersection(line1: [Point2D, Point2D], 
      line2: [Point2D, Point2D]): Point2D | null {
    let d1 = this.distance(line1[0], line1[1])
    let d2 = this.distance(line2[0], line2[1])

    let epsilon = 1e-8
    if (Math.abs(d1) < epsilon || Math.abs(d2) < epsilon) {
      return null
    }

    // https://en.wikipedia.org/wiki/Line–line_intersection
    let x1 = line1[0].x
    let y1 = line1[0].y

    let x2 = line1[1].x
    let y2 = line1[1].y

    let x3 = line2[0].x
    let y3 = line2[0].y

    let x4 = line2[1].x
    let y4 = line2[1].y

    let denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if (Math.abs(denominator) < epsilon) {
      return null
    }

    return {
      x: ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator,
      y: ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator
    }
  }
```

```ts

  /**
   * computes vanishing points in image plane coordinates given a set of
   * vanishing point control points.
   * @param image
   * @param controlpointstates
   * @param errors
   */
  private static computeVanishingPointsFromControlPoints(
    image: ImageState,
    controlPointStates: VanishingPointControlState[],
    errors: string[]
  ): Point2D[] | null {
    let result: Point2D[] = []
    for (let i = 0; i < controlPointStates.length; i++) {
      console.log('calculate vanishing point: ', 
        JSON.stringify(controlPointStates, undefined, 2))
      let vanishingPoint = MathUtil.lineIntersection(
        controlPointStates[i].lineSegments[0],
        controlPointStates[i].lineSegments[1]
      )
      if (vanishingPoint) {
        result.push(
          CoordinatesUtil.convert(
            vanishingPoint,
            ImageCoordinateFrame.Relative,
            ImageCoordinateFrame.ImagePlane,
            image.width!,
            image.height!
          )
        )
      } else {
        errors.push('Failed to compute vanishing point')
      }
    }

    return errors.length == 0 ? result : null
  }
```

```ts

  /**
   * Computes the focal length based on two vanishing points and a center of projection.
   * See 3.2 "Determining the focal length from a single image"
   * @param Fu the first vanishing point in image plane coordinates.
   * @param Fv the second vanishing point in image plane coordinates.
   * @param P the center of projection in image plane coordinates.
   * @returns The relative focal length.
   */
  static computeFocalLength(Fu: Point2D, Fv: Point2D, P: Point2D)
    : number | null {
    // compute Puv, the orthogonal projection of P onto FuFv
    console.log('Fu: ' + JSON.stringify(Fu, undefined, 2))
    console.log('Fv: ' + JSON.stringify(Fv, undefined, 2))
    console.log('P: ' + JSON.stringify(P, undefined, 2))
    let dirFuFv = new Vector3D(Fu.x - Fv.x, Fu.y - Fv.y).normalized()
    let FvP = new Vector3D(P.x - Fv.x, P.y - Fv.y)
    let proj = dirFuFv.dot(FvP)
    let Puv = {
      x: proj * dirFuFv.x + Fv.x,
      y: proj * dirFuFv.y + Fv.y
    }

    let PPuv = new Vector3D(P.x - Puv.x, P.y - Puv.y).length
    let FvPuv = new Vector3D(Fv.x - Puv.x, Fv.y - Puv.y).length
    let FuPuv = new Vector3D(Fu.x - Puv.x, Fu.y - Puv.y).length

    let fSq = FvPuv * FuPuv - PPuv * PPuv

    if (fSq <= 0) {
      return null
    }

    let out = Math.sqrt(fSq)
    console.log('focal length: ' + JSON.stringify(out, undefined, 2))
    return out
  }

```

```ts

    let axisAssignmentMatrix = new Transform()
    let row1 = this.axisVector(settings.firstVanishingPointAxis)
    let row2 = this.axisVector(settings.secondVanishingPointAxis)
    let row3 = row1.cross(row2)
    axisAssignmentMatrix.matrix[0][0] = row1.x
    axisAssignmentMatrix.matrix[0][1] = row1.y
    axisAssignmentMatrix.matrix[0][2] = row1.z
    axisAssignmentMatrix.matrix[1][0] = row2.x
    axisAssignmentMatrix.matrix[1][1] = row2.y
    axisAssignmentMatrix.matrix[1][2] = row2.z
    axisAssignmentMatrix.matrix[2][0] = row3.x
    axisAssignmentMatrix.matrix[2][1] = row3.y
    axisAssignmentMatrix.matrix[2][2] = row3.z

    cameraParameters.principalPoint = principalPoint

    cameraParameters.horizontalFieldOfView = this.computeFieldOfView(
      imageWidth,
      imageHeight,
      relativeFocalLength,
      false
    )

    let cameraRotationMatrix = this.computeCameraRotationMatrix(
      vp1, vp2, relativeFocalLength, principalPoint
    )

    cameraParameters.viewTransform = axisAssignmentMatrix.leftMultiplied(cameraRotationMatrix)

    cameraParameters.cameraTransform = cameraParameters.viewTransform.inverted()

```

### methods

```ts


  private static computeFieldOfView(
    imageWidth: number,
    imageHeight: number,
    fRelative: number,
    vertical: boolean
  ): number {
    let aspectRatio = imageWidth / imageHeight
    let d = vertical ? 1 / aspectRatio : 1
    return 2 * Math.atan(d / fRelative)
  }

  /**
   * computes the camera rotation matrix based on two vanishing points
   * and a focal length as in section 3.3 "computing the rotation matrix".
   * @param fu the first vanishing point in normalized image coordinates.
   * @param fv the second vanishing point in normalized image coordinates.
   * @param f the relative focal length.
   * @param p the principal point
   * @returns the matrix moc
   */
  static computeCameraRotationMatrix(fu: point2d, fv: point2d, f: number, p: point2d)
    : transform {
    let ofu = new vector3d(fu.x - p.x, fu.y - p.y, -f)
    let ofv = new vector3d(fv.x - p.x, fv.y - p.y, -f)

    let s1 = ofu.length
    let uprc = ofu.normalized()

    let s2 = ofv.length
    let vprc = ofv.normalized()

    let wprc = uprc.cross(vprc)

    let m = new transform()
    m.matrix[0][0] = ofu.x / s1
    m.matrix[0][1] = ofv.x / s2
    m.matrix[0][2] = wprc.x

    m.matrix[1][0] = ofu.y / s1
    m.matrix[1][1] = ofv.y / s2
    m.matrix[1][2] = wprc.y

    m.matrix[2][0] = -f / s1
    m.matrix[2][1] = -f / s2
    m.matrix[2][2] = wprc.z
    console.log('rotation matrix: ' + json.stringify(m, undefined, 2))
    return m
  }

  private static computeTranslationVector(
    controlPoints: ControlPointsStateBase,
    settings: CalibrationSettingsBase,
    imageWidth: number,
    imageHeight: number,
    cameraParameters: CameraParameters
  ): void {
    // The 3D origin in image plane coordinates
    let origin = CoordinatesUtil.convert(
      controlPoints.origin,
      ImageCoordinateFrame.Relative,
      ImageCoordinateFrame.ImagePlane,
      imageWidth,
      imageHeight
    )

    let k = Math.tan(0.5 * cameraParameters.horizontalFieldOfView)
    let origin3D = new Vector3D(
      k * (origin.x - cameraParameters.principalPoint.x),
      k * (origin.y - cameraParameters.principalPoint.y),
      -1
    ).multipliedByScalar(this.DEFAULT_CAMERA_DISTANCE_SCALE)

    // Set a default translation vector
    cameraParameters.viewTransform.matrix[0][3] = origin3D.x
    cameraParameters.viewTransform.matrix[1][3] = origin3D.y
    cameraParameters.viewTransform.matrix[2][3] = origin3D.z

    if (settings.referenceDistanceAxis) {
      // If requested, scale the translation vector so that
      // the distance between the 3d handle positions equals the
      // specified reference distance

      // See what the distance between the 3d handle positions is given the current,
      // default, translation vector
      let referenceDistanceHandles3D = this.referenceDistanceHandlesWorldPositions(
        controlPoints,
        settings.referenceDistanceAxis,
        imageWidth,
        imageHeight,
        cameraParameters
      )
      let defaultHandleDistance = referenceDistanceHandles3D[0].subtracted(referenceDistanceHandles3D[1]).length

      // Scale the translation vector by the ratio of the reference distance
      //to the computed distance
      let referenceDistance = settings.referenceDistance
      let scale = referenceDistance / defaultHandleDistance
      origin3D.multiplyByScalar(scale)
    }

    cameraParameters.viewTransform.matrix[0][3] = origin3D.x
    cameraParameters.viewTransform.matrix[1][3] = origin3D.y
    cameraParameters.viewTransform.matrix[2][3] = origin3D.z
  }
```

```sh
RUST_LOG=perspective=trace cargo r --release -- -i perspective.jpg
RUST_LOG=local_test=trace cargo test --release local_test::local_tests::twist_test -- --no-capture
RUST_LOG=perspective=trace cargo r --release -- 7.54 3.77 2.75 -i twist/2025*.jpg
```
