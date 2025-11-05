# Notes

## implemetation

starting point for the implementation `https://github.com/stuffmatic/fSpy-Blender`, after switched to pose estimation using lambda twist that requires to have 3 world points

![Release](https://github.com/denisbog/perspective/releases/tag/0.1.0)

## documentation

- `https://annals-csis.org/proceedings/2012/pliks/110.pdf`
- `https://openaccess.thecvf.com/content_ECCV_2018/papers/Mikael_Persson_Lambda_Twist_An_ECCV_2018_paper.pdf`

## example

Rendered default cube:
![Default Cube](docs/default_cube.jpg)

Prespective App:
![Prespective App](docs/perspective_app.jpg)

Right Click to export pose in fSpy format:
![Right Click](docs/right_click.jpg)

Import calculated pose back to Blender:
![Import Pose](docs/imported_camera.jpg)

## cargo run from command line

```sh
RUST_LOG=perspective=trace cargo r --release -- -i perspective.jpg
```

## calibration params

```sh
cargo r --release --example calibrate

k3=-.008
field of view = 100.9

```

## build

```sh
cargo build --release
cross build --target x86_64-pc-windows-gnu --release
```
