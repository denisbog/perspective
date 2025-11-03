//! This example showcases an interactive `Canvas` for drawing Bézier curves.
use std::path::PathBuf;

use iced::widget::image::Handle;
use iced::widget::{Image, button, column, container, row, slider, text};
use iced::{Alignment, Element, Length, Theme};
use image::{DynamicImage, GenericImageView, ImageBuffer, Rgba};

pub fn main() -> iced::Result {
    iced::application(
        || Example {
            scale: 0.2,
            k3: -0.008,
            ..Example::default()
        },
        Example::update,
        Example::view,
    )
    .theme(Theme::CatppuccinMocha)
    .run()
}

#[derive(Default)]
struct Example {
    scale: f32,
    k1: f32,
    k2: f32,
    k3: f32,
    // original image bytes and dimensions
    original: Option<DynamicImage>,
    preview_handle: Option<Handle>,
    loaded_path: Option<PathBuf>,
}

#[derive(Debug, Clone)]
enum Message {
    K1Changed(f32),
    K2Changed(f32),
    K3Changed(f32),
    LoadImage,
    ImageLoaded(Option<PathBuf>),
    SaveImage,
    ScaleChanged(f32),
    Reset,
}

impl Example {
    fn recompute_preview(&mut self) {
        if let Some(ref img) = self.original {
            match undistort_image(img.clone(), self.k1, self.k2, self.k3) {
                Ok(corrected) => {
                    // Convert to RGBA8 bytes and build iced image handle
                    let rgba = corrected.to_rgba8();
                    let (w, h) = rgba.dimensions();
                    let pixels = rgba.into_raw();
                    let handle = Handle::from_rgba(w, h, pixels);
                    self.preview_handle = Some(handle);
                }
                Err(e) => eprintln!("Failed to undistort image: {}", e),
            }
        }
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::K1Changed(v) => {
                self.k1 = v;
                self.recompute_preview();
            }
            Message::K2Changed(v) => {
                self.k2 = v;
                self.recompute_preview();
            }
            Message::K3Changed(v) => {
                self.k3 = v;
                self.recompute_preview();
            }
            Message::LoadImage => {
                // Use rfd file dialog if available to pick a file, executed synchronously here by design.
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("Image", &["png", "jpg", "jpeg"])
                    .pick_file()
                {
                    self.update(Message::ImageLoaded(Some(path)));
                }
            }
            Message::ImageLoaded(opt_path) => {
                if let Some(path) = opt_path {
                    match image::open(&path) {
                        Ok(img) => {
                            let (w, h) = img.dimensions();
                            let img = img.resize(
                                (w as f32 * self.scale) as u32,
                                (h as f32 * self.scale) as u32,
                                image::imageops::FilterType::Triangle,
                            );
                            self.original = Some(img);
                            self.loaded_path = Some(path);
                            self.recompute_preview();
                        }
                        Err(e) => {
                            eprintln!("Failed to open image: {}", e);
                        }
                    }
                }
            }
            Message::SaveImage => {
                // Save corrected image — ask where to write
                if let Some(path) = rfd::FileDialog::new()
                    .set_file_name("corrected.jpg")
                    .save_file()
                {
                    match image::open(self.loaded_path.as_ref().unwrap()) {
                        Ok(img) => {
                            // compute corrected and write
                            if let Ok(corrected) =
                                undistort_image(img.clone(), self.k1, self.k2, self.k3)
                            {
                                let _ = corrected.save(&path);
                            }
                        }
                        Err(e) => {
                            eprintln!("Failed to open image: {}", e);
                        }
                    }
                }
            }
            Message::ScaleChanged(scale) => {
                self.scale = scale;
                match image::open(self.loaded_path.as_ref().unwrap()) {
                    Ok(img) => {
                        let (w, h) = img.dimensions();
                        let img = img.resize(
                            (w as f32 * self.scale) as u32,
                            (h as f32 * self.scale) as u32,
                            image::imageops::FilterType::Triangle,
                        );
                        self.original = Some(img);
                        self.recompute_preview();
                    }
                    Err(e) => {
                        eprintln!("Failed to open image: {}", e);
                    }
                }
            }
            Message::Reset => {
                self.k1 = 0.0;
                self.k2 = 0.0;
                self.k3 = 0.0;
                self.recompute_preview();
            }
        }
    }

    fn view(&self) -> Element<'_, Message> {
        let title = text("Manual Camera Calibration").size(30);

        let load_btn = button("Load Image").on_press(Message::LoadImage);
        let save_btn = button("Save Corrected Image").on_press(Message::SaveImage);

        let sliders = column![
            row![
                text(format!("scale: {:+.1}", self.scale)),
                slider(0.2..=1.0, self.scale, Message::ScaleChanged).step(0.1)
            ]
            .spacing(10)
            .align_y(Alignment::Center),
            row![
                text(format!("k1: {:+.4}", self.k1)),
                slider(-0.5..=0.5, self.k1, Message::K1Changed).step(0.001)
            ]
            .spacing(10)
            .align_y(Alignment::Center),
            row![
                text(format!("k2: {:+.4}", self.k2)),
                slider(-0.5..=0.5, self.k2, Message::K2Changed).step(0.001)
            ]
            .spacing(10)
            .align_y(Alignment::Center),
            row![
                text(format!("k3: {:+.4}", self.k3)),
                slider(-0.5..=0.5, self.k3, Message::K3Changed).step(0.001)
            ]
            .spacing(10)
            .align_y(Alignment::Center),
            row![button("Reset").on_press(Message::Reset),]
                .spacing(10)
                .align_y(Alignment::Center),
        ]
        .spacing(8);

        let img_widget = if let Some(ref handle) = self.preview_handle {
            Image::new(handle.clone())
                .width(640.0)
                .height(Length::Shrink)
        } else {
            // placeholder
            let placeholder = text("No image loaded. Click 'Load Image' to pick an image.");
            // We'll wrap placeholder in an image slot using a transparent 1x1 pixel if needed — but for simplicity return a container
            return container(
                column![title, row![load_btn, save_btn].spacing(10), placeholder]
                    .spacing(20)
                    .align_x(Alignment::Center),
            )
            .padding(20)
            .center(Length::Fill)
            .into();
        };

        let content = column![
            title,
            row![load_btn, save_btn].spacing(10),
            row![
                img_widget.width(Length::Fill).height(Length::Fill),
                sliders.width(300.0)
            ]
            .spacing(20),
        ]
        .spacing(10.0);

        container(content).padding(10.0).into()
    }
}

fn undistort_image(
    img: DynamicImage,
    k1: f32,
    k2: f32,
    k3: f32,
) -> Result<DynamicImage, image::ImageError> {
    let rgba = img.to_rgba8();
    let (w, h) = rgba.dimensions();
    let cx = (w as f32) / 2.0;
    let cy = (h as f32) / 2.0;
    let fx = (w as f32) / 2.0; // crude focal-length proxy — you may want to use a more accurate value or UI control
    let fy = (h as f32) / 2.0;

    let mut out: ImageBuffer<Rgba<u8>, Vec<u8>> = ImageBuffer::new(w, h);

    for y in 0..h {
        for x in 0..w {
            // convert target pixel to normalized coordinates
            let xn = (x as f32 - cx) / fx;
            let yn = (y as f32 - cy) / fy;
            let r2 = xn * xn + yn * yn;
            let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            // apply inverse of distortion by mapping the destination pixel back to source
            let xs = xn * radial;
            let ys = yn * radial;
            let src_x = xs * fx + cx;
            let src_y = ys * fy + cy;

            // bilinear sample
            let px = sample_bilinear(&rgba, src_x, src_y);
            out.put_pixel(x, y, px);
        }
    }

    Ok(DynamicImage::ImageRgba8(out))
}

fn sample_bilinear(img: &ImageBuffer<Rgba<u8>, Vec<u8>>, fx: f32, fy: f32) -> Rgba<u8> {
    let (w, h) = img.dimensions();
    if fx < 0.0 || fy < 0.0 || fx >= w as f32 - 1.0 || fy >= h as f32 - 1.0 {
        // outside, return black or nearest
        let sx = fx.clamp(0.0, (w - 1) as f32) as u32;
        let sy = fy.clamp(0.0, (h - 1) as f32) as u32;
        return *img.get_pixel(sx, sy);
    }

    let x0 = fx.floor() as u32;
    let y0 = fy.floor() as u32;
    let x1 = x0 + 1;
    let y1 = y0 + 1;

    let dx = fx - x0 as f32;
    let dy = fy - y0 as f32;

    let p00 = img.get_pixel(x0, y0).0;
    let p10 = img.get_pixel(x1, y0).0;
    let p01 = img.get_pixel(x0, y1).0;
    let p11 = img.get_pixel(x1, y1).0;

    let mut out = [0u8; 4];
    for i in 0..4 {
        let v = (p00[i] as f32) * (1.0 - dx) * (1.0 - dy)
            + (p10[i] as f32) * dx * (1.0 - dy)
            + (p01[i] as f32) * (1.0 - dx) * dy
            + (p11[i] as f32) * dx * dy;
        out[i] = v.clamp(0.0, 255.0) as u8;
    }
    Rgba(out)
}
