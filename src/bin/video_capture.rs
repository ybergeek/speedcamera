use opencv::{core::Point, core::Scalar, highgui, imgproc, prelude::*, videoio, Result};

fn main() -> Result<()> {
    let window = "video capture";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    #[cfg(ocvrs_opencv_branch_32)]
    let mut cam = videoio::VideoCapture::new_default(0)?; // 0 is the default camera
    #[cfg(not(ocvrs_opencv_branch_32))]
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
    let opened = videoio::VideoCapture::is_opened(&cam)?;
    let p = Point::new(100, 100);
    let color = Scalar::new(0., 0., 255.,255.);
    if !opened {
        panic!("Unable to open default camera!");
    }
    loop {
        let mut frame = Mat::default();
        cam.read(&mut frame)?;
        if frame.size()?.width > 0 {
            imgproc::put_text(
                &mut frame,
                "Text to show",
                p,
                imgproc::FONT_HERSHEY_SIMPLEX,
                1.,
                color,
                1,
                imgproc::LINE_8,
                false,
            )?;
            highgui::imshow(window, &mut frame)?;
            imgproc::put_text(
                &mut frame,
                "Text to show",
                p,
                imgproc::FONT_HERSHEY_SIMPLEX,
                1.,
                color,
                1,
                imgproc::LINE_8,
                false,
            )?;
        }
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }
    Ok(())
}
