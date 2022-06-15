use opencv::{
    highgui,
    prelude::*,
    Result,
    videoio,
    video
};

fn main() -> Result<()> {
    let window = "video capture";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    #[cfg(ocvrs_opencv_branch_32)]
        let mut cam = videoio::VideoCapture::new_default(0)?; // 0 is the default camera
    #[cfg(not(ocvrs_opencv_branch_32))]
        let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
    let opened = videoio::VideoCapture::is_opened(&cam)?;
    if !opened {
        panic!("Unable to open default camera!");
    }
    
	
	let mut img = Mat::default();
    let mut _img_delta = Mat::default();
    let mut _img_thresh = Mat::default();
    let mog2 = as_raw_mut_BackgroundSubtractorMOG2();
	loop {
        


        cam.read(&mut img)?;
        if img.size()?.width > 0 {
            highgui::imshow(window, &mut img)?;
        }
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }
    Ok(())
}