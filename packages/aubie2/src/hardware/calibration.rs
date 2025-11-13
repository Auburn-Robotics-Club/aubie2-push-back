use std::time::Instant;

use log::{error, info};
use vexide::{
    color::Color,
    display::{Display, Alignment, Font, FontFamily, FontSize, Text},
    controller::Controller,
    smart::imu::InertialSensor,
};

pub async fn calibrate_imu(
    controller: &mut Controller,
    display: &mut Display,
    imu: &mut InertialSensor,
) {
    info!("Calibrating IMU");
    _ = controller.try_set_text("Calibrating...", 1, 1);
    let imu_calibration_start = Instant::now();

    if imu.calibrate().await.is_err() {
        error!("Calibration fail!");
        _ = controller.try_set_text("Calibration fail!    ", 1, 1);
        return;
    }

    let imu_calibration_elapsed = imu_calibration_start.elapsed();

    info!("Calibration completed in {:?}.", imu_calibration_elapsed);

    _ = controller.try_set_text(format!("{:?}    ", imu_calibration_elapsed), 1, 1);

    display.draw_text(
        &Text::from_string_aligned(
            format!("{:?}", imu_calibration_elapsed),
            Font::new(FontSize::LARGE, FontFamily::Monospace),
            [
                Display::HORIZONTAL_RESOLUTION / 2,
                Display::VERTICAL_RESOLUTION / 2,
            ],
            Alignment::Center,
            Alignment::Center,
        ),
        Color::new(255, 255, 255),
        None,
    );
}
