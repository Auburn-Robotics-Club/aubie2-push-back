use std::time::Duration;

use aubie2::subsystems::intake::{ElementColor, HoodPosition};
use evian::{
    control::loops::Pid,
    motion::{Basic, Seeking},
    prelude::*,
};
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn red_safe(&mut self) {
        self.drivetrain.tracking.set_heading(180.0.deg());

        let dt = &mut self.drivetrain;
        let mut basic = Basic {
            linear_controller: Robot::LINEAR_PID,
            angular_controller: Robot::ANGUALR_PID,
            linear_tolerances: Robot::LINEAR_TOLERANCES,
            angular_tolerances: Robot::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };
        let mut seeking = Seeking {
            linear_controller: Robot::LINEAR_PID,
            lateral_controller: Robot::LATERAL_PID,
            tolerances: Robot::LINEAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };

        // Clear matchload
        basic.drive_distance_at_heading(dt, 18.0, 180.0.deg()).await;
        println!("backing to matchload: {}", dt.tracking.position());
        basic.turn_to_heading(dt, 270.0.deg()).await;
        _ = self.intake.lift.set_high();
        _ = self.intake.grabber.set_high();
        sleep(Duration::from_millis(1000)).await;
        basic
            .drive_distance_at_heading(dt, 10.0, 270.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_secs(3))
            .await;
        println!("at matchload: {}", dt.tracking.position());
        _ = self.intake.set_voltage(12.0);
        self.intake.set_reject_color(Some(ElementColor::Blue));
        sleep(Duration::from_secs(4)).await;

        // Score
        seeking.move_to_point(dt, (-17.5, 6.0)).await;
        // basic.drive_distance_at_heading(dt, -13.0, 270.0.deg()).await;
        _ = self.intake.set_voltage(0.0);
        _ = self.intake.grabber.set_low();
        basic.turn_to_heading(dt, 90.0.deg()).await;
        basic.drive_distance_at_heading(dt, 15.0, 90.0.deg()).await;
        _ = self.intake.set_hood_position(HoodPosition::Half);
        _ = self.intake.set_voltage(12.0);
        basic
            .drive_distance_at_heading(dt, 4.0, 90.0.deg())
            .with_linear_output_limit(0.1)
            .with_timeout(Duration::from_secs(3))
            .await;
        sleep(Duration::from_secs(2)).await;
    }
}
