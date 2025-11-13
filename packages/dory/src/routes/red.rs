use std::time::Duration;

use aubie2::subsystems::intake::ElementColor;
use evian::{
    control::loops::Pid,
    motion::{Basic, Seeking},
    prelude::*,
};
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn red(&mut self) {
        self.drivetrain.tracking.set_heading(180.0.deg());
        // self.intake.set_reject_color(Some(ElementColor::Blue));

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

        // basic.drive_distance_at_heading(dt, 8.0, 180.0.deg()).await;
        // basic.drive_distance_at_heading(dt, -8.0, 180.0.deg()).await;


        basic.drive_distance_at_heading(dt, 18.0, 180.0.deg()).await;
        basic.turn_to_heading(dt, 270.0.deg()).await;
        _ = self.intake.grabber.set_high();
        sleep(Duration::from_millis(250)).await;
        _ = self.intake.lift.set_high();
        _ = self.intake.set_voltage(12.0);
        basic
            .drive_distance_at_heading(dt, 9.0, 270.0.deg())
            .with_linear_output_limit(0.2)
            .with_timeout(Duration::from_secs(3))
            .await;
        sleep(Duration::from_secs(2)).await;
        // log::info!("unintake");
        basic.drive_distance_at_heading(dt, -5.0, 270.0.deg()).with_linear_output_limit(0.3).await;
        _ = self.intake.lift.set_low();
        self.intake.set_reject_color(Some(ElementColor::Blue));

        // println!("Hi");
        // seeking.move_to_point(dt, (-24.0, 24.0)).await;
        // basic.drive_distance(dt, 24.0).await;
        println!("Bye");
    }
}
