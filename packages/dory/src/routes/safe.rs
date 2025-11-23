use std::time::Duration;

use aubie2::subsystems::intake::{ElementColor, HoodPosition, IntakeStage};
use evian::{
    control::loops::Pid,
    motion::{Basic, Seeking},
    prelude::*,
};
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn safe(&mut self) {
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

        seeking.linear_controller.set_output_limit(Some(0.5));
        basic.linear_controller.set_output_limit(Some(0.5));

        // Clear matchload
        futures::join!(
            async {
                seeking.move_to_point(dt, (-19.0, 0.0)).await;
            },
            async {
                _ = self.doohickey.set_high();
                sleep(Duration::from_millis(250)).await;
                _ = self.doohickey.set_low();
            },
        );
        println!("backing to matchload: {}", dt.tracking.position());
        basic.turn_to_heading(dt, 270.0.deg()).await;
        _ = self.intake.lift.set_high();
        _ = self.intake.grabber.set_high();
        sleep(Duration::from_millis(1000)).await;
        _ = self.intake.set_voltage(IntakeStage::all(), 12.0);
        self.intake.set_reject_color(Some(Robot::REJECT_COLOR));
        basic
            .drive_distance_at_heading(dt, 10.0, 270.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_secs(1))
            .await;
        basic.drive_distance_at_heading(dt, -2.0, 271.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.drive_distance_at_heading(dt, 3.0, 270.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.turn_to_heading(dt, 272.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.turn_to_heading(dt, 269.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.turn_to_heading(dt, 270.0.deg()).with_timeout(Duration::from_millis(500)).await;
        println!("at matchload: {}", dt.tracking.position());
        sleep(Duration::from_secs(4)).await;

        // Score
        seeking.move_to_point(dt, (-19.25, 6.0)).await;
        _ = self.intake.set_voltage(IntakeStage::all(), 0.0);
        _ = self.intake.grabber.set_low();
        basic.turn_to_heading(dt, 90.0.deg()).await;
        seeking.move_to_point(dt, (-19.25, 24.0)).with_timeout(Duration::from_secs(1)).await;
        _ = self.intake.set_hood_position(HoodPosition::Half);
        _ = self.intake.set_voltage(IntakeStage::all(), 12.0);
        basic
            .drive_distance_at_heading(dt, 6.0, 90.0.deg())
            .with_linear_output_limit(0.1)
            .with_timeout(Duration::from_secs(3))
            .await;
        sleep(Duration::from_secs(2)).await;

        _ = self.intake.set_hood_position(HoodPosition::High);
        sleep(Duration::from_millis(250)).await;
        _ = self.intake.lift.set_low();
        _ = self.intake.set_voltage(IntakeStage::all(), 0.0);
    }
}
