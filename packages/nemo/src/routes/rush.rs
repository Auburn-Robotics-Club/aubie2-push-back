use std::time::Duration;

use aubie2::{
    motion::basic::BasicExt,
    subsystems::intake::{ElementColor, HoodPosition, IntakeStage},
};
use evian::{
    motion::{Basic, Seeking},
    prelude::*,
};
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn rush(&mut self) {
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
            tolerances: Tolerances::new()
                .error(1.0)
                .duration(Duration::from_millis(100)),
            timeout: Some(Duration::from_secs(3)),
        };

        dt.tracking.set_position((0.0, -9.0));
        dt.tracking.set_heading(45.0.deg());

        seeking.linear_controller.set_output_limit(Some(0.5));

        // rush
        _ = self
            .intake
            .set_voltage(IntakeStage::FRONT_BOTTOM | IntakeStage::BACK_BOTTOM, 12.0);
        futures::join!(
            async {
                sleep(Duration::from_millis(1200)).await;
                _ = self.intake.grabber.set_high();
            },
            async {
                basic
                    .drive_to_y(dt, 32.0, 49.0.deg())
                    .without_linear_tolerance_duration()
                    .without_angular_tolerance_duration()
                    .await;
            }
        );
        sleep(Duration::from_millis(500)).await;

        // Score in middle
        seeking.move_to_point(dt, (15.0, 14.0)).await;
        _ = self.intake.set_voltage(IntakeStage::all(), 0.0);
        _ = self.intake.grabber.set_low();

        basic.turn_to_heading(dt, 135.0.deg()).await;
        seeking
            .move_to_point(dt, (5.5, 26.0))
            .with_timeout(Duration::from_secs(1))
            .await;
        _ = self
            .intake
            .set_voltage(IntakeStage::all() ^ IntakeStage::FRONT_BOTTOM, -12.0);
        _ = self.intake.set_voltage(IntakeStage::FRONT_BOTTOM, -5.0); // slower so we dont go out the other end.

        // odom reset at center goal
        basic
            .drive_distance_at_heading(dt, 8.0, 135.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_secs(1))
            .await;
        sleep(Duration::from_secs_f64(1.5)).await; // ensure we're fully settled
        dt.tracking.set_position((6.0, 27.0));

        // go to matchload
        let matchloader_x = 40.5;
        let matchloader_angle = 269.0.deg();
        _ = self.intake.lift.set_high();
        _ = self.intake.grabber.set_high();
        seeking.move_to_point(dt, (matchloader_x, -14.0)).await;
        _ = self.intake.set_voltage(IntakeStage::all(), 12.0);
        _ = self.intake.set_hood_position(HoodPosition::Closed);
        basic.turn_to_heading(dt, matchloader_angle).await;

        // matchload
        sleep(Duration::from_millis(1000)).await;
        _ = self.intake.set_voltage(IntakeStage::all(), 12.0);
        self.intake.set_reject_color(Some(Robot::REJECT_COLOR));
        basic
            .drive_distance_at_heading(dt, 12.0, matchloader_angle)
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_secs(1))
            .await;

        basic.drive_distance_at_heading(dt, -2.0, 271.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.drive_distance_at_heading(dt, 3.0, 270.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.turn_to_heading(dt, 272.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.turn_to_heading(dt, 269.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.turn_to_heading(dt, 270.0.deg()).with_timeout(Duration::from_millis(500)).await;
        sleep(Duration::from_secs(3)).await;

        // score
        _ = self
            .intake
            .set_voltage(IntakeStage::all() ^ IntakeStage::FRONT_BOTTOM, 0.0);
        seeking
            .move_to_point(dt, (matchloader_x + 3.0, -10.25))
            .with_timeout(Duration::from_secs(3))
            .await;
        _ = self.intake.grabber.set_low();
        basic.turn_to_heading(dt, 90.0.deg()).await;
        seeking.move_to_point(dt, (matchloader_x + 3.0, 9.0)).await;
        println!("at score: {}", dt.tracking.position());
        _ = self.intake.set_hood_position(HoodPosition::Half);
        _ = self.intake.set_voltage(IntakeStage::all(), 12.0);
        basic
            .drive_distance_at_heading(dt, 4.0, 90.0.deg())
            .with_linear_output_limit(0.1)
            .with_timeout(Duration::from_millis(500))
            .await;
        sleep(Duration::from_secs(2)).await;

        // park
        // seeking.move_to_point(dt, (-7.0, 0.0)).await;

        // _ = self.intake.set_hood_position(HoodPosition::Closed);
        // _ = self.intake.lift.set_low();
        // _ = self.intake.set_voltage(IntakeStage::FRONT_BOTTOM, -12.0);

        // basic.turn_to_heading(dt, 270.0.deg()).await;
        // basic.drive_distance_at_heading(dt, 36.0, 270.0.deg()).await;
    }
}
