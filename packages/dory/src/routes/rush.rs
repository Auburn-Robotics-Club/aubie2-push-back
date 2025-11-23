use std::time::Duration;

use aubie2::{
    motion::{basic::BasicExt},
    subsystems::intake::{ElementColor, HoodPosition, IntakeStage},
};
use evian::{
    motion::{Basic, Seeking},
    prelude::*,
};
use futures::{join};
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
            timeout: Some(Duration::from_secs(3)),
        };
        let mut seeking = Seeking {
            linear_controller: Robot::LINEAR_PID,
            lateral_controller: Robot::LATERAL_PID,
            tolerances: Robot::LINEAR_TOLERANCES,
            timeout: Some(Duration::from_secs(3)),
        };

        seeking.linear_controller.set_output_limit(Some(0.5));

        dt.tracking.set_heading(135.0.deg());

        // rush
        _ = self.intake.set_voltage(IntakeStage::all(), 12.0);
        join!(
            async {
                basic
                    .drive_to_y(dt, 32.0, 131.0.deg())
                    .without_linear_tolerance_duration()
                    .without_angular_tolerance_duration()
                    .await;
            },
            async {
                _ = self.doohickey.set_high();
                sleep(Duration::from_millis(250)).await;
                _ = self.doohickey.set_low();
            },
        );
        sleep(Duration::from_millis(50)).await;
        _ = self.intake.grabber.set_high();
        sleep(Duration::from_millis(500)).await;

        // Score in middle
        seeking.move_to_point(dt, (-6.0, 15.0)).await;
        _ = self.intake.set_voltage(IntakeStage::all(), 0.0);

        basic.turn_to_heading(dt, 45.0.deg()).await;
        _ = self.intake.set_hood_position(HoodPosition::High);
        seeking
            .move_to_point(dt, (5.0, 26.0))
            .with_timeout(Duration::from_secs(1))
            .await;
        basic.turn_to_heading(dt, 45.0.deg()).await;
        println!("{}", dt.tracking.position());
        _ = self
            .intake
            .set_voltage(IntakeStage::all() ^ IntakeStage::FRONT_TOP, 12.0);
        _ = self.intake.set_voltage(IntakeStage::FRONT_TOP, 8.0); // slower so we dont go out the other end.
        sleep(Duration::from_secs(2)).await;
        _ = self.intake.set_voltage(IntakeStage::all(), 12.0);

        // go to matchload
        seeking.move_to_point(dt, (-29.25, -8.0)).await;
        _ = self.intake.set_hood_position(HoodPosition::Closed);
        basic.turn_to_heading(dt, 270.0.deg()).await;
        _ = self.intake.lift.set_high();

        // matchload
        _ = self.intake.grabber.set_high();
        sleep(Duration::from_millis(500)).await;
        _ = self.intake.set_voltage(IntakeStage::all(), 12.0);
        seeking
            .move_to_point(dt, (-30.25, -25.0))
            .with_linear_output_limit(0.25)
            .with_timeout(Duration::from_secs_f64(1.5))
            .await;
        // schizophrenia
        self.intake.set_reject_color(Some(Robot::REJECT_COLOR));
        basic.drive_distance_at_heading(dt, -2.0, 271.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.drive_distance_at_heading(dt, 3.0, 270.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.turn_to_heading(dt, 272.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.turn_to_heading(dt, 269.0.deg()).with_timeout(Duration::from_millis(500)).await;
        basic.turn_to_heading(dt, 270.0.deg()).with_timeout(Duration::from_millis(500)).await;
        println!("at matchload: {}", dt.tracking.position());
        sleep(Duration::from_secs(4)).await;

        // score
        basic
            .drive_distance_at_heading(dt, -13.0, 272.0.deg())
            .await;
        println!("back to score: {}", dt.tracking.position());
        _ = self.intake.grabber.set_low();
        basic.turn_to_heading(dt, 90.0.deg()).await;
        _ = self
            .intake
            .set_voltage(IntakeStage::all() ^ IntakeStage::FRONT_BOTTOM, -3.0);
        seeking
            .move_to_point(dt, (-30.25, 10.0))
            .with_timeout(Duration::from_secs(1))
            .await;
        basic.turn_to_heading(dt, 90.0.deg()).await;
        println!("at score: {}", dt.tracking.position());
        _ = self.intake.set_hood_position(HoodPosition::Half);
        _ = self.intake.set_voltage(IntakeStage::all(), 12.0);
        basic
            .drive_distance_at_heading(dt, 5.0, 90.0.deg())
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
