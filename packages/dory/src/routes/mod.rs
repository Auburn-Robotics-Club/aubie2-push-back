use std::time::Duration;

use aubie2::{
    motion::basic::BasicExt,
    subsystems::intake::{ElementColor, HoodPosition, IntakeStage},
};
use evian::{
    motion::{Basic, Seeking},
    prelude::*,
};
use futures::future::join;
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn safe(&mut self) {
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

        dt.tracking.set_heading(270.0.deg());

        // drive to goal
        basic
            .drive_distance_at_heading(dt, -34.0, 270.0.deg())
            .await;
        basic.turn_to_heading(dt, 225.0.deg()).await;

        basic
            .drive_distance_at_heading(dt, -12.0, 225.0.deg())
            .with_timeout(Duration::from_millis(1000))
            .await;
        _ = self.hood.set_high(); // deploy
        sleep(Duration::from_millis(50)).await;
        _ = self.snacky.set_high();
        _ = self.intake_score.set_voltage(-6.0);
        sleep(Duration::from_millis(350)).await;

        // drive back
        basic.drive_distance_at_heading(dt, 56.0, 225.0.deg()).await;
        basic.turn_to_heading(dt, 270.0.deg()).await;

        // Reset
        basic
            .drive_distance_at_heading(dt, -100.0, 270.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_millis(1000))
            .await;
        sleep(Duration::from_millis(500)).await; // ensure we're fully settled
        dt.tracking.set_position((0.0, 0.0));

        // Matchloader
        _ = self.matchloader.set_high();
        _ = self.intake_bottom.set_voltage(12.0);
        _ = self.intake_conveyor.set_voltage(12.0);
        _ = self.intake_score.set_voltage(0.0);
        _ = self.intake_hood.set_voltage(-2.0);
        basic
            .drive_distance_at_heading(dt, 100.0, 270.0.deg())
            .with_timeout(Duration::from_millis(1500))
            .with_linear_output_limit(0.35)
            .await;

        // Score
        basic
            .drive_distance_at_heading(dt, -100.0, 270.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_millis(1500))
            .await;
        _ = self.intake_score.set_voltage(12.0);
        _ = self.intake_hood.set_voltage(12.0);
        dt.tracking.set_position((0.0, 0.0)); // odom reset
        sleep(Duration::from_secs(2)).await;

        // Matchloader
        join(
            async {
                basic
                    .drive_distance_at_heading(dt, 100.0, 270.0.deg())
                    .with_timeout(Duration::from_millis(6000))
                    .with_linear_output_limit(0.3)
                    .await;
            },
            async {
                sleep(Duration::from_millis(3500)).await;
                _ = self.intake_score.set_voltage(0.0);
                _ = self.intake_hood.set_voltage(-2.0);
            },
        )
        .await;

        // Score again
        basic
            .drive_distance_at_heading(dt, -100.0, 270.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_millis(2000))
            .await;
        _ = self.intake_score.set_voltage(12.0);
        _ = self.intake_hood.set_voltage(12.0);
        dt.tracking.set_position((0.0, 0.0)); // odom reset
        sleep(Duration::from_secs(2)).await;

        //
        _ = self.matchloader.set_low();
        _ = self.snacky.set_low();
        _ = self.intake_bottom.set_voltage(0.0);
        _ = self.intake_conveyor.set_voltage(0.0);
        _ = self.intake_score.set_voltage(0.0);
        _ = self.intake_hood.set_voltage(0.0);
        basic.drive_distance_at_heading(dt, 12.0, 270.0.deg()).await;
        basic.turn_to_heading(dt, 0.0.deg()).await;
        basic.drive_distance_at_heading(dt, 11.0, 0.0.deg()).await;
        basic.turn_to_heading(dt, 90.0.deg()).await;
        basic.drive_distance_at_heading(dt, 28.0, 90.0.deg()).await;
        _ = self.snacky.set_high();

        // basic
        //     .drive_distance_at_heading(dt, -44.0, 180.0.deg())
        //     .await;
        // println!("{}", dt.tracking.position());
        seeking.move_to_point(dt, (48.8, 7.0)).await;
        basic.turn_to_heading(dt, 270.0.deg()).await;
        basic
            .drive_distance_at_heading(dt, 100.0, 270.0.deg())
            .with_timeout(Duration::from_millis(750))
            .await;
    }
}
