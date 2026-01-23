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

mod aura;

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

        // Descore from park zone
        // failures: 0
        _ = self.descore.set_high();
        sleep(Duration::from_millis(50)).await;
        basic
            .drive_distance_at_heading(dt, -10.0, 270.0.deg())
            .await;
        _ = self.descore.set_low();

        // Go to matchloader
        // failures: 0
        basic.turn_to_heading(dt, 0.0.deg()).await;
        seeking
            .move_to_point(dt, (50.0, 9.0))
            .with_linear_output_limit(0.7)
            .await;
        println!("{}", dt.tracking.position());
        basic.turn_to_heading(dt, 270.0.deg()).await;

        // Reset
        // Failures: 0
        basic
            .drive_distance_at_heading(dt, -12.0, 270.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_secs(1))
            .await;
        _ = self.intake_hood.set_voltage(12.0);
        _ = self.intake_score.set_voltage(12.0);
        sleep(Duration::from_millis(750)).await; // ensure we're fully settled
        dt.tracking.set_position((0.0, 0.0));

        // Matchloader
        _ = self.matchloader.set_high();
        _ = self.intake_front.set_voltage(12.0);
        _ = self.intake_middle.set_voltage(12.0);
        _ = self.intake_score.set_voltage(1.0);
        _ = self.intake_hood.set_voltage(-12.0);
        basic
            .drive_distance_at_heading(dt, 100.0, 270.0.deg())
            .with_timeout(Duration::from_secs(4))
            .with_linear_output_limit(0.3)
            .await;

        // Eject
        basic
            .drive_distance_at_heading(dt, -12.0, 270.0.deg())
            .with_linear_output_limit(0.5)
            .await;
        basic.turn_to_heading(dt, 0.0.deg()).await;
        _ = self.matchloader.set_low();
        join(
            async {
                _ = self.intake_middle.set_voltage(-12.0);
                sleep(Duration::from_millis(50)).await;
                _ = self.intake_middle.set_voltage(0.0);
            },
            async {
                _ = self.intake_front.set_voltage(-12.0);
                sleep(Duration::from_millis(750)).await;
            }
        ).await;

        _ = self.intake_front.set_voltage(0.0);

        // Round 2
        basic.turn_to_heading(dt, 272.0.deg()).await;
        _ = self.matchloader.set_high();
        _ = self.intake_front.set_voltage(12.0);
        basic
            .drive_distance_at_heading(dt, 100.0, 272.0.deg())
            .with_linear_output_limit(0.4)
            .with_timeout(Duration::from_secs(2))
            .await;

        // Score
        basic
            .drive_distance_at_heading(dt, -100.0, 268.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_millis(1500))
            .await;
        _ = self.intake_front.set_voltage(12.0);
        _ = self.intake_middle.set_voltage(12.0);
        _ = self.intake_score.set_voltage(12.0);
        _ = self.intake_hood.set_voltage(12.0);
        dt.tracking.set_position((0.0, 0.0)); // odom reset
        sleep(Duration::from_secs(2)).await;

        // Snacky
        _ = self.matchloader.set_low();
        basic.drive_distance_at_heading(dt, 12.0, 270.0.deg()).await;
        basic.turn_to_heading(dt, 180.0.deg()).await;
        basic.drive_distance_at_heading(dt, 11.0, 180.0.deg()).await;
        basic.turn_to_heading(dt, 90.0.deg()).await;
        basic.drive_distance_at_heading(dt, 32.0, 90.0.deg()).await;
    }
}
