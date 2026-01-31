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

use crate::Dory;

impl Dory {
    pub async fn aura(&mut self) {
        let dt = &mut self.drivetrain;
        let mut basic = Basic {
            linear_controller: Dory::LINEAR_PID,
            angular_controller: Dory::ANGUALR_PID,
            linear_tolerances: Dory::LINEAR_TOLERANCES,
            angular_tolerances: Dory::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };
        let mut seeking = Seeking {
            linear_controller: Dory::LINEAR_PID,
            lateral_controller: Dory::LATERAL_PID,
            tolerances: Tolerances::new()
                .error(1.0)
                .duration(Duration::from_millis(100)),
            timeout: Some(Duration::from_secs(3)),
        };

        dt.tracking.set_heading(270.0.deg());
        _ = self.aligner.set_high();

        // drive to goal
        basic
            .drive_distance_at_heading(dt, -30.0, 270.0.deg())
            .with_linear_error_tolerance(20.0)
            .with_angular_output_limit(0.75)
            .without_angular_tolerance_duration()
            .without_linear_tolerance_duration()
            .await;
        basic
            .drive_distance_at_heading(dt, -29.0, 225.0.deg())
            .with_timeout(Duration::from_secs(1))
            .await;
        _ = self.hood.set_high(); // deploy
        sleep(Duration::from_millis(50)).await;
        _ = self.snacky.set_high();
        _ = self.intake_score.set_voltage(-6.0);
        sleep(Duration::from_millis(350)).await;

        // drive back
        println!("{}", dt.tracking.position());
        basic.drive_distance_at_heading(dt, 54.0, 225.0.deg()).await;
        basic.turn_to_heading(dt, 270.0.deg()).await;

        // Reset
        basic
            .drive_distance_at_heading(dt, -100.0, 270.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_millis(1000))
            .await;
        dt.tracking.set_position((0.0, 0.0));

        // Matchloader
        _ = self.matchloader.set_high();
        _ = self.intake_bottom.set_voltage(12.0);
        _ = self.intake_conveyor.set_voltage(12.0);
        _ = self.intake_score.set_voltage(0.0);
        _ = self.intake_hood.set_voltage(-2.0);
        basic
            .drive_distance_at_heading(dt, 100.0, 272.0.deg())
            .with_timeout(Duration::from_millis(1800))
            .with_linear_output_limit(0.35)
            .await;

        // Score
        basic
            .drive_distance_at_heading(dt, -100.0, 272.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_millis(1250))
            .await;
        _ = self.intake_score.set_voltage(12.0);
        _ = self.intake_hood.set_voltage(12.0);
        dt.tracking.set_position((0.0, 0.0)); // odom reset
        sleep(Duration::from_secs(1)).await;

        // snacky
        _ = self.matchloader.set_low();
        _ = self.snacky.set_low();
        _ = self.intake_bottom.set_voltage(0.0);
        _ = self.intake_conveyor.set_voltage(0.0);
        _ = self.intake_score.set_voltage(0.0);
        _ = self.intake_hood.set_voltage(0.0);
        basic
            .drive_distance_at_heading(dt, 12.0, 270.0.deg())
            .without_linear_tolerance_duration()
            .without_angular_tolerance_duration()
            .await;
        basic.turn_to_heading(dt, 0.0.deg()).await;
        basic
            .drive_distance_at_heading(dt, 12.5, 0.0.deg())
            .without_linear_tolerance_duration()
            .without_angular_tolerance_duration()
            .await;
        basic.turn_to_heading(dt, 90.0.deg()).await;
        basic
            .drive_distance_at_heading(dt, 38.0, 90.0.deg())
            .without_timeout()
            .with_linear_error_tolerance(0.0)
            .without_linear_velocity_tolerance()
            .await;
    }
}
