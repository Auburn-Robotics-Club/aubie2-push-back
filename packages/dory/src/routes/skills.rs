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
    pub async fn skills(&mut self) {
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

        dt.tracking.set_heading(180.0.deg());

        basic.drive_distance_at_heading(dt, 33.5, 180.0.deg()).await;
        basic.turn_to_heading(dt, 270.0.deg()).await;

        _ = self.hood.set_high(); // deploy
        sleep(Duration::from_millis(50)).await;
        _ = self.snacky.set_high();
        _ = self.matchloader.set_high();
        _ = self.aligner.set_high();
        sleep(Duration::from_millis(250)).await;

        // matchloader
        _ = self.intake_bottom.set_voltage(12.0);
        _ = self.intake_conveyor.set_voltage(12.0);
        _ = self.intake_score.set_voltage(0.0);
        _ = self.intake_hood.set_voltage(-2.0);
        basic
            .drive_distance_at_heading(dt, 100.0, 270.0.deg())
            .with_timeout(Duration::from_millis(2500))
            .with_linear_output_limit(0.3)
            .await;
        sleep(Duration::from_secs(1)).await;

        // Score
        basic
            .drive_distance_at_heading(dt, -100.0, 272.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_millis(3000))
            .await;
        _ = self.intake_score.set_voltage(12.0);
        _ = self.intake_hood.set_voltage(12.0);
        dt.tracking.set_position((0.0, 0.0)); // odom reset
        sleep(Duration::from_secs(3)).await;
        _ = self.intake_score.set_voltage(-2.0);
        _ = self.intake_hood.set_voltage(0.0);
        _ = self.intake_conveyor.set_voltage(-6.0);
        sleep(Duration::from_millis(150)).await;
        _ = self.intake_score.set_voltage(12.0);
        _ = self.intake_hood.set_voltage(12.0);
        _ = self.intake_conveyor.set_voltage(12.0);
        sleep(Duration::from_secs(2)).await;
        _ = self.intake_score.set_voltage(-2.0);
        _ = self.intake_hood.set_voltage(0.0);
        _ = self.intake_conveyor.set_voltage(-6.0);
        sleep(Duration::from_millis(150)).await;
        _ = self.intake_score.set_voltage(12.0);
        _ = self.intake_hood.set_voltage(12.0);
        _ = self.intake_conveyor.set_voltage(12.0);
        sleep(Duration::from_secs(2)).await;

        _ = self.intake_bottom.set_voltage(-12.0);
        _ = self.intake_conveyor.set_voltage(0.0);
        _ = self.intake_score.set_voltage(0.0);
        _ = self.intake_hood.set_voltage(0.0);
        _ = self.matchloader.set_low();
        _ = self.aligner.set_low();
        _ = self.hood.set_low();
        seeking
            .move_to_point(dt, (30.0, -34.0))
            .with_timeout(Duration::from_secs(1))
            .await;
        basic.turn_to_heading(dt, -25.0.deg()).await;
        basic.drive_distance_at_heading(dt, 28.0, 0.0.deg()).await;
        basic
            .drive_distance_at_heading(dt, 20.0, 0.0.deg())
            .with_linear_output_limit(0.2)
            .await;
        _ = self.intake_bottom.set_voltage(0.0);

        return;
    }
}
