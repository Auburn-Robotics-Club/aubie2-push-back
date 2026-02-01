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

use crate::Nemo;

mod aura;
mod skills;

impl Nemo {
    pub async fn safe(&mut self) {
        let dt = &mut self.drivetrain;
        let mut basic = Basic {
            linear_controller: Nemo::LINEAR_PID,
            angular_controller: Nemo::ANGUALR_PID,
            linear_tolerances: Nemo::LINEAR_TOLERANCES,
            angular_tolerances: Nemo::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };
        let mut seeking = Seeking {
            linear_controller: Nemo::LINEAR_PID,
            lateral_controller: Nemo::LATERAL_PID,
            tolerances: Tolerances::new()
                .error(1.0)
                .duration(Duration::from_millis(100)),
            timeout: Some(Duration::from_secs(3)),
        };

        dt.tracking.set_heading(270.0.deg());

        // Descore from park zone
        _ = self.descore.set_high();
        sleep(Duration::from_millis(150)).await;
        basic
            .drive_distance_at_heading(dt, -10.0, 270.0.deg())
            .await;
        _ = self.descore.set_low();
        sleep(Duration::from_millis(100)).await;

        // Go to matchloader
        _ = self.intake_front.set_voltage(-12.0);
        basic.turn_to_heading(dt, 0.0.deg()).await;
        seeking
            .move_to_point(dt, (50.0, 9.0))
            .with_linear_output_limit(0.7)
            .await;
        println!("{}", dt.tracking.position());
        basic.turn_to_heading(dt, 270.0.deg()).await;

        // Reset
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
        sleep(Duration::from_millis(350)).await;
        _ = self.intake_front.set_voltage(12.0);
        _ = self.intake_middle.set_voltage(12.0);
        _ = self.intake_score.set_voltage(1.0);
        _ = self.intake_hood.set_voltage(-12.0);
        basic
            .drive_distance_at_heading(dt, 100.0, 268.0.deg())
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
                sleep(Duration::from_millis(100)).await;
                _ = self.intake_middle.set_voltage(0.0);
            },
            async {
                _ = self.intake_front.set_voltage(-12.0);
                sleep(Duration::from_millis(650)).await;
            },
        )
        .await;

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

        basic
            .drive_distance_at_heading(dt, 10.0, 268.0.deg())
            .without_tolerance_duration()
            .without_angular_error_tolerance()
            .await;
        basic
            .drive_distance_at_heading(dt, 28.0, 90.0.deg())
            .with_linear_output_limit(0.55)
            .with_angular_output_limit(1.0)
            .await;
        _ = self.intake_front.set_voltage(0.0);
        _ = self.intake_middle.set_voltage(0.0);
        _ = self.intake_score.set_voltage(0.0);
        _ = self.intake_hood.set_voltage(0.0);

        // basic
        //     .drive_distance_at_heading(dt, 19.0, 90.0.deg())
        //     .with_linear_kp(2.0)
        //     .with_timeout(Duration::from_millis(500))
        //     .await;

        basic
            .drive_distance_at_heading(dt, 19.0, 90.0.deg())
            .with_linear_output_limit(0.7)
            .with_timeout(Duration::from_millis(1000))
            .await;
        basic
            .drive_distance_at_heading(dt, 0.0, 90.0.deg())
            .without_timeout()
            .with_linear_error_tolerance(0.0)
            .without_linear_velocity_tolerance()
            .await;
    }
}
