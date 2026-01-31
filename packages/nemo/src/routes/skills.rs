use std::time::Duration;

use evian::{
    math::Angle,
    motion::{Basic, Seeking},
    prelude::*,
};
use futures::future::join;
use vexide::time::sleep;

use crate::Nemo;

impl Nemo {
    pub async fn skills(&mut self) {
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

        {
            // Descore from park zone
            _ = self.descore.set_high();
            sleep(Duration::from_millis(150)).await;
            _ = self.intake_front.set_voltage(12.0);
            _ = self.intake_middle.set_voltage(12.0);
            _ = self.intake_score.set_voltage(1.0);
            _ = self.intake_hood.set_voltage(-12.0);
            basic
                .drive_distance_at_heading(dt, -10.0, 270.0.deg())
                .await;
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
            sleep(Duration::from_millis(1000)).await; // ensure we're fully settled
            dt.tracking.set_position((0.0, 0.0));
            _ = self.descore.set_low();

            // Matchloader
            _ = self.matchloader.set_high();
            _ = self.intake_front.set_voltage(12.0);
            _ = self.intake_middle.set_voltage(12.0);
            _ = self.intake_score.set_voltage(1.0);
            _ = self.intake_hood.set_voltage(-12.0);
            basic
                .drive_distance_at_heading(dt, 100.0, 272.0.deg())
                .with_timeout(Duration::from_secs(4))
                .with_linear_output_limit(0.4)
                .await;

            // Score
            basic
                .drive_distance_at_heading(dt, -100.0, 272.0.deg())
                .with_linear_output_limit(0.5)
                .with_timeout(Duration::from_millis(1500))
                .await;
            _ = self.intake_front.set_voltage(12.0);
            _ = self.intake_middle.set_voltage(12.0);
            _ = self.intake_score.set_voltage(12.0);
            _ = self.intake_hood.set_voltage(12.0);
            dt.tracking.set_position((0.0, 0.0)); // odom reset
            sleep(Duration::from_secs(2)).await;

            // go
            _ = self.matchloader.set_low();

            basic.drive_distance_at_heading(dt, 18.0, 270.0.deg()).await;
            basic.turn_to_heading(dt, 0.0.deg()).await;
            basic.drive_distance_at_heading(dt, 16.0, 0.0.deg()).await;
            basic.turn_to_heading(dt, 90.0.deg()).await;

            basic.drive_distance_at_heading(dt, 90.0, 89.0.deg()).await;
            basic.turn_to_heading(dt, 0.0.deg()).await;
            basic.drive_distance_at_heading(dt, -12.25, 0.0.deg()).await;
            basic.turn_to_heading(dt, 90.0.deg()).await;
        }

        {
            // Reset
            basic
                .drive_distance_at_heading(dt, -100.0, 88.0.deg())
                .with_linear_output_limit(0.5)
                .with_timeout(Duration::from_millis(1000))
                .await;
            _ = self.intake_hood.set_voltage(12.0);
            _ = self.intake_score.set_voltage(12.0);
            sleep(Duration::from_millis(500)).await; // ensure we're fully settled
            dt.tracking.set_position((0.0, 0.0));
            _ = self.descore.set_low();

            // Matchloader
            _ = self.matchloader.set_high();
            _ = self.intake_front.set_voltage(12.0);
            _ = self.intake_middle.set_voltage(12.0);
            _ = self.intake_score.set_voltage(1.0);
            _ = self.intake_hood.set_voltage(-12.0);
            basic
                .drive_distance_at_heading(dt, 100.0, 90.0.deg())
                .with_timeout(Duration::from_secs(4))
                .with_linear_output_limit(0.3)
                .await;

            // Score
            basic
                .drive_distance_at_heading(dt, -100.0, 90.0.deg())
                .with_linear_output_limit(0.5)
                .with_timeout(Duration::from_millis(1500))
                .await;
            _ = self.intake_front.set_voltage(12.0);
            _ = self.intake_middle.set_voltage(12.0);
            _ = self.intake_score.set_voltage(12.0);
            _ = self.intake_hood.set_voltage(12.0);
            basic.turn_to_heading(dt, 90.0.deg()).await;
            dt.tracking.set_position((0.0, 0.0)); // odom reset
            sleep(Duration::from_secs(2)).await;
        }

        // go
        dt.tracking
            .set_heading(Angle::HALF_TURN + dt.tracking.heading());
        _ = self.matchloader.set_low();
        basic.drive_distance_at_heading(dt, 10.0, 270.0.deg()).await;
        basic.turn_to_heading(dt, 0.0.deg()).await;
        basic.drive_distance_at_heading(dt, 49.0, 0.0.deg()).await;
        basic.turn_to_heading(dt, 270.0.deg()).await;
        basic
            .drive_distance(dt, 10.0)
            .with_timeout(Duration::from_millis(800))
            .await;
        dt.tracking.set_position((0.0, 0.0)); // odom reset

        {
            // Descore from park zone
            _ = self.descore.set_high();
            sleep(Duration::from_millis(50)).await;
            _ = self.intake_front.set_voltage(12.0);
            _ = self.intake_middle.set_voltage(12.0);
            _ = self.intake_score.set_voltage(1.0);
            _ = self.intake_hood.set_voltage(-12.0);
            basic
                .drive_distance_at_heading(dt, -10.0, 270.0.deg())
                .await;
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
            sleep(Duration::from_millis(1000)).await; // ensure we're fully settled
            dt.tracking.set_position((0.0, 0.0));
            _ = self.descore.set_low();

            // Matchloader
            _ = self.matchloader.set_high();
            _ = self.intake_front.set_voltage(12.0);
            _ = self.intake_middle.set_voltage(12.0);
            _ = self.intake_score.set_voltage(12.0);
            _ = self.intake_hood.set_voltage(12.0);

            join(
                async {
                    basic
                        .drive_distance_at_heading(dt, 100.0, 268.0.deg())
                        .with_timeout(Duration::from_millis(4250))
                        .with_linear_output_limit(0.3)
                        .await;
                },
                async {
                    sleep(Duration::from_millis(2500)).await;
                    _ = self.intake_score.set_voltage(1.0);
                    _ = self.intake_hood.set_voltage(-12.0);
                },
            )
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

            // park
            _ = self.matchloader.set_low();
            basic
                .drive_distance_at_heading(dt, 10.0, 268.0.deg())
                .without_tolerance_duration()
                .without_angular_error_tolerance()
                .await;
            basic
                .drive_distance_at_heading(dt, 48.0, 90.0.deg())
                .without_tolerance_duration()
                .without_angular_error_tolerance()
                .await;
            basic.drive_distance_at_heading(dt, 75.0, 90.0.deg()).await;
            basic.turn_to_heading(dt, -20.0.deg()).await;
            basic.drive_distance_at_heading(dt, -26.0, 0.0.deg()).await;
        }
    }
}
