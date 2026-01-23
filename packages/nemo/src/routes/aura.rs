use std::time::Duration;

use evian::{
    motion::{Basic, Seeking},
    prelude::*,
};
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn aura(&mut self) {
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

        dt.tracking.set_heading(90.0.deg());

        // drift to middle goal
        basic
            .drive_distance_at_heading(dt, 40.0, 135.0.deg())
            .with_angular_output_limit(0.1)
            .with_timeout(Duration::from_secs(1))
            .await;
        basic
            .drive_distance_at_heading(dt, 100.0, 135.0.deg())
            .with_linear_output_limit(0.1)
            .with_timeout(Duration::from_millis(250))
            .await;
        basic.drive_distance_at_heading(dt, -2.0, 135.0.deg()).await;
        _ = self.intake_front.set_voltage(-12.0);
        dt.tracking.set_position((0.0, 0.0));

        // drift back
        basic
            .drive_distance_at_heading(dt, -40.0, 270.0.deg())
            .with_timeout(Duration::from_millis(1500))
            .with_angular_output_limit(0.3)
            .await;
        basic
            .drive_distance_at_heading(dt, -100.0, 270.0.deg())
            .with_linear_output_limit(0.2)
            .with_timeout(Duration::from_millis(200))
            .await;
        dt.tracking.set_position((0.0, 0.0));
        _ = self.matchloader.set_high();

        // matchload
        _ = self.intake_front.set_voltage(12.0);
        _ = self.intake_middle.set_voltage(12.0);
        _ = self.intake_score.set_voltage(1.0);
        _ = self.intake_hood.set_voltage(-12.0);
        basic
            .drive_distance_at_heading(dt, 100.0, 268.0.deg())
            .with_timeout(Duration::from_millis(350))
            .await;
        basic
            .drive_distance_at_heading(dt, 100.0, 268.0.deg())
            .with_linear_output_limit(0.3)
            .with_timeout(Duration::from_millis(1250))
            .await;

        // Score
        basic
            .drive_distance_at_heading(dt, -100.0, 268.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_millis(2000))
            .await;
        _ = self.intake_front.set_voltage(-12.0);
        _ = self.intake_middle.set_voltage(12.0);
        _ = self.intake_score.set_voltage(12.0);
        _ = self.intake_hood.set_voltage(12.0);
        dt.tracking.set_position((0.0, 0.0)); // odom reset
        sleep(Duration::from_millis(400)).await;

        // Snacky
        _ = self.matchloader.set_low();

        basic
            .drive_distance_at_heading(dt, 10.0, 268.0.deg())
            .without_tolerance_duration()
            .without_angular_error_tolerance()
            .await;
        basic
            .drive_distance_at_heading(dt, 48.0, 90.0.deg())
            .with_linear_output_limit(0.5)
            .with_angular_output_limit(1.0)
            .await;
        // basic.drive_distance_at_heading(dt, 12.0, 270.0.deg()).await;
        // basic.turn_to_heading(dt, 180.0.deg()).await;
        // basic.drive_distance_at_heading(dt, 11.0, 180.0.deg()).await;
        // basic.turn_to_heading(dt, 90.0.deg()).await;
        // basic.drive_distance_at_heading(dt, 38.0, 90.0.deg()).await;
    }
}
