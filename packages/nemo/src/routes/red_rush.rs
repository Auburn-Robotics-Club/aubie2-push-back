use std::time::Duration;

use aubie2::{
    motion::{basic::BasicExt, distance_sensor::DistanceSensorDriving},
    subsystems::intake::{ElementColor, HoodPosition},
};
use evian::{
    control::loops::Pid,
    motion::{Basic, Seeking},
    prelude::*,
};
use futures::{future::join, join};
use vexide::time::sleep;

use crate::Robot;

impl Robot {
    pub async fn red_rush(&mut self) {
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
                .duration(Duration::from_millis(1000)),
            timeout: Some(Duration::from_secs(3)),
        };

        dt.tracking.set_position((0.0, -9.0));
        dt.tracking.set_heading(45.0.deg());

        seeking.linear_controller.set_output_limit(Some(0.5));
        // basic.linear_controller.set_output_limit(Some(0.25));

        // rush
        _ = self.intake.set_bottom_voltage(12.0);
        join!(
            async {
                sleep(Duration::from_millis(1200)).await;
                _ = self.intake.grabber.set_high();
            },
            async {
                basic
                    .drive_to_y(dt, 29.0, 49.0.deg())
                    .without_linear_tolerance_duration()
                    .without_angular_tolerance_duration()
                    .await;
            }
        );
        sleep(Duration::from_millis(500)).await;

        // Score in middle
        seeking.move_to_point(dt, (11.0, 11.0)).await;
        // basic
        //     .drive_to_y(dt, 6.0, 46.0.deg())
        //     .with_linear_output_limit(0.7)
        //     .await;
        _ = self.intake.set_voltage(0.0);
        _ = self.intake.grabber.set_low();

        basic.turn_to_heading(dt, 130.5.deg()).await;
        seeking.move_to_point(dt, (2.5, 21.0)).await;
        basic.turn_to_heading(dt, 130.5.deg()).await;
        println!("{}", dt.tracking.position());
        _ = self.intake.set_top_voltage(-1.0);
        _ = self.intake.set_bottom_voltage(-1.0); // slower so we dont go out the other end.
        sleep(Duration::from_secs(2)).await;

        // go to matchload
        let matchloader_x = 36.5;
        seeking.move_to_point(dt, (matchloader_x, -11.0)).await;
        _ = self.intake.set_hood_position(HoodPosition::Closed);
        basic.turn_to_heading(dt, 270.0.deg()).with_linear_output_limit(0.0).without_linear_error_tolerance().await;

        // matchload
        _ = self.intake.lift.set_high();
        _ = self.intake.grabber.set_high();
        sleep(Duration::from_millis(1000)).await;
        _ = self.intake.set_voltage(12.0);
        basic
            .drive_distance_at_heading(dt, 12.5, 270.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_secs(3))
            .await;

        println!("at matchload: {}", dt.tracking.position());
        _ = self.intake.set_voltage(12.0);
        self.intake.set_reject_color(Some(ElementColor::Blue));
        sleep(Duration::from_secs(3)).await;

        // score
        seeking.move_to_point(dt, (matchloader_x - 2.0, -14.0)).with_timeout(Duration::from_secs(3)).await;
        sleep(Duration::from_secs(1)).await;
        _ = self.intake.grabber.set_low();
        basic.turn_to_heading(dt, 90.0.deg()).await;
        seeking.move_to_point(dt, (matchloader_x + 1.0, 7.0)).await;
        println!("at score: {}", dt.tracking.position());
        _ = self.intake.set_hood_position(HoodPosition::Half);
        _ = self.intake.set_voltage(12.0);
        basic
            .drive_distance_at_heading(dt, 4.0, 90.0.deg())
            .with_linear_output_limit(0.1)
            .with_timeout(Duration::from_secs(3))
            .await;
        sleep(Duration::from_secs(2)).await;

        _ = self.intake.set_hood_position(HoodPosition::High);
        sleep(Duration::from_millis(250)).await;
        _ = self.intake.lift.set_low();
        _ = self.intake.set_voltage(0.0);
    }
}
