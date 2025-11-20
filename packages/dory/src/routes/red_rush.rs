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
            tolerances: Robot::LINEAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };

        dt.tracking.set_heading(135.0.deg());

        // rush
        _ = self.intake.set_voltage(12.0);
        join!(
            async {
                basic
                    .drive_to_y(dt, 30.0, 131.0.deg())
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
        basic
            .drive_to_y(dt, 11.5, 135.0.deg())
            .with_linear_output_limit(0.7)
            .await;
        _ = self.intake.set_voltage(0.0);

        basic.turn_to_heading(dt, 45.0.deg()).await;
        _ = self.intake.set_hood_position(HoodPosition::High);
        basic.drive_to_x(dt, 5.0, 45.0.deg()).await;
        _ = self.intake.set_bottom_voltage(12.0);
        _ = self.intake.set_top_voltage(9.0); // slower so we dont go out the other end.
        sleep(Duration::from_secs(2)).await;
        _ = self.intake.set_top_voltage(12.0);

        // go to matchload
        basic
            .drive_to_x(dt, -29.0, 45.0.deg())
            .with_linear_output_limit(0.6)
            .await;
        _ = self.intake.set_hood_position(HoodPosition::Closed);
        basic.turn_to_heading(dt, 270.0.deg()).await;

        // matchload
        _ = self.intake.lift.set_high();
        _ = self.intake.grabber.set_high();
        sleep(Duration::from_millis(1000)).await;
        basic
            .drive_distance_at_heading(dt, 15.0, 270.0.deg())
            .with_linear_output_limit(0.5)
            .with_timeout(Duration::from_secs(3))
            .await;
        basic
            .turn_to_heading(dt, 273.0.deg())
            .without_linear_tolerance_duration()
            .without_angular_tolerance_duration()
            .await;
        basic
            .turn_to_heading(dt, 267.0.deg())
            .without_linear_tolerance_duration()
            .without_angular_tolerance_duration()
            .await;
        basic
            .turn_to_heading(dt, 270.0.deg())
            .without_linear_tolerance_duration()
            .without_angular_tolerance_duration()
            .await;

        // basic
        //     .drive(dt, -4.0, 270.0.deg())
        //     .with_linear_output_limit(0.5)
        //     .with_timeout(Duration::from_secs(3))
        //     .await;
        // basic
        //     .drive(dt, 4.0, 270.0.deg())
        //     .with_linear_output_limit(0.5)
        //     .with_timeout(Duration::from_secs(3))
        //     .await;
        // basic
        //     .drive(dt, -4.0, 270.0.deg())
        //     .with_linear_output_limit(0.5)
        //     .with_timeout(Duration::from_secs(3))
        //     .await;
        // basic
        //     .drive(dt, 4.0, 270.0.deg())
        //     .with_linear_output_limit(0.5)
        //     .with_timeout(Duration::from_secs(3))
        //     .await;
        println!("at matchload: {}", dt.tracking.position());
        _ = self.intake.set_voltage(12.0);
        self.intake.set_reject_color(Some(ElementColor::Blue));
        sleep(Duration::from_secs(4)).await;

        // score
        basic.drive_distance_at_heading(dt, -13.0, 272.0.deg()).await;
        println!("back to score: {}", dt.tracking.position());
        _ = self.intake.grabber.set_low();
        basic.turn_to_heading(dt, 90.0.deg()).await;
        basic.drive_distance_at_heading(dt, 15.0, 90.0.deg()).await;
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
