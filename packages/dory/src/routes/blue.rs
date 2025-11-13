use std::time::Duration;

use aubie2::subsystems::intake::ElementColor;
use evian::{motion::{Basic, Seeking}, prelude::*};

use crate::Robot;

impl Robot {
    pub async fn blue(&mut self) {
        self.drivetrain.tracking.set_heading(90.0.deg());
        self.intake.set_reject_color(Some(ElementColor::Red));

        let dt = &mut self.drivetrain;
        let mut basic = Basic {
            linear_controller: Robot::LINEAR_PID,
            angular_controller: Robot::ANGUALR_PID,
            linear_tolerances: Robot::LINEAR_TOLERANCES,
            angular_tolerances: Robot::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };
        // let mut seeking = Seeking {
        //     linear_controller: Robot::LINEAR_PID,
        //     angular_controller: Robot::ANGUALR_PID,
        //     tolerances: Robot::LINEAR_TOLERANCES,
        //     timeout: Some(Duration::from_secs(5)),
        // };

        basic.drive_distance(dt, 24.0).await;
    }
}