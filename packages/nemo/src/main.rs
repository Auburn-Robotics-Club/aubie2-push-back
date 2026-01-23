use std::time::{Duration, Instant};

use aubie2::{
    hardware::{calibration::calibrate_imu, encoder::Amt102V},
    logger::RobotLogger,
    subsystems::intake::{ElementColor, HoodPosition, Intake, IntakeStage},
    theme::THEME_WAR_EAGLE,
};
use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::Differential,
    math::Angle,
    prelude::*,
    tracking::{
        shared_motors,
        wheeled::{TrackingWheel, WheeledTracking},
    },
};
use log::{LevelFilter, info};
use vexide::{prelude::*, smart::motor::BrakeMode};

mod routes;

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, WheeledTracking>,

    intake_score: Motor,
    intake_middle: Motor,
    intake_front: Motor,
    intake_hood: Motor,

    snacky: AdiDigitalOut,
    matchloader: AdiDigitalOut,
    descore: AdiDigitalOut,
    trapdoor: AdiDigitalOut,
}

impl Robot {
    // Measurements
    pub const TRACK_WIDTH: f64 = 11.5;
    pub const WHEEL_DIAMETER: f64 = 3.25;

    // Control Loops
    pub const LINEAR_PID: Pid = Pid::new(0.1, 0.001, 0.0101, Some(3.0));
    pub const LATERAL_PID: Pid = Pid::new(0.09, 0.001, 0.004, Some(2.0));
    pub const ANGUALR_PID: AngularPid =
        AngularPid::new(3.0, 0.1, 0.125, Some(Angle::from_degrees(5.0)));

    // Tolerances
    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(1.0)
        .velocity(0.25)
        .duration(Duration::from_millis(15));
    pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.05)
        .duration(Duration::from_millis(15));
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let start = Instant::now();

        self.aura().await;

        info!("Route completed successfully in {:?}.", start.elapsed());
        info!(
            "Position: {} Heading: {}°",
            self.drivetrain.tracking.position(),
            self.drivetrain.tracking.heading().as_degrees(),
        );
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self
                .drivetrain
                .model
                .drive_tank(state.left_stick.y(), state.right_stick.y());

            if state.button_right.is_pressed() {
                _ = self.intake_front.set_voltage(12.0);
                _ = self.intake_middle.set_voltage(12.0);
                _ = self.intake_score.set_voltage(-12.0);
                _ = self.intake_hood.brake(BrakeMode::Coast);
                _ = self.trapdoor.set_high();
            } else if state.button_l2.is_pressed() {
                _ = self.intake_front.set_voltage(12.0);
                _ = self.intake_middle.set_voltage(12.0);
                _ = self.intake_score.set_voltage(1.0);
                _ = self.intake_hood.set_voltage(-12.0);
            } else if state.button_r2.is_pressed() {
                _ = self.intake_front.set_voltage(12.0);
                _ = self.intake_middle.set_voltage(12.0);
                _ = self.intake_score.set_voltage(12.0);
                _ = self.intake_hood.set_voltage(12.0);
            } else if state.button_l1.is_pressed() {
                _ = self.intake_front.set_voltage(-12.0);
                _ = self.intake_middle.set_voltage(-12.0);
                _ = self.intake_hood.set_voltage(-12.0);
                _ = self.intake_score.set_voltage(-12.0);
            } else {
                _ = self.intake_front.brake(BrakeMode::Coast);
                _ = self.intake_middle.brake(BrakeMode::Coast);
                _ = self.intake_hood.brake(BrakeMode::Coast);
                _ = self.intake_score.brake(BrakeMode::Coast);
            }

            if state.button_right.is_now_released() {
                _ = self.trapdoor.set_low();
            }

            if state.button_r1.is_now_pressed() {
                _ = self.snacky.toggle();  
            }

            if state.button_y.is_now_pressed() {
                _ = self.matchloader.toggle();
            }

            if state.button_b.is_now_pressed() {
                _ = self.descore.toggle();
            }

            sleep(Motor::WRITE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    RobotLogger.init(LevelFilter::Trace).unwrap();

    let mut controller = peripherals.primary_controller;
    let mut display = peripherals.display;

    let mut imu = InertialSensor::new(peripherals.port_21);

    calibrate_imu(&mut controller, &mut display, &mut imu).await;

    let l = shared_motors![
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
    ];
    let r = shared_motors![
        Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
    ];

    let robot = Robot {
        controller,
        drivetrain: Drivetrain::new(
            Differential::from_shared(l.clone(), r.clone()),
            WheeledTracking::forward_only(
                (0.0, 0.0),
                90.0.deg(),
                [
                    TrackingWheel::new(l, Robot::WHEEL_DIAMETER, 0.0, Some(36.0 / 48.0)),
                    TrackingWheel::new(r, Robot::WHEEL_DIAMETER, 0.0, Some(36.0 / 48.0)),
                ],
                Some(imu),
            ),
        ),
        intake_front: Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),
        intake_hood: Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
        intake_middle: Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
        intake_score: Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
        matchloader: AdiDigitalOut::new(peripherals.adi_c),
        snacky: AdiDigitalOut::new(peripherals.adi_a),
        trapdoor: AdiDigitalOut::new(peripherals.adi_e),
        descore: AdiDigitalOut::new(peripherals.adi_d),
    };

    robot.compete().await;
}
