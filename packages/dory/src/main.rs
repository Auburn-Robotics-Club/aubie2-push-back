use std::time::{Duration, Instant};

use aubie2::{hardware::calibration::calibrate_imu, logger::RobotLogger, theme::THEME_WAR_EAGLE};
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
use vexide::prelude::*;

pub mod routes;

struct Dory {
    controller: Controller,
    drivetrain: Drivetrain<Differential, WheeledTracking>,
    intake_bottom: Motor,
    intake_conveyor: Motor,
    intake_hood: Motor,
    intake_score: Motor,
    hood: AdiDigitalOut,
    snacky: AdiDigitalOut,
    trapdoor: AdiDigitalOut,
    matchloader: AdiDigitalOut,
    aligner: AdiDigitalOut,
}

impl Dory {
    // Measurements
    pub const TRACK_WIDTH: f64 = 11.5;
    pub const WHEEL_DIAMETER: f64 = 2.75;
    pub const TRACKING_WHEEL_DIAMETER: f64 = 2.0;

    pub const SIDEWAYS_TRACKING_WHEEL_OFFSET: f64 = -2.5;

    // Control Loops
    pub const LINEAR_PID: Pid = Pid::new(0.1, 0.001, 0.01, Some(3.0));
    pub const LATERAL_PID: Pid = Pid::new(0.09, 0.001, 0.004, Some(2.0));
    pub const ANGUALR_PID: AngularPid =
        AngularPid::new(3.0, 0.1, 0.175, Some(Angle::from_degrees(5.0)));

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

impl Compete for Dory {
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
        _ = self.hood.set_high();
        _ = self.aligner.set_high();

        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x());

            // Intake controls
            if state.button_r1.is_pressed() {
                _ = self.intake_bottom.set_voltage(12.0);
                _ = self.intake_conveyor.set_voltage(12.0);

                if state.button_a.is_pressed() {
                    _ = self.intake_hood.set_voltage(12.0);
                    _ = self.intake_score.set_voltage(12.0);
                } else if state.button_b.is_pressed() {
                    _ = self.intake_hood.set_voltage(12.0);
                    _ = self.intake_score.set_voltage(-6.0);
                } else {
                    _ = self.intake_hood.set_voltage(-2.0);
                }
            } else if state.button_r2.is_pressed() {
                _ = self.intake_bottom.set_voltage(-12.0);
                _ = self.intake_conveyor.set_voltage(-12.0);
                _ = self.intake_hood.set_voltage(-12.0);
                _ = self.intake_score.set_voltage(-12.0);
            } else {
                _ = self.intake_bottom.set_voltage(0.0);
                _ = self.intake_conveyor.set_voltage(0.0);
                _ = self.intake_hood.set_voltage(0.0);
                _ = self.intake_score.set_voltage(0.0);
            }

            if state.button_b.is_now_pressed() {
                _ = self.trapdoor.set_high();
            }

            if state.button_b.is_now_released() {
                _ = self.trapdoor.set_low();
            }

            if state.button_x.is_now_pressed() {
                _ = self.snacky.toggle();
            }

            if state.button_l2.is_now_pressed() {
                _ = self.matchloader.toggle();
            }

            if state.button_l1.is_now_pressed() {
                _ = self.hood.toggle();
                _ = self.aligner.toggle();
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

    let mut imu = InertialSensor::new(peripherals.port_13);

    calibrate_imu(&mut controller, &mut display, &mut imu).await;

    let left_motors = shared_motors![
        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
    ];
    let right_motors = shared_motors![
        Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
    ];

    let robot = Dory {
        controller,
        drivetrain: Drivetrain::new(
            Differential::from_shared(left_motors.clone(), right_motors.clone()),
            WheeledTracking::forward_only(
                (0.0, 0.0),
                90.0.deg(),
                [
                    TrackingWheel::new(left_motors, 2.75, 0.0, None),
                    TrackingWheel::new(right_motors, 2.75, 0.0, None),
                ],
                Some(imu),
            ),
        ),
        intake_bottom: Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
        intake_conveyor: Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
        intake_hood: Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
        intake_score: Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
        hood: AdiDigitalOut::new(peripherals.adi_a),
        snacky: AdiDigitalOut::new(peripherals.adi_b),
        trapdoor: AdiDigitalOut::new(peripherals.adi_d),
        matchloader: AdiDigitalOut::new(peripherals.adi_g),
        aligner: AdiDigitalOut::new(peripherals.adi_e),
    };

    // skills : c

    robot.compete().await;
}
