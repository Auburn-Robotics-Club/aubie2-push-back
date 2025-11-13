use std::time::{Duration, Instant};

use aubie2::{
    hardware::{calibration::calibrate_imu, encoder::Amt102V},
    logger::RobotLogger,
    subsystems::intake::{ElementColor, HoodPosition, Intake},
    theme::THEME_WAR_EAGLE,
};
use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::Differential,
    math::Angle,
    prelude::*,
    tracking::wheeled::{TrackingWheel, WheeledTracking},
};
use log::{LevelFilter, info};
use vexide::prelude::*;

pub mod routes;

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, WheeledTracking>,
    intake: Intake<4, 1>,
    snacky: AdiDigitalOut,
}

impl Robot {
    // Measurements
    pub const TRACK_WIDTH: f64 = 11.5;
    pub const WHEEL_DIAMETER: f64 = 2.75;
    pub const TRACKING_WHEEL_DIAMETER: f64 = 2.0;

    pub const SIDEWAYS_TRACKING_WHEEL_OFFSET: f64 = -2.5;

    // Control Loops
    pub const LINEAR_PID: Pid = Pid::new(0.2, 0.005, 0.01, Some(3.0));
    pub const LATERAL_PID: Pid = Pid::new(0.09, 0.001, 0.004, Some(2.0));
    pub const ANGUALR_PID: AngularPid =
        AngularPid::new(3.0, 0.1, 0.175, Some(Angle::from_degrees(5.0)));

    // Tolerances
    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(5.0)
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

        #[cfg(route = "red")]
        self.red().await;
        #[cfg(route = "blue")]
        self.blue().await;

        info!("Route completed successfully in {:?}.", start.elapsed());
        info!(
            "Position: {} Heading: {}°",
            self.drivetrain.tracking.position(),
            self.drivetrain.tracking.heading().as_degrees(),
        );
    }

    async fn driver(&mut self) {
        self.intake.set_reject_color(Some(ElementColor::Blue));

        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x());

            // Intake controls
            if state.button_r1.is_pressed() {
                _ = self.intake.set_voltage(12.0);
            } else if state.button_r2.is_pressed() {
                _ = self.intake.set_voltage(-12.0);
            } else {
                _ = self.intake.set_voltage(0.0);
            }

            // Grabber
            if state.button_l2.is_pressed() {
                if self.intake.grabber.is_low().unwrap_or_default() {
                    _ = self.intake.grabber.set_high();
                }
            } else {
                if self.intake.grabber.is_high().unwrap_or_default() {
                    _ = self.intake.grabber.set_low();
                }
            }

            if state.button_l1.is_now_pressed() {
                _ = self.intake.lift.toggle();
            }

            // if state.button_y.is_now_pressed() {
            //     _ = self.intake.ejector.toggle();
            // }

            if state.button_x.is_now_pressed() {
                _ = self.snacky.toggle();
            }

            if state.button_a.is_now_pressed() {
                let new_position = match (
                    self.intake.lift.is_high().unwrap_or_default(),
                    self.intake.hood_position().unwrap_or_default(),
                ) {
                    (_, HoodPosition::High | HoodPosition::Half) => HoodPosition::Closed,
                    (true, HoodPosition::Closed) => HoodPosition::Half,
                    (false, HoodPosition::Closed) => HoodPosition::High,
                };

                _ = self.intake.set_hood_position(new_position);

                _ = self
                    .controller
                    .rumble(if new_position == HoodPosition::Closed {
                        "."
                    } else {
                        ".."
                    })
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

    let expander = AdiExpander::new(peripherals.port_3);
    let forward_tracker_2 = Amt102V::new(expander.adi_e, expander.adi_f, Direction::Reverse);
    let forward_tracker = Amt102V::new(expander.adi_c, expander.adi_d, Direction::Reverse);
    let sideways_tracker = Amt102V::new(expander.adi_a, expander.adi_b, Direction::Forward);

    let mut imu = InertialSensor::new(peripherals.port_4);

    // calibrate_imu(&mut controller, &mut display, &mut imu).await;

    let robot = Robot {
        controller,
        drivetrain: Drivetrain::new(
            Differential::new(
                [
                    Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
                ],
                [
                    Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
                ],
            ),
            WheeledTracking::new(
                (0.0, 0.0),
                90.0.deg(),
                [TrackingWheel::new(
                    forward_tracker,
                    Robot::TRACKING_WHEEL_DIAMETER,
                    0.0,
                    None,
                ),
                TrackingWheel::new(
                    forward_tracker_2,
                    Robot::TRACKING_WHEEL_DIAMETER,
                    0.0,
                    None,
                )],
                [TrackingWheel::new(
                    sideways_tracker,
                    Robot::TRACKING_WHEEL_DIAMETER,
                    Robot::SIDEWAYS_TRACKING_WHEEL_OFFSET,
                    None,
                )],
                Some(imu),
            ),
        ),
        intake: Intake::new(
            [
                Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
            ],
            [Motor::new(
                peripherals.port_15,
                Gearset::Blue,
                Direction::Reverse,
            )],
            // Grabber
            AdiDigitalOut::new(peripherals.adi_a),
            // Ejector
            AdiDigitalOut::new(peripherals.adi_g),
            // Lift
            AdiDigitalOut::new(peripherals.adi_d),
            // Hood low
            AdiDigitalOut::new(peripherals.adi_e),
            // Hood high
            AdiDigitalOut::new(peripherals.adi_b),
            // Color sorter
            OpticalSensor::new(peripherals.port_6),
        ),
        snacky: AdiDigitalOut::new(peripherals.adi_f),
    };

    // skills : c

    robot.compete().await;
}
