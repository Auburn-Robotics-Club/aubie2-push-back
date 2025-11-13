use aubie2::{
    subsystems::{HoodPosition, Intake},
    theme::THEME_WAR_EAGLE,
};
use evian::{drivetrain::model::Differential, prelude::*};
use vexide::prelude::*;

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, ()>,
    intake: Intake<4, 1>,
    snacky: AdiDigitalOut,
}

impl Compete for Robot {
    async fn driver(&mut self) {
        _ = self.intake.grabber.set_high();

        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x());

            // Intake controls
            if state.button_b.is_pressed() || state.button_r1.is_pressed() {
                _ = self.intake.set_voltage(12.0);
            } else if state.button_down.is_pressed() {
                _ = self.intake.set_voltage(-12.0);
            } else {
                _ = self.intake.set_voltage(0.0);
            }

            // Grabber
            if state.button_r1.is_pressed() {
                if self.intake.grabber.is_low().unwrap_or_default() {
                    _ = self.intake.grabber.set_low();
                }
            } else {
                if self.intake.grabber.is_high().unwrap_or_default() {
                    _ = self.intake.grabber.set_high();
                }
            }

            if state.right_stick.y() > 0.70 {
                _ = self.intake.lift.set_low();
            } else if state.right_stick.y() < -0.70 {
                _ = self.intake.lift.set_high();
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
    let robot = Robot {
        controller: peripherals.primary_controller,
        drivetrain: Drivetrain::new(
            Differential::new(
                [
                    Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
                ],
                [
                    Motor::new(peripherals.port_14, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_13, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
                ],
            ),
            (),
        ),
        intake: Intake::new(
            [
                Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
            ],
            [Motor::new(
                peripherals.port_10,
                Gearset::Blue,
                Direction::Reverse,
            )],
            // Grabber
            AdiDigitalOut::new(peripherals.adi_f),
            // Ejector
            AdiDigitalOut::new(peripherals.adi_d),
            // Lift
            AdiDigitalOut::new(peripherals.adi_c),
            // Hood low
            AdiDigitalOut::new(peripherals.adi_e),
            // Hood high
            AdiDigitalOut::new(peripherals.adi_a),
            // Color sorter
            OpticalSensor::new(peripherals.port_6),
        ),
        snacky: AdiDigitalOut::new(peripherals.adi_b),
    };

    robot.compete().await;
}
