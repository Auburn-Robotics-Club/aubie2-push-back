use aubie2::{subsystems::Intake, theme::THEME_WAR_EAGLE};
use evian::{drivetrain::model::Differential, prelude::*};
use vexide::prelude::*;

struct Robot {
    controller: Controller,
    drivetrain: Drivetrain<Differential, ()>,
    intake: Intake<2, 1>,
    lift: AdiDigitalOut,
}

impl Compete for Robot {
    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x());

            if state.button_b.is_pressed() {
                _ = self.intake.set_voltage(12.0);
            } else if state.button_down.is_pressed() {
                _ = self.intake.set_voltage(-12.0);
            } else {
                _ = self.intake.set_voltage(0.0);
            }

            if state.button_a.is_now_pressed() {
                _ = self.intake.toggle_open();

                if !self.intake.is_open() {
                    _ = self.controller.rumble(".").await;
                } else {
                    _ = self.controller.rumble("..").await;
                }
            }

            if state.button_r1.is_now_pressed() {
                _ = self.lift.toggle();
            }

            sleep(Motor::WRITE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    let robot = Robot {
        controller: peripherals.primary_controller,
        lift: AdiDigitalOut::new(peripherals.adi_a),
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
                Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),
            ],
            [Motor::new(
                peripherals.port_10,
                Gearset::Blue,
                Direction::Reverse,
            )],
        ),
    };

    robot.compete().await;
}
