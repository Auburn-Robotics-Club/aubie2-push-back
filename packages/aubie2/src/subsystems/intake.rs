use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use vexide::{
    adi::digital::LogicLevel,
    prelude::{AdiDigitalOut, OpticalSensor},
    smart::{PortError, SmartDevice, motor::Motor},
    task::{Task, spawn},
    time::sleep,
};

#[derive(Default, Debug, Clone, Copy, Eq, PartialEq)]
pub enum HoodPosition {
    High,
    Half,
    #[default]
    Closed,
}

/// Game element color
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum ElementColor {
    /// Reject blue rings
    Blue,

    /// Reject red rings
    Red,
}

bitflags::bitflags! {
    /// Intake stages for controlling different parts of the intake separately
    pub struct IntakeStage: u8 {
        const FRONT_BOTTOM = 1 << 0;
        const BACK_BOTTOM = 1 << 1;
        const BACK_TOP = 1 << 2;
        const FRONT_TOP = 1 << 3;
    }
}

pub struct Intake<
    const FRONT_BOTTOM_COUNT: usize,
    const BACK_BOTTOM_COUNT: usize,
    const BACK_TOP_COUNT: usize,
    const FRONT_TOP_COUNT: usize,
> {
    _task: Task<()>,
    reject_color: Rc<RefCell<Option<ElementColor>>>,
    emergency_override: Rc<RefCell<bool>>,

    pub front_bottom_motors: [Motor; FRONT_BOTTOM_COUNT],
    pub back_bottom_motors: [Motor; BACK_BOTTOM_COUNT],
    pub back_top_motors: [Motor; BACK_TOP_COUNT],
    pub front_top_motors: [Motor; FRONT_TOP_COUNT],

    hood_high: AdiDigitalOut,
    hood_low: AdiDigitalOut,
    pub lift: AdiDigitalOut,
    pub grabber: AdiDigitalOut,
    hood_position: HoodPosition,
}

impl<    const FRONT_BOTTOM_COUNT: usize,
const BACK_BOTTOM_COUNT: usize,
const BACK_TOP_COUNT: usize,
const FRONT_TOP_COUNT: usize,> Intake<FRONT_BOTTOM_COUNT, BACK_BOTTOM_COUNT, BACK_TOP_COUNT, FRONT_TOP_COUNT> {
    pub fn new(
        front_bottom_motors: [Motor; FRONT_BOTTOM_COUNT],
        back_bottom_motors: [Motor; BACK_BOTTOM_COUNT],
        back_top_motors: [Motor; BACK_TOP_COUNT],
        front_top_motors: [Motor; FRONT_TOP_COUNT],
        grabber: AdiDigitalOut,
        ejector: AdiDigitalOut,
        lift: AdiDigitalOut,
        hood_low: AdiDigitalOut,
        hood_high: AdiDigitalOut,
        optical: OpticalSensor,
    ) -> Self {
        let reject_color = Rc::new(RefCell::new(None));
        let emergency_override = Rc::new(RefCell::new(false));

        Self {
            _task: spawn(Self::task(optical, reject_color.clone(), emergency_override.clone(), ejector)),
            reject_color,
            front_bottom_motors,
            back_bottom_motors,
            back_top_motors,
            front_top_motors,
            lift,
            hood_high,
            hood_low,
            grabber,
            emergency_override,
            hood_position: HoodPosition::Closed,
        }
    }

    async fn task(
        mut optical: OpticalSensor,
        reject_color: Rc<RefCell<Option<ElementColor>>>,
        emergency_override: Rc<RefCell<bool>>,
        mut ejector: AdiDigitalOut,
    ) {
        _ = optical.set_integration_time(Duration::from_millis(2));
        _ = optical.set_led_brightness(1.0);

        let mut prox_timestamp = Instant::now();
        let mut reject_timestamp = Instant::now();
        let mut in_prox = false;
        let mut rejecting = false;

        loop {
            if let Some(reject_color) = *reject_color.borrow() {
                if let Ok(prox) = optical.proximity() {
                    if prox > 0.3 && !in_prox {
                        prox_timestamp = Instant::now();
                        in_prox = true;
                    }

                    if in_prox && prox_timestamp.elapsed() > Duration::from_millis(20) {
                        in_prox = false;
                    }
                }

                if let Ok(hue) = optical.hue()
                    && in_prox
                {
                    let is_seeing_blue = (200.0..250.0).contains(&hue);
                    let is_seeing_red = (0.0..20.0).contains(&hue) || (340.0..360.0).contains(&hue);

                    let (is_seeing_bad, is_seeing_good) = match reject_color {
                        ElementColor::Blue => (is_seeing_blue, is_seeing_red),
                        ElementColor::Red => (is_seeing_red, is_seeing_blue),
                    };

                    if is_seeing_bad {
                        reject_timestamp = Instant::now();
                        rejecting = true;
                    }

                    if is_seeing_good && rejecting {
                        rejecting = false;
                        _ = ejector.set_low();
                    }
                }
            }

            if *emergency_override.borrow() {
                if ejector.is_low().unwrap_or_default() {
                    _ = ejector.set_high();
                }
            } else {
                if rejecting {
                    if reject_timestamp.elapsed() < Duration::from_millis(1000) {
                        _ = ejector.set_high();
                    } else {
                        rejecting = false;
                        _ = ejector.set_low();
                    }
                } else {
                    if ejector.is_high().unwrap_or_default() {
                        _ = ejector.set_low();
                    }
                }
            }

            sleep(OpticalSensor::UPDATE_INTERVAL).await;
        }
    }

    pub fn set_reject_color(&mut self, reject_color: Option<ElementColor>) {
        *self.reject_color.borrow_mut() = reject_color;
    }

    pub fn reject_color(&self) -> Option<ElementColor> {
        *self.reject_color.borrow()
    }

    pub fn set_emergency_override(&mut self, ov: bool) {
        *self.emergency_override.borrow_mut() = ov;
    }

    pub fn set_hood_position(&mut self, position: HoodPosition) -> Result<(), PortError> {
        let mut result = Ok(());
        let (high_level, low_level) = match position {
            HoodPosition::Closed => (LogicLevel::Low, LogicLevel::Low),
            HoodPosition::Half => (LogicLevel::High, LogicLevel::Low),
            HoodPosition::High => (LogicLevel::Low, LogicLevel::High),
        };

        if let Err(error) = self.hood_low.set_level(high_level) {
            result = Err(error);
        }
        if let Err(error) = self.hood_high.set_level(low_level) {
            result = Err(error);
        }

        self.hood_position = position;

        result
    }

    pub fn hood_position(&self) -> HoodPosition {
        self.hood_position
    }

    pub fn set_voltage(&mut self, stage: IntakeStage, voltage: f64) -> Result<(), PortError> {
        let mut rtn = Ok(());

        if stage.contains(IntakeStage::FRONT_BOTTOM) {
            for motor in self.front_bottom_motors.iter_mut() {
                let result = motor.set_voltage(voltage);

                if result.is_err() {
                    rtn = result;
                }
            }
        }

        if stage.contains(IntakeStage::FRONT_TOP) {
            for motor in self.front_top_motors.iter_mut() {
                let result = motor.set_voltage(voltage);

                if result.is_err() {
                    rtn = result;
                }
            }
        }

        if stage.contains(IntakeStage::BACK_BOTTOM) {
            for motor in self.back_bottom_motors.iter_mut() {
                let result = motor.set_voltage(voltage);

                if result.is_err() {
                    rtn = result;
                }
            }
        }

        if stage.contains(IntakeStage::BACK_TOP) {
            for motor in self.back_top_motors.iter_mut() {
                let result = motor.set_voltage(voltage);

                if result.is_err() {
                    rtn = result;
                }
            }
        }

        rtn
    }
}
