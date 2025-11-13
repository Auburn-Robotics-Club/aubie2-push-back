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

pub struct Intake<const BOTTOM_COUNT: usize, const TOP_COUNT: usize> {
    _task: Task<()>,
    reject_color: Rc<RefCell<Option<ElementColor>>>,
    bottom_motors: [Motor; BOTTOM_COUNT],
    top_motors: [Motor; TOP_COUNT],
    hood_high: AdiDigitalOut,
    hood_low: AdiDigitalOut,
    pub lift: AdiDigitalOut,
    pub grabber: AdiDigitalOut,
}

impl<const BOTTOM_COUNT: usize, const TOP_COUNT: usize> Intake<BOTTOM_COUNT, TOP_COUNT> {
    pub fn new(
        bottom_motors: [Motor; BOTTOM_COUNT],
        top_motors: [Motor; TOP_COUNT],
        grabber: AdiDigitalOut,
        ejector: AdiDigitalOut,
        lift: AdiDigitalOut,
        hood_low: AdiDigitalOut,
        hood_high: AdiDigitalOut,
        optical: OpticalSensor,
    ) -> Self {
        let reject_color = Rc::new(RefCell::new(None));

        Self {
            _task: spawn(Self::task(optical, reject_color.clone(), ejector)),
            reject_color,
            bottom_motors,
            top_motors,
            lift,
            hood_high,
            hood_low,
            grabber,
        }
    }

    async fn task(
        mut optical: OpticalSensor,
        reject_color: Rc<RefCell<Option<ElementColor>>>,
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
                    let is_seeing_red =
                        (10.0..40.0).contains(&hue) || (338.0..360.0).contains(&hue);

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

            if rejecting {
                if reject_timestamp.elapsed() < Duration::from_millis(1000) {
                    _ = ejector.set_high();
                } else {
                    rejecting = false;
                    _ = ejector.set_low();
                }
            }

            sleep(OpticalSensor::UPDATE_INTERVAL).await;
        }
    }

    pub fn set_reject_color(&mut self, reject_color: Option<ElementColor>) {
        *self.reject_color.borrow_mut() = reject_color;
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

        result
    }

    pub fn hood_position(&self) -> Result<HoodPosition, PortError> {
        Ok(
            match (self.hood_high.is_high()?, self.hood_low.is_high()?) {
                (true, true) | (true, false) => HoodPosition::High,
                (false, true) => HoodPosition::Half,
                (false, false) => HoodPosition::Closed,
            },
        )
    }

    pub fn set_voltage(&mut self, voltage: f64) -> Result<(), PortError> {
        self.set_top_voltage(
            if self.hood_position() == Ok(HoodPosition::Closed) && voltage.is_sign_positive() {
                0.0
            } else {
                voltage
            },
        )?;
        self.set_bottom_voltage(voltage)?;

        Ok(())
    }

    pub fn set_bottom_voltage(&mut self, voltage: f64) -> Result<(), PortError> {
        let mut rtn = Ok(());

        for motor in self.bottom_motors.iter_mut() {
            let result = motor.set_voltage(voltage * motor.max_voltage());

            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }

    pub fn set_top_voltage(&mut self, voltage: f64) -> Result<(), PortError> {
        let mut rtn = Ok(());

        for motor in self.top_motors.iter_mut() {
            let result = motor.set_voltage(voltage * motor.max_voltage());

            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }
}
