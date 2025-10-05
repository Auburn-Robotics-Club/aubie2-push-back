use vexide::devices::{
    smart::motor::{Motor, MotorError},
    PortError,
};

pub struct Intake<const BOTTOM_COUNT: usize, const TOP_COUNT: usize> {
    bottom_motors: [Motor; BOTTOM_COUNT],
    top_motors: [Motor; TOP_COUNT],
    is_raised: bool,
    is_open: bool,
}

impl<const BOTTOM_COUNT: usize, const TOP_COUNT: usize> Intake<BOTTOM_COUNT, TOP_COUNT> {
    pub const fn new(bottom_motors: [Motor; BOTTOM_COUNT], top_motors: [Motor; TOP_COUNT]) -> Self {
        Self {
            bottom_motors,
            top_motors,
            is_raised: false,
            is_open: true,
        }
    }

    pub fn open(&mut self) -> Result<(), PortError> {
        self.is_open = true;
        Ok(())
    }

    pub fn close(&mut self) -> Result<(), PortError> {
        self.is_open = false;
        Ok(())
    }

    pub fn is_open(&self) -> bool {
        self.is_open
    }

    pub fn toggle_open(&mut self) -> Result<(), PortError> {
        self.is_open = !self.is_open;
        Ok(())
    }

    pub fn raise(&mut self) -> Result<(), PortError> {
        self.is_raised = true;
        Ok(())
    }

    pub fn lower(&mut self) -> Result<(), PortError> {
        self.is_raised = false;

        Ok(())
    }

    pub fn toggle_lift(&mut self) -> Result<(), PortError> {
        self.is_raised = !self.is_raised;
        Ok(())
    }

    pub fn set_voltage(&mut self, voltage: f64) -> Result<(), MotorError> {
        self.set_top_voltage(if !self.is_open && voltage.is_sign_positive() {
            0.0
        } else {
            voltage
        })?;
        self.set_bottom_voltage(voltage)?;

        Ok(())
    }

    pub fn set_bottom_voltage(&mut self, voltage: f64) -> Result<(), MotorError> {
        let mut rtn = Ok(());

        for motor in self.bottom_motors.iter_mut() {
            let result = motor.set_voltage(voltage * motor.max_voltage());

            if result.is_err() {
                rtn = result;
            }
        }

        rtn
    }

    pub fn set_top_voltage(&mut self, voltage: f64) -> Result<(), MotorError> {
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
