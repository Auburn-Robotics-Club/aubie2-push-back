use evian::tracking::RotarySensor;
use vexide::{
    adi::{AdiPort, encoder::AdiEncoder},
    math::{Angle, Direction},
    smart::PortError,
};

pub type Amt102V = CustomEncoder<8192>;

pub struct CustomEncoder<const TPR: u32> {
    enc: AdiEncoder<TPR>,
    direction: Direction,
}

impl<const TPR: u32> CustomEncoder<TPR> {
    pub fn new(top_port: AdiPort, bottom_port: AdiPort, direction: Direction) -> Self {
        let enc = AdiEncoder::new(top_port, bottom_port);

        Self { enc, direction }
    }
}

impl<const TPR: u32> RotarySensor for CustomEncoder<TPR> {
    type Error = PortError;

    fn position(&self) -> Result<Angle, Self::Error> {
        Ok(self.enc.position()?
            * match self.direction {
                Direction::Forward => 1.0,
                Direction::Reverse => -1.0,
            })
    }
}
