//! Feedback-driven driving and turning.

use std::time::Duration;

use evian::{control::loops::Feedback, math::Angle, prelude::{Arcade, Drivetrain, Tolerances, TracksForwardTravel, TracksHeading, TracksVelocity}};
use vexide::prelude::DistanceSensor;

use crate::motion::distance_sensor::future::DistanceDriveFuture;

mod future;

/// Feedback-driven driving and turning.
#[derive(PartialEq)]
pub struct DistanceSensorDriving<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
{
    /// Linear (forward driving) feedback controller.
    pub linear_controller: L,

    /// Angular (turning) feedback controller.
    pub angular_controller: A,

    /// Linear settling conditions.
    pub linear_tolerances: Tolerances,

    /// Angular settling conditions.
    pub angular_tolerances: Tolerances,

    /// Maximum duration the motion can take before being cancelled.
    pub timeout: Option<Duration>,
}

impl<L, A> DistanceSensorDriving<L, A>
where
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
{
    /// Moves the robot forwards by a given distance (measured in wheel units) while
    /// turning to face a heading.
    ///
    /// Negative `target_distance` values will move the robot backwards.
    pub fn drive_to_distance<
        'a,
        M: Arcade,
        T: TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        sensor: &'a DistanceSensor,
        target_distance: f64,
        target_heading: Angle,
    ) -> DistanceDriveFuture<'a, M, L, A, T> {
        DistanceDriveFuture {
            target_distance,
            target_heading,
            timeout: self.timeout,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
            sensor,
            drivetrain,
            state: None,
        }
    }
}
