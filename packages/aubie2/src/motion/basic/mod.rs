use evian::{
    control::loops::Feedback,
    math::Angle,
    motion::Basic,
    prelude::{
        Arcade, Drivetrain, TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity,
    },
};

use crate::motion::basic::cartesian_drive::CartesianDriveFuture;

pub(crate) mod cartesian_drive;

pub trait BasicExt<
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
>
{
    fn drive_to_x<
        'a,
        M: Arcade,
        T: TracksPosition + TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        x: f64,
        heading: Angle,
    ) -> CartesianDriveFuture<'a, M, L, A, T>;

    fn drive_to_y<
        'a,
        M: Arcade,
        T: TracksPosition + TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        y: f64,
        heading: Angle,
    ) -> CartesianDriveFuture<'a, M, L, A, T>;
}

impl<
    L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
    A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
> BasicExt<L, A> for Basic<L, A>
{
    fn drive_to_x<
        'a,
        M: Arcade,
        T: TracksPosition + TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        x: f64,
        heading: Angle,
    ) -> CartesianDriveFuture<'a, M, L, A, T> {
        CartesianDriveFuture {
            coordinate: cartesian_drive::Coordinate::X,
            target_coordinate: x,
            target_heading: heading,
            timeout: self.timeout,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
            drivetrain,
            state: None,
        }
    }

    fn drive_to_y<
        'a,
        M: Arcade,
        T: TracksPosition + TracksForwardTravel + TracksHeading + TracksVelocity,
    >(
        &mut self,
        drivetrain: &'a mut Drivetrain<M, T>,
        y: f64,
        heading: Angle,
    ) -> CartesianDriveFuture<'a, M, L, A, T> {
        CartesianDriveFuture {
            coordinate: cartesian_drive::Coordinate::Y,
            target_coordinate: y,
            target_heading: heading,
            timeout: self.timeout,
            linear_tolerances: self.linear_tolerances,
            angular_tolerances: self.angular_tolerances,
            linear_controller: self.linear_controller.clone(),
            angular_controller: self.angular_controller.clone(),
            drivetrain,
            state: None,
        }
    }
}
