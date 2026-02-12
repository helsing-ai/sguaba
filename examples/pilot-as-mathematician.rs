use sguaba::{
    engineering::Orientation,
    math::RigidBodyTransform,
    system,
    systems::{Ecef, Wgs84},
    Bearing, Coordinate,
};
use uom::si::f64::{Angle, Length};
use uom::si::{angle::degree, length::meter};

fn main() {
    // FRD and NED systems are "local" coordinate systems, meaning a given
    // coordinate in the FRD of one plane will have a completely different
    // coordinate if it were to be expressed in the FRD of another. so, to
    // guard against accidentally getting them mixed up, we construct a new
    // type for this plane's FRD and NED:

    // the pilot observes things in FRD of the plane
    system!(struct PlaneFrd using FRD);

    // the pilot's instruments indicate the plane's orientation in NED
    system!(struct PlaneNed using NED);

    // what the pilot saw:
    let observation = Coordinate::<PlaneFrd>::from_bearing(
        Bearing::builder()
            // clockwise from forward
            .azimuth(Angle::new::<degree>(20.))
            // upwards from straight-ahead
            .elevation(Angle::new::<degree>(10.))
            .expect("elevation is in [-90°, 90°]")
            .build(),
        Length::new::<meter>(400.), // at this range
    );

    // where the plane was at the time (eg, from GPS):
    let wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(12.))
        .expect("latitude is in [-90°, 90°]")
        .longitude(Angle::new::<degree>(30.))
        .altitude(Length::new::<meter>(1000.))
        .build();

    // where the plane was facing at the time (eg, from instrument panel);
    // expressed in yaw, pitch, roll relative to North-East-Down:
    let orientation_in_ned = Orientation::<PlaneNed>::tait_bryan_builder()
        .yaw(Angle::new::<degree>(8.))
        .pitch(Angle::new::<degree>(45.))
        .roll(Angle::new::<degree>(0.))
        .build();

    // we need to find the ECEF<>NED transform for the plane's location
    // SAFETY: we're claiming that `wgs84` is the location of `PlaneNed`'s origin.
    let ecef_to_plane_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&wgs84) };
    // the plane's orientation in NED is really a rotation and translation in ECEF
    let pose_in_ecef = ecef_to_plane_ned * orientation_in_ned;
    // that rotation and translation is exactly equal to the FRD of the plane
    // we could also have just constructed this rotation directly instead of an `Orientation`
    // SAFETY: `PlaneNed` is the orientation of the plane's FRD body axes (ie, `PlaneFrd`).
    let ecef_to_frd = unsafe { pose_in_ecef.map_as_zero_in::<PlaneFrd>() };
    // and we can apply that transform to the original observation to get it in ECEF
    let observation_in_ecef: Coordinate<Ecef> = ecef_to_frd * observation;
    // which we can then turn into WGS84 lat/lon/altitude!
    println!("{:?}", observation_in_ecef.to_wgs84());
}
