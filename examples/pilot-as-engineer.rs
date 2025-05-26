use sguaba::{
    engineering::Orientation, math::RigidBodyTransform, system, systems::Wgs84, Bearing, Coordinate,
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
            .expect("elevation is in [-90º, 90º]")
            .build(),
        Length::new::<meter>(400.), // at this range
    );

    // where the plane was at the time (eg, from GPS):
    let wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(12.))
        .expect("latitude is in [-90º, 90º]")
        .longitude(Angle::new::<degree>(30.))
        .altitude(Length::new::<meter>(1000.))
        .build();

    // where the plane was facing at the time (eg, from instrument panel);
    // expressed in yaw, pitch, roll relative to North-East-Down:
    let orientation_in_ned = Orientation::<PlaneNed>::from_tait_bryan_angles(
        Angle::new::<degree>(8.),  // yaw
        Angle::new::<degree>(45.), // pitch
        Angle::new::<degree>(0.),  // roll
    );

    // to convert between NED and ECEF, we need a transform between the two.
    // this transform depends on where on the globe you are, so it takes the WGS84 position:
    // SAFETY: we're claiming that `wgs84` is the location of `PlaneNed`'s origin.
    let ecef_to_plane_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&wgs84) };

    // to convert between FRD (which the observation was made in) and NED,
    // we just need the plane's orientation, which we have from the instruments!
    // SAFETY: we're claiming that the given NED orientation makes up the axes of `PlaneFrd`.
    let plane_ned_to_plane_frd = unsafe { orientation_in_ned.map_as_zero_in::<PlaneFrd>() };

    // these transformations can be chained to go from ECEF to NED.
    // this chaining would fail to compile if you got the arguments wrong!
    let ecef_to_plane_frd = ecef_to_plane_ned.and_then(plane_ned_to_plane_frd);

    // this transform lets you go from ECEF to FRD, but transforms work both ways,
    // so we can apply it in inverse to take our `Coordinate<PlaneFrd>` and produce
    // a `Coordinate<Ecef>`:
    let observation_in_ecef = ecef_to_plane_frd.inverse_transform(observation);

    // we can then turn that into WGS84 lat/lon/altitude!
    println!("{:?}", observation_in_ecef.to_wgs84());
}
