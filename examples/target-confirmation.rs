use sguaba::{
    engineering::Orientation, math::RigidBodyTransform, system, systems::Wgs84, Bearing, Coordinate,
};
use uom::si::f64::{Angle, Length};
use uom::si::{angle::degree, length::meter};

fn main() {

    // the pilot's instruments indicate the plane's orientation in NED
    system!(struct PlaneNed using NED);

    // the ground observer's instruments indicate their orientation in ENU
    system!(struct GroundEnu using ENU);

    // the pilot observes things in FRD of the plane
    system!(struct PlaneFrd using FRD);

    // the ground observer observes things in their own FRD system
    system!(struct GroundFrd using FRD);

    // where the pilot is (eg, from GPS)
    let pilot_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.7068))
        .expect("latitude is in [-90º, 90º]")
        .longitude(Angle::new::<degree>(-78.7355))
        .altitude(Length::new::<meter>(17678.))
        .build();

    // where the ground observer is (eg, from GPS)
    let ground_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.6954))
        .expect("latitude is in [-90º, 90º]")
        .longitude(Angle::new::<degree>(-78.8802))
        .altitude(Length::new::<meter>(3.))
        .build();

    // where the plane was facing at the time (eg, from instrument panel);
    // expressed in yaw, pitch, roll relative to North-East-Down,
    // the plane is facing directly towards the target
    let pilot_orientation_in_ned = Orientation::<PlaneNed>::from_tait_bryan_angles(
        Angle::new::<degree>(214.74),  // yaw
        Angle::new::<degree>(5.342), // pitch
        Angle::new::<degree>(0.),     // roll
    );

    // where the ground observer was facing at the time (eg, with a range finder)
    // expressed in yaw, pitch, roll relative to East-North-Up,
    // the ground observer is facing directly towards the target
    let ground_orientation_in_enu = Orientation::<GroundEnu>::from_tait_bryan_angles(
        Angle::new::<degree>(90. - 155.023), // yaw, ENU azimuth is 155.023 from N
        Angle::new::<degree>(-57.842),     // pitch, ENU elevation is 57.842 from horizon
        Angle::new::<degree>(0.),           // roll
    );

    // what the pilot saw (looking straight ahead in their FRD system)
    let pilot_observation = Coordinate::<PlaneFrd>::from_bearing(
        Bearing::builder()
            .azimuth(Angle::new::<degree>(0.))
            .elevation(Angle::new::<degree>(0.))
            .expect("elevation is in [-90º, 90º]")
            .build(),
        Length::new::<meter>(14824.),
    );

    // what the ground observer saw (also looking straight ahead in their FRD system)
    let ground_observation = Coordinate::<GroundFrd>::from_bearing(
        Bearing::builder()
            .azimuth(Angle::new::<degree>(0.))
            .elevation(Angle::new::<degree>(0.))
            .expect("elevation is in [-90º, 90º]")
            .build(),
        Length::new::<meter>(22515.1),
    );

    // to convert between NED and ECEF, we need a transform between the two.
    // this transform depends on where on the globe you are, so it takes the WGS84 position:
    // SAFETY: we're claiming that `wgs84` is the location of `PlaneNed`'s origin.
    let ecef_to_plane_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&pilot_wgs84) };

    // to convert between FRD (which the observation was made in) and NED,
    // we just need the plane's orientation, which we have from the instruments!
    // SAFETY: we're claiming that the given NED orientation makes up the axes of `PlaneFrd`.
    let plane_ned_to_plane_frd = unsafe { pilot_orientation_in_ned.map_as_zero_in::<PlaneFrd>() };

    // these transformations can be chained to go from ECEF to FRD.
    // this chaining would fail to compile if you got the arguments wrong!
    let ecef_to_plane_frd = ecef_to_plane_ned.and_then(plane_ned_to_plane_frd);

    // this transform lets you go from ECEF to FRD, but transforms work both ways,
    // so we can apply it in inverse to take our `Coordinate<PlaneFrd>` and produce
    // a `Coordinate<Ecef>`:
    let pilot_target_in_ecef = ecef_to_plane_frd.inverse_transform(pilot_observation);

    println!("Pilot  observes the balloon at {}", &pilot_target_in_ecef.to_wgs84());

    // similar conversion between ENU and ECEF.
    // SAFETY: we're claiming that `wgs84` is the location of `GroundEnu`'s origin.
    let ecef_to_ground_enu = unsafe { RigidBodyTransform::ecef_to_enu_at(&ground_wgs84) };

    // similar conversion between FRD and ENU
    // SAFETY: we're claiming that the given ENU orientation makes up the axes of `GroundFrd`
    let ground_enu_to_ground_frd = unsafe { ground_orientation_in_enu.map_as_zero_in::<GroundFrd>() };
    
    let ecef_to_ground_frd = ecef_to_ground_enu.and_then(ground_enu_to_ground_frd);
    
    // similar transform between `Coordinate<GroundFrd>` and `Coordinate<Ecef>`
    let ground_target_in_ecef = ecef_to_ground_frd.inverse_transform(ground_observation);

    println!("Ground observes the balloon at {}", &ground_target_in_ecef.to_wgs84());
    
    let distance_difference = (pilot_target_in_ecef - ground_target_in_ecef).magnitude();
    println!("Distance difference between pilot and ground balloon locations: {:.1}m", 
        distance_difference.get::<meter>());
}
