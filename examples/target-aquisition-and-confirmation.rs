use approx::assert_relative_eq;
use sguaba::{
    Bearing, Coordinate,
    engineering::Orientation,
    math::RigidBodyTransform,
    system,
    systems::{Ecef, Wgs84},
};
use uom::si::f64::{Angle, Length};
use uom::si::{angle::degree, length::meter};

fn main() {
    // This example uses a handful of hard-coded values to demonstrate conversions
    // between NED, FRD, and ENU coordinate systems. Hard-coded values are meant to
    // emulate real-world instrument readings, and are derived from the the expected
    // plane and target WGS84 coordinates found at the bottom of this example. The
    // following "instrument readings" are used to confirm that both the plane and
    // ground control are looking at the same target:
    // * Plane heading relative to Plane NED (eg. compass)
    // * Pilot observation of the target relative to Plane FRD (eg. range finder)
    // * Ground control observation of the plane relative to Ground ENU (eg. radar)
    // * Ground control location as world coordinate (eg. GPS)

    system!(struct PlaneNed using NED); // Plane Instruments
    system!(struct PlaneFrd using FRD); // Pilot Observation
    system!(struct GroundEnu using ENU); // Ground Control Instruments

    // Value from the plane's onboard compass, manually picked to align with the actual target position (see bottom).
    let plane_bearing: Bearing<PlaneNed> = Bearing::builder()
        .azimuth(Angle::new::<degree>(204.74)) // clockwise from North
        .elevation(Angle::new::<degree>(0.)) // level with the horizon
        .expect("elevation is in [-90°, 90°]")
        .build();

    // Values from the pilot's range finder, manually picked to align with the actual target position (see bottom).
    let target_range = Length::new::<meter>(14824.);
    let target_bearing: Bearing<PlaneFrd> = Bearing::builder()
        .azimuth(Angle::new::<degree>(10.)) // clockwise from forward
        .elevation(Angle::new::<degree>(5.342)) // above nose
        .expect("elevation is in [-90°, 90°]")
        .build();

    // Value from the ground control's known location
    let ground_control_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.6954))
        .expect("latitude is in [-90°, 90°]")
        .longitude(Angle::new::<degree>(-78.8802))
        .altitude(Length::new::<meter>(3.))
        .build();

    // Where the ground control's radar sees the plane.
    // The values here were manually computed from the actual relative position of the plane to the ground control radar.
    let ground_control_plane_observation = Coordinate::<GroundEnu>::from_bearing(
        Bearing::builder()
            .azimuth(Angle::new::<degree>(84.574)) // clockwise from North
            .elevation(Angle::new::<degree>(52.582)) // above the horizon
            .expect("elevation is in [-90°, 90°]")
            .build(),
        Length::new::<meter>(22236.3), // distance to plane
    );

    println!(
        "[FRANK01] Tally one balloon, heading {:.3}°, range of {:.3}m at elevation {:.1}°",
        plane_bearing.azimuth().get::<degree>(),
        target_range.get::<meter>(),
        target_bearing.elevation().get::<degree>()
    );

    let transform_ecef_to_ground_enu =
        unsafe { RigidBodyTransform::ecef_to_enu_at(&ground_control_wgs84) };

    let plane_ecef =
        transform_ecef_to_ground_enu.inverse_transform(ground_control_plane_observation);

    println!(
        "[GROUNDC] Radar confirmed aircraft at {}",
        &plane_ecef.to_wgs84()
    );

    let transform_ecef_to_plane_ned =
        unsafe { RigidBodyTransform::ecef_to_ned_at(&plane_ecef.to_wgs84()) };

    let plane_orientation = Orientation::<PlaneNed>::tait_bryan_builder()
        .yaw(plane_bearing.azimuth())
        .pitch(plane_bearing.elevation())
        .roll(Angle::new::<degree>(0.))
        .build();

    let transform_plane_ned_to_plane_frd =
        unsafe { plane_orientation.map_as_zero_in::<PlaneFrd>() };

    let transform_ecef_to_plane_frd =
        transform_ecef_to_plane_ned.and_then(transform_plane_ned_to_plane_frd);

    let pilot_observation = Coordinate::<PlaneFrd>::from_bearing(target_bearing, target_range);

    let plane_target_ecef = transform_ecef_to_plane_frd.inverse_transform(pilot_observation);

    println!(
        "[GROUNDC] Standby for visual inspection of target at {}",
        &plane_target_ecef.to_wgs84()
    );

    let target_ground_enu = transform_ecef_to_ground_enu.transform(plane_target_ecef);

    let calculated_ground_bearing_to_target = target_ground_enu
        .bearing_from_origin()
        .expect("Target should not be at ground control origin");
    let calculated_ground_distance_to_target = target_ground_enu.distance_from_origin();

    println!(
        "[GROUNDC] Ground bearing to target: azimuth {:.3}°, elevation {:.3}°, range {:.1}m",
        calculated_ground_bearing_to_target
            .azimuth()
            .get::<degree>(),
        calculated_ground_bearing_to_target
            .elevation()
            .get::<degree>(),
        calculated_ground_distance_to_target.get::<meter>()
    );

    let ground_control_target_observation = Coordinate::<GroundEnu>::from_bearing(
        calculated_ground_bearing_to_target,
        calculated_ground_distance_to_target,
    );

    let ground_control_target_ecef =
        transform_ecef_to_ground_enu.inverse_transform(ground_control_target_observation);

    assert_distance_tolerance(ground_control_target_ecef, plane_target_ecef);

    println!(
        "[GROUNDC] Confirmed target locked at {}, cleared to engage",
        &ground_control_target_ecef.to_wgs84()
    );

    println!("[FRANK01] Copy, target confirmed and locked, weapons hot");

    // Sanity checking example against expected coordinates and manually calculated values
    let expected_plane_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.7068))
        .expect("latitude is in [-90°, 90°]")
        .longitude(Angle::new::<degree>(-78.7355))
        .altitude(Length::new::<meter>(17678.))
        .build();
    let expected_plane_ecef = Coordinate::<Ecef>::from_wgs84(&expected_plane_wgs84);

    let expected_target_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.597744245441966))
        .expect("latitude is in [-90°, 90°]")
        .longitude(Angle::new::<degree>(-78.82584635802755))
        .altitude(Length::new::<meter>(19075.36499))
        .build();
    let expected_target_ecef = Coordinate::<Ecef>::from_wgs84(&expected_target_wgs84);

    // These values were computed manually and committed to ensure no regression.
    let expected_ground_distance_to_target = Length::new::<meter>(22515.102);
    let expected_ground_bearing_to_target = Bearing::<GroundEnu>::builder()
        .azimuth(Angle::new::<degree>(155.02306))
        .elevation(Angle::new::<degree>(57.84293))
        .expect("elevation is in [-90°, 90°]")
        .build();

    println!(
        "\nChecking calculated ground bearing to target given expected ground bearing to target"
    );
    assert_relative_eq!(
        expected_ground_bearing_to_target,
        calculated_ground_bearing_to_target,
        epsilon = 0.001_f64.to_radians()
    );
    assert_relative_eq!(
        expected_ground_distance_to_target.get::<meter>(),
        calculated_ground_distance_to_target.get::<meter>(),
        epsilon = 1.0
    );

    println!("Checking calculated plane position given expected plane position");
    assert_distance_tolerance(expected_plane_ecef, plane_ecef);

    println!("Checking calculated plane's target position given expected target position");
    assert_distance_tolerance(expected_target_ecef, plane_target_ecef);

    println!("Checking calculated ground's target position given expected target position");
    assert_distance_tolerance(expected_target_ecef, ground_control_target_ecef);

    println!("\nAll checks passed");
}

fn assert_distance_tolerance(expected_ecef: Coordinate<Ecef>, actual_ecef: Coordinate<Ecef>) {
    let distance_difference = (expected_ecef - actual_ecef).magnitude().get::<meter>();
    assert!(
        distance_difference < 1.,
        "Position calculation error exceeds 1m tolerance: {}",
        distance_difference
    ); // sanity check
}
