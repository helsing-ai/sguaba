use sguaba::{
    engineering::Orientation, math::RigidBodyTransform, system, systems::{Ecef, Wgs84}, Bearing, Coordinate,
};
use uom::si::f64::{Angle, Length};
use uom::si::{angle::degree, length::meter};

fn main() {
    // Calculate and confirm the position of a target given the following data
    // * Plane heading relative to the Plane NED (eg. compass)
    // * Pilot observation of target relative to Plane FRD (eg. range finder)
    // * Ground control observation of the plane relative to Ground ENU (eg. Radar)
    // * Ground control location as world coordinate (eg. GPS)

    let expected_plane_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.7068))
        .expect("latitude is in [-90º, 90º]")
        .longitude(Angle::new::<degree>(-78.7355))
        .altitude(Length::new::<meter>(17678.))
        .build();
    let expected_plane_ecef = Coordinate::<Ecef>::from_wgs84(&expected_plane_wgs84);

    let expected_target_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.597744245441966))
        .expect("latitude is in [-90º, 90º]")
        .longitude(Angle::new::<degree>(-78.82584635802755))
        .altitude(Length::new::<meter>(19075.36499))
        .build();
    let expected_target_ecef = Coordinate::<Ecef>::from_wgs84(&expected_target_wgs84);

    system!(struct PlaneNed using NED);  // Plane Instruments
    system!(struct PlaneFrd using FRD);  // Pilot Observation
    
    let plane_bearing: Bearing<PlaneNed> = 
        Bearing::builder()
            .azimuth(Angle::new::<degree>(214.74)) // clockwise from North
            .elevation(Angle::new::<degree>(0.)) // level with the horizon
            .expect("elevation is in [-90º, 90º]")
            .build();

    let target_bearing: Bearing<PlaneFrd> = 
        Bearing::builder()
            .azimuth(Angle::new::<degree>(0.)) // straight ahead
            .elevation(Angle::new::<degree>(5.342)) // above nose
            .expect("elevation is in [-90º, 90º]")
            .build();

    let target_range = Length::new::<meter>(14824.);

    println!("[FRANK01] Tally one balloon, heading {:.3}°, range of {:.3}m at elevation {:.1}°", 
             plane_bearing.azimuth().get::<degree>(), 
             target_range.get::<meter>(), 
             target_bearing.elevation().get::<degree>());

    system!(struct GroundEnu using ENU); // Ground Control Instruments

    let ground_control_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.6954))
        .expect("latitude is in [-90º, 90º]")
        .longitude(Angle::new::<degree>(-78.8802))
        .altitude(Length::new::<meter>(3.))
        .build();

    let ground_control_plane_observation = Coordinate::<GroundEnu>::from_bearing(
        Bearing::builder()
            .azimuth(Angle::new::<degree>(84.574)) // clockwise from North
            .elevation(Angle::new::<degree>(52.582)) // above the horizon
            .expect("elevation is in [-90º, 90º]")
            .build(),
            Length::new::<meter>(22236.3) // distance to plane
    );

    let transform_ecef_to_ground_enu = unsafe { RigidBodyTransform::ecef_to_enu_at(&ground_control_wgs84) };
    
    let plane_ecef = transform_ecef_to_ground_enu.inverse_transform(ground_control_plane_observation);
    assert_distance_tolerance(expected_plane_ecef, plane_ecef);

    println!("[GROUNDC] Radar confirmed aircraft at {}", &plane_ecef.to_wgs84());
    
    let transform_ecef_to_plane_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&plane_ecef.to_wgs84()) };
    
    let plane_orientation = Orientation::<PlaneNed>::from_tait_bryan_angles(
        plane_bearing.azimuth(),     // yaw from NED bearing
        plane_bearing.elevation(), // pitch from NED bearing
        Angle::new::<degree>(0.),   // roll
    );
    
    let transform_plane_ned_to_plane_frd = unsafe { plane_orientation.map_as_zero_in::<PlaneFrd>() };
    
    let transform_ecef_to_plane_frd = transform_ecef_to_plane_ned.and_then(transform_plane_ned_to_plane_frd);
    
    let pilot_observation = Coordinate::<PlaneFrd>::from_bearing(target_bearing, target_range);
    
    let target_ecef = transform_ecef_to_plane_frd.inverse_transform(pilot_observation);
    assert_distance_tolerance(expected_target_ecef, target_ecef);
    
    println!("[GROUNDC] Standby for visual inspection of target at {}", &target_ecef.to_wgs84());

    let target_ground_enu = transform_ecef_to_ground_enu.transform(expected_target_ecef);
    
    let calculated_bearing = target_ground_enu.bearing_from_origin()
        .expect("Target should not be at ground control origin");
    let calculated_distance = target_ground_enu.distance_from_origin();

    println!("[GROUNDC] Ground bearing to target: azimuth {:.3}°, elevation {:.3}°, range {:.1}m", 
             calculated_bearing.azimuth().get::<degree>(),
             calculated_bearing.elevation().get::<degree>(),
             calculated_distance.get::<meter>());

    let ground_control_target_observation = Coordinate::<GroundEnu>::from_bearing(
        calculated_bearing,
        calculated_distance
    );

    let target_ecef = transform_ecef_to_ground_enu.inverse_transform(ground_control_target_observation);    
    assert_distance_tolerance(expected_target_ecef, target_ecef);

    println!("[GROUNDC] Confirmed target locked at {}, cleared to engage", &target_ecef.to_wgs84());

    println!("[FRANK01] Copy, target confirmed and locked, weapons hot")
}

fn assert_distance_tolerance(expected_ecef: Coordinate<Ecef>, actual_ecef: Coordinate<Ecef>) {
    let distance_difference = (expected_ecef - actual_ecef).magnitude().get::<meter>();
    assert!(distance_difference < 1., "Position calculation error exceeds 1m tolerance: {}", distance_difference); // sanity check
}