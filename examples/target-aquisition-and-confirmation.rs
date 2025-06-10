use sguaba::{
    engineering::Orientation, math::RigidBodyTransform, system, systems::{Ecef, Wgs84}, Bearing, Coordinate, Vector,
};
use uom::si::f64::{Angle, Length};
use uom::si::{angle::degree, length::meter};

fn main() {
    
    let expected_plane_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.7068))
        .expect("latitude is in [-90º, 90º]")
        .longitude(Angle::new::<degree>(-78.7355))
        .altitude(Length::new::<meter>(17678.))
        .build();

    let expected_target_wgs84 = Wgs84::builder()
        .latitude(Angle::new::<degree>(33.597744245441966))
        .expect("latitude is in [-90º, 90º]")
        .longitude(Angle::new::<degree>(-78.82584635802755))
        .altitude(Length::new::<meter>(19075.36499))
        .build();

    system!(struct PlaneNed using NED);  // Plane Instruments
    system!(struct PilotFrd using FRD);  // Pilot Observation
    
    let plane_bearing: Bearing<PlaneNed> = 
        Bearing::builder()
            .azimuth(Angle::new::<degree>(214.74))
            .elevation(Angle::new::<degree>(0.))
            .expect("elevation is in [-90º, 90º]")
            .build();

    let target_bearing: Bearing<PilotFrd> = 
        Bearing::builder()
            .azimuth(Angle::new::<degree>(0.))
            .elevation(Angle::new::<degree>(5.342))
            .expect("elevation is in [-90º, 90º]")
            .build();

    let target_range = Length::new::<meter>(14824.);

    println!("[FRANK01] Tally one balloon, heading {:.2}°, range of {:.0}m at elevation {:.3}°", 
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
            .azimuth(Angle::new::<degree>(84.574))
            .elevation(Angle::new::<degree>(52.582))
            .expect("elevation is in [-90º, 90º]")
            .build(),
            Length::new::<meter>(22236.3)
    );

    let transform_ecef_to_ground_enu = unsafe { RigidBodyTransform::ecef_to_enu_at(&ground_control_wgs84) };
    
    let plane_ecef = transform_ecef_to_ground_enu.inverse_transform(ground_control_plane_observation);

    let expected_plane_ecef = Coordinate::<Ecef>::from_wgs84(&expected_plane_wgs84);
    let distance_difference = (expected_plane_ecef - plane_ecef).magnitude().get::<meter>();
    assert!(distance_difference < 1., "Position calculation error exceeds 1m tolerance: {}", distance_difference);

    println!("[GROUNDC] Aircraft confirmed at {}", &plane_ecef.to_wgs84());
    
    let transform_ecef_to_plane_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&plane_ecef.to_wgs84()) };
    
    let plane_orientation = Orientation::<PlaneNed>::from_tait_bryan_angles(
        plane_bearing.azimuth(),     // yaw from NED bearing
        plane_bearing.elevation(), // pitch from NED bearing
        Angle::new::<degree>(0.),   // roll
    );
    
    let transform_plane_ned_to_pilot_frd = unsafe { plane_orientation.map_as_zero_in::<PilotFrd>() };
    
    let transform_ecef_to_pilot_frd = transform_ecef_to_plane_ned.and_then(transform_plane_ned_to_pilot_frd);
    
    let pilot_observation = Coordinate::<PilotFrd>::from_bearing(target_bearing, target_range);
    
    let target_relative_vector = pilot_observation - Coordinate::<PilotFrd>::origin();
    let target_relative_ecef: Vector<Ecef> = transform_ecef_to_pilot_frd.inverse_transform(target_relative_vector);
    
    let target_ecef = plane_ecef + target_relative_ecef;
    
    let expected_target_ecef = Coordinate::<Ecef>::from_wgs84(&expected_target_wgs84);
    let target_distance_difference = (expected_target_ecef - target_ecef).magnitude().get::<meter>();
    assert!(target_distance_difference < 1., "Position calculation error exceeds 1m tolerance: {}", target_distance_difference);
    
    println!("[GROUNDC] Target confirmed at {}", &target_ecef.to_wgs84());

    // alternative

    let composite_transform_to_plane_ned = {
        let computed_plane_ecef = transform_ecef_to_ground_enu.inverse_transform(ground_control_plane_observation);
        
        let ecef_to_computed_plane_ned = unsafe { 
            RigidBodyTransform::ecef_to_ned_at(&computed_plane_ecef.to_wgs84()) 
        };
        
        ecef_to_computed_plane_ned
    };
    
    let transform_plane_ned_to_pilot_frd_chained = unsafe { plane_orientation.map_as_zero_in::<PilotFrd>() };
    let complete_chained_transform = composite_transform_to_plane_ned.and_then(transform_plane_ned_to_pilot_frd_chained);
    
    let target_relative_vector_chained = pilot_observation - Coordinate::<PilotFrd>::origin();
    let target_relative_ecef_chained: Vector<Ecef> = complete_chained_transform.inverse_transform(target_relative_vector_chained);
    
    let plane_ecef_chained = composite_transform_to_plane_ned.inverse_transform(Coordinate::<PlaneNed>::origin());
    let target_ecef_chained = plane_ecef_chained + target_relative_ecef_chained;
    
    let target_difference = (target_ecef - target_ecef_chained).magnitude().get::<meter>();
    assert!(target_difference < 1., "Position calculation error exceeds 1m tolerance: {}", target_difference);
    
    println!("[GROUNDC] Target confirmed (chained approach) at {}", &target_ecef_chained.to_wgs84());
}