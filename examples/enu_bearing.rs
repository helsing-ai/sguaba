use sguaba::{system, Bearing, Coordinate};
use uom::si::angle::degree;
use uom::si::f64::*;
use uom::si::length::meter;

system!(struct CameraEnu using ENU);

fn main() {
    assert_eq!(
        Coordinate::<CameraEnu>::builder()
            .enu_east(Length::new::<meter>(0.0))
            .enu_north(Length::new::<meter>(0.0))
            .enu_up(Length::new::<meter>(0.0))
            .build()
            .bearing_from_origin(),
        None
    );

    assert_eq!(
        Coordinate::<CameraEnu>::builder()
            .enu_east(Length::new::<meter>(0.0))
            .enu_north(Length::new::<meter>(1.0))
            .enu_up(Length::new::<meter>(0.0))
            .build()
            .bearing_from_origin(),
        Some(
            Bearing::<CameraEnu>::builder()
                .azimuth(Angle::new::<degree>(0.0))
                .elevation(Angle::new::<degree>(0.0))
                .expect("elevation on range [-90, 90]")
                .build()
        )
    );

    assert_eq!(
        Coordinate::<CameraEnu>::builder()
            .enu_east(Length::new::<meter>(1.0))
            .enu_north(Length::new::<meter>(0.0))
            .enu_up(Length::new::<meter>(0.0))
            .build()
            .bearing_from_origin(),
        Some(
            Bearing::<CameraEnu>::builder()
                .azimuth(Angle::new::<degree>(90.0))
                .elevation(Angle::new::<degree>(0.0))
                .expect("elevation on range [-90, 90]")
                .build()
        )
    );

    // Failing case.
    assert_eq!(
        Coordinate::<CameraEnu>::builder()
            .enu_east(Length::new::<meter>(0.0))
            .enu_north(Length::new::<meter>(0.0))
            .enu_up(Length::new::<meter>(1.0))
            .build()
            .bearing_from_origin(),
        Some(
            Bearing::<CameraEnu>::builder()
                // Actually returns an azimuth of PI / 2.0.
                .azimuth(Angle::new::<degree>(0.0))
                .elevation(Angle::new::<degree>(90.0))
                .expect("elevation on range [-90, 90]")
                .build()
        )
    );

    assert_eq!(
        Coordinate::<CameraEnu>::builder()
            .enu_east(Length::new::<meter>(0.0))
            .enu_north(Length::new::<meter>(0.0))
            .enu_up(Length::new::<meter>(-1.0))
            .build()
            .bearing_from_origin(),
        Some(
            Bearing::<CameraEnu>::builder()
                // Actually returns an azimuth of PI / 2.0.
                .azimuth(Angle::new::<degree>(0.0))
                .elevation(Angle::new::<degree>(-90.0))
                .expect("elevation on range [-90, 90]")
                .build()
        )
    );
}
