use crate::{Bearing, LengthPossiblyPer};
use typenum::Z0;
use uom::si::f64::Angle;
use uom::si::angle::radian;
use std::f64::consts::PI;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(doc)]
use crate::{Coordinate, Vector};

/// Defines how a coordinate system behaves.
pub trait CoordinateSystem {
    /// What standard coordinate system convention this coordinate system conforms to.
    type Convention;
}

/// Links a coordinate system convention to the type holding the constituent parts under proper names.
pub trait HasComponents<Time = Z0>
where
    Time: typenum::Integer,
{
    type Components: Into<[LengthPossiblyPer<Time>; 3]>;
}

/// Indicates that the implementing coordinate system is exactly equivalent to another.
pub unsafe trait EquivalentTo<OtherCoordinateSystem> {}

/// All coordinate systems are equivalent to themselves.
unsafe impl<System> EquivalentTo<System> for System {}

/// Defines the meaning of "bearing" (ie, azimuth and elevation) in a coordinate system.
pub trait BearingDefined: Sized {
    /// Returns the spherical-coordinate polar and azimuthal angles equivalent to a bearing.
    fn bearing_to_spherical(bearing: Bearing<Self>) -> (Angle, Angle);

    /// Returns the bearing azimuth and elevation equivalent to spherical coordinates.
    fn spherical_to_bearing(
        polar: impl Into<Angle>,
        azimuth: impl Into<Angle>,
    ) -> Option<Bearing<Self>>;
}

/// Marks an NED-like coordinate system where the axes are North, East, and Down.
pub struct NedLike;

/// Components for Cartesian points in an [`NedLike`] coordinate system.
#[derive(Debug, Clone, Copy)]
#[must_use]
pub struct NedComponents<Time = Z0>
where
    Time: typenum::Integer,
{
    pub north: LengthPossiblyPer<Time>,
    pub east: LengthPossiblyPer<Time>,
    pub down: LengthPossiblyPer<Time>,
}

impl<Time: typenum::Integer> From<NedComponents<Time>> for [LengthPossiblyPer<Time>; 3] {
    fn from(c: NedComponents<Time>) -> [LengthPossiblyPer<Time>; 3] {
        [c.north, c.east, c.down]
    }
}

impl<Time: typenum::Integer> HasComponents<Time> for NedLike {
    type Components = NedComponents<Time>;
}

/// Marks an FRD-like coordinate system where the axes are Front, Right, and Down.
pub struct FrdLike;

/// Components for Cartesian points in an [`FrdLike`] coordinate system.
#[derive(Debug, Clone, Copy)]
#[must_use]
pub struct FrdComponents<Time = Z0>
where
    Time: typenum::Integer,
{
    pub front: LengthPossiblyPer<Time>,
    pub right: LengthPossiblyPer<Time>,
    pub down: LengthPossiblyPer<Time>,
}

impl<Time: typenum::Integer> From<FrdComponents<Time>> for [LengthPossiblyPer<Time>; 3] {
    fn from(c: FrdComponents<Time>) -> [LengthPossiblyPer<Time>; 3] {
        [c.front, c.right, c.down]
    }
}

impl<Time: typenum::Integer> HasComponents<Time> for FrdLike {
    type Components = FrdComponents<Time>;
}

/// Marks an ENU-like coordinate system where the axes are East, North, and Up.
pub struct EnuLike;

/// Components for Cartesian points in an [`EnuLike`] coordinate system.
#[derive(Debug, Clone, Copy)]
#[must_use]
pub struct EnuComponents<Time = Z0>
where
    Time: typenum::Integer,
{
    pub east: LengthPossiblyPer<Time>,
    pub north: LengthPossiblyPer<Time>,
    pub up: LengthPossiblyPer<Time>,
}

impl<Time: typenum::Integer> From<EnuComponents<Time>> for [LengthPossiblyPer<Time>; 3] {
    fn from(c: EnuComponents<Time>) -> [LengthPossiblyPer<Time>; 3] {
        [c.east, c.north, c.up]
    }
}

impl<Time: typenum::Integer> HasComponents<Time> for EnuLike {
    type Components = EnuComponents<Time>;
}

/// Marks a coordinate system whose axes are simply named X, Y, and Z.
pub struct RightHandedXyzLike;

/// Components for Cartesian points in an XYZ coordinate system.
#[derive(Debug, Clone, Copy)]
#[must_use]
pub struct XyzComponents<Time = Z0>
where
    Time: typenum::Integer,
{
    pub x: LengthPossiblyPer<Time>,
    pub y: LengthPossiblyPer<Time>,
    pub z: LengthPossiblyPer<Time>,
}

impl<Time: typenum::Integer> From<XyzComponents<Time>> for [LengthPossiblyPer<Time>; 3] {
    fn from(c: XyzComponents<Time>) -> [LengthPossiblyPer<Time>; 3] {
        [c.x, c.y, c.z]
    }
}

impl<Time: typenum::Integer> HasComponents<Time> for RightHandedXyzLike {
    type Components = XyzComponents<Time>;
}

/// Marks an ICRS-like (International Celestial Reference System) coordinate system.
#[allow(dead_code)]
pub struct IcrsLike;

/// Components for Cartesian points in an [`IcrsLike`] coordinate system.
#[derive(Debug, Clone, Copy)]
#[must_use]
#[allow(dead_code)]
pub struct IcrsComponents<Time = Z0>
where
    Time: typenum::Integer,
{
    pub x: LengthPossiblyPer<Time>,  // Towards vernal equinox (J2000)
    pub y: LengthPossiblyPer<Time>,  // 90° east of X in celestial equator plane
    pub z: LengthPossiblyPer<Time>,  // Towards north celestial pole
}

impl<Time: typenum::Integer> From<IcrsComponents<Time>> for [LengthPossiblyPer<Time>; 3] {
    fn from(c: IcrsComponents<Time>) -> [LengthPossiblyPer<Time>; 3] {
        [c.x, c.y, c.z]
    }
}

impl<Time: typenum::Integer> HasComponents<Time> for IcrsLike {
    type Components = IcrsComponents<Time>;
}

impl BearingDefined for IcrsLike {
    /// Converts ICRS bearing (Right Ascension, Declination) to spherical coordinates
    fn bearing_to_spherical(bearing: Bearing<Self>) -> (Angle, Angle) {
        let ra = bearing.azimuth();
        let dec = bearing.elevation();
        
        let polar_rad = PI/2.0 - dec.get::<radian>();
        let polar = Angle::new::<radian>(polar_rad);
        
        (polar, ra)
    }

    /// Converts spherical coordinates to ICRS bearing
    fn spherical_to_bearing(
        polar: impl Into<Angle>,
        azimuth: impl Into<Angle>,
    ) -> Option<Bearing<Self>> {
        let polar = polar.into();
        let ra = azimuth.into();
        let polar_rad = polar.get::<radian>();
        
        if polar_rad < 0.0 || polar_rad > PI {
            return None;
        }
        
        let dec_rad = PI/2.0 - polar_rad;
        let dec = Angle::new::<radian>(dec_rad);
        
        Some(Bearing::builder()
            .azimuth(ra)
            .elevation(dec)?
            .build())
    }
}

/// Marks a Moon-centered inertial coordinate system.
#[allow(dead_code)]
pub struct MoonCenteredLike;

/// Components for Cartesian points in a [`MoonCenteredLike`] coordinate system.
#[derive(Debug, Clone, Copy)]
#[must_use]
#[allow(dead_code)]
pub struct MoonCenteredComponents<Time = Z0>
where
    Time: typenum::Integer,
{
    pub x: LengthPossiblyPer<Time>,  // Towards Earth
    pub y: LengthPossiblyPer<Time>,  // East in lunar equator plane
    pub z: LengthPossiblyPer<Time>,  // Towards Moon's north pole
}

impl<Time: typenum::Integer> From<MoonCenteredComponents<Time>> for [LengthPossiblyPer<Time>; 3] {
    fn from(c: MoonCenteredComponents<Time>) -> [LengthPossiblyPer<Time>; 3] {
        [c.x, c.y, c.z]
    }
}

impl<Time: typenum::Integer> HasComponents<Time> for MoonCenteredLike {
    type Components = MoonCenteredComponents<Time>;
}

impl BearingDefined for MoonCenteredLike {
    /// Converts Moon-centered bearing to spherical coordinates
    fn bearing_to_spherical(bearing: Bearing<Self>) -> (Angle, Angle) {
        let azimuth = bearing.azimuth();
        let elevation = bearing.elevation();
        
        let polar_rad = PI/2.0 - elevation.get::<radian>();
        let polar = Angle::new::<radian>(polar_rad);
        
        (polar, azimuth)
    }

    /// Converts spherical coordinates to Moon-centered bearing
    fn spherical_to_bearing(
        polar: impl Into<Angle>,
        azimuth: impl Into<Angle>,
    ) -> Option<Bearing<Self>> {
        let polar = polar.into();
        let az = azimuth.into();
        let polar_rad = polar.get::<radian>();
        
        if polar_rad < 0.0 || polar_rad > PI {
            return None;
        }
        
        let elevation_rad = PI/2.0 - polar_rad;
        let elevation = Angle::new::<radian>(elevation_rad);
        
        Some(Bearing::builder()
            .azimuth(az)
            .elevation(elevation)?
            .build())
    }
}

/// Defines a new coordinate system and its conventions.
#[macro_export]
macro_rules! system {
    ($(#[$attr:meta])* $vis:vis struct $name:ident using right-handed XYZ) => {
        $crate::system!($(#[$attr])* $vis struct $name as $crate::systems::RightHandedXyzLike);
    };
    ($(#[$attr:meta])* $vis:vis struct $name:ident using NED) => {
        $crate::system!($(#[$attr])* $vis struct $name as $crate::systems::NedLike);
        $crate::system!(_clockwise_from_x_and_negative_z_bearing, $name);
    };
    ($(#[$attr:meta])* $vis:vis struct $name:ident using FRD) => {
        $crate::system!($(#[$attr])* $vis struct $name as $crate::systems::FrdLike);
        $crate::system!(_clockwise_from_x_and_negative_z_bearing, $name);
    };
    ($(#[$attr:meta])* $vis:vis struct $name:ident using ENU) => {
        $crate::system!($(#[$attr])* $vis struct $name as $crate::systems::EnuLike);
        $crate::system!(_clockwise_from_y_and_positive_z_bearing, $name);
    };
    ($(#[$attr:meta])* $vis:vis struct $name:ident using ICRS) => {
        $crate::system!($(#[$attr])* $vis struct $name as $crate::systems::IcrsLike);
        $crate::system!(_icrs_bearing, $name);
    };
    ($(#[$attr:meta])* $vis:vis struct $name:ident using MoonCentered) => {
        $crate::system!($(#[$attr])* $vis struct $name as $crate::systems::MoonCenteredLike);
        $crate::system!(_moon_centered_bearing, $name);
    };
    (_clockwise_from_x_and_negative_z_bearing, $name:ident) => {
        impl $crate::systems::BearingDefined for $name {
            fn bearing_to_spherical(bearing: $crate::Bearing<Self>) -> (uom::si::f64::Angle, uom::si::f64::Angle) {
                let polar = bearing.elevation() + uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0);
                let azimuth = bearing.azimuth();
                (polar, azimuth)
            }
            fn spherical_to_bearing(
                polar: impl Into<uom::si::f64::Angle>,
                azimuth: impl Into<uom::si::f64::Angle>
            ) -> Option<$crate::Bearing<Self>> {
                let elevation = polar.into() - uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0);
                let azimuth = azimuth.into();
                Some($crate::Bearing::builder().azimuth(azimuth).elevation(elevation)?.build())
            }
        }
    };
    (_clockwise_from_y_and_positive_z_bearing, $name:ident) => {
        impl $crate::systems::BearingDefined for $name {
            fn bearing_to_spherical(bearing: $crate::Bearing<Self>) -> (uom::si::f64::Angle, uom::si::f64::Angle) {
                let polar = uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0) - bearing.elevation();
                let azimuth = uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0) - bearing.azimuth();
                (polar, azimuth)
            }
            fn spherical_to_bearing(
                polar: impl Into<uom::si::f64::Angle>,
                azimuth: impl Into<uom::si::f64::Angle>
            ) -> Option<$crate::Bearing<Self>> {
                let elevation = uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0) - polar.into();
                let mut azimuth = uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0) - azimuth.into();

                if elevation.abs() == uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0) {
                    azimuth = uom::si::f64::Angle::new::<uom::si::angle::radian>(0.0);
                }

                Some($crate::Bearing::builder().azimuth(azimuth).elevation(elevation)?.build())
            }
        }
    };
    (_icrs_bearing, $name:ident) => {
        impl $crate::systems::BearingDefined for $name {
            fn bearing_to_spherical(bearing: $crate::Bearing<Self>) -> (uom::si::f64::Angle, uom::si::f64::Angle) {
                let ra = bearing.azimuth();
                let dec = bearing.elevation();
                let polar = uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0) - dec;
                (polar, ra)
            }
            
            fn spherical_to_bearing(
                polar: impl Into<uom::si::f64::Angle>,
                azimuth: impl Into<uom::si::f64::Angle>
            ) -> Option<$crate::Bearing<Self>> {
                let polar = polar.into();
                let ra = azimuth.into();
                
                if polar < uom::si::f64::Angle::new::<uom::si::angle::radian>(0.0) || polar > uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI) {
                    return None;
                }
                
                let dec = uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0) - polar;
                
                Some($crate::Bearing::builder()
                    .azimuth(ra)
                    .elevation(dec)
                    ?.build())
            }
        }
    };
    (_moon_centered_bearing, $name:ident) => {
        impl $crate::systems::BearingDefined for $name {
            fn bearing_to_spherical(bearing: $crate::Bearing<Self>) -> (uom::si::f64::Angle, uom::si::f64::Angle) {
                let azimuth = bearing.azimuth();
                let elevation = bearing.elevation();
                let polar = uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0) - elevation;
                (polar, azimuth)
            }
            
            fn spherical_to_bearing(
                polar: impl Into<uom::si::f64::Angle>,
                azimuth: impl Into<uom::si::f64::Angle>
            ) -> Option<$crate::Bearing<Self>> {
                let polar = polar.into();
                let az = azimuth.into();
                
                if polar < uom::si::f64::Angle::new::<uom::si::angle::radian>(0.0) || polar > uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI) {
                    return None;
                }
                
                let elevation = uom::si::f64::Angle::new::<uom::si::angle::radian>(::std::f64::consts::PI / 2.0) - polar;
                
                Some($crate::Bearing::builder()
                    .azimuth(az)
                    .elevation(elevation)
                    ?.build())
            }
        }
    };
    {
        $(#[$attr:meta])*
        $vis:vis struct $name:ident
        as $convention:path
    } => {
        $(#[$attr])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        $vis struct $name;

        impl $crate::CoordinateSystem for $name {
            type Convention = $convention;
        }
    };
}

system! {
    /// The Earth-centered, Earth-fixed (ECEF) coordinate system.
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    #[allow(clippy::upper_case_acronyms)]
    pub struct Ecef using right-handed XYZ
}

#[cfg(test)]
mod testing {
    use super::*;

    system! {
        #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
        pub(crate) struct Ned using NED
    }

    system! {
        #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
        pub(crate) struct Frd using FRD
    }

    system! {
        #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
        pub(crate) struct Enu using ENU
    }

    system! {
        #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
        #[allow(dead_code)]
        pub(crate) struct TestIcrs using ICRS
    }

    system! {
        #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
        #[allow(dead_code)]
        pub(crate) struct TestMoonFrame using MoonCentered
    }
}

#[cfg(test)]
#[allow(clippy::items_after_test_module)]
pub(crate) use testing::*;