//! This library provides hard-to-misuse rigid body transforms (aka "spatial math") for engineers
//! with other things to worry about than linear algebra.
//!
//! First and foremost, the library provides [`Coordinate`] and [`Vector`] types for representing
//! points and vectors in coordinate spaces respectively. They are all generic over a
//! [`CoordinateSystem`] so that coordinates from one system cannot (easily) be incorrectly misused
//! as though they were in a different one. The [`system!`] macro allows you to define additional
//! coordinate systems with particular semantics (eg, [`NedLike`] or [`FrdLike`]) such that you can
//! distinguish between coordinates in, say, `PlaneFrd` and `EmitterFrd`.
//!
//! To move between coordinate systems, you'll want to use the mathematical constructs from the
//! [`math`] submodule like [rigid body transforms](math::RigidBodyTransform) and
//! [rotations](math::Rotation).
//!
//! Now _technically_, those are all you need to express anything in coordinate system math,
//! including poses and orientation. It turns out that everything is an isometry if you think hard
//! enough about it. But if your brain is more used to thinking about orientation and poses (ie,
//! position + orientation), you'll want to make use of the [`engineering`] module which has
//! easier-to-grasp types like [`Pose`](engineering::Pose) and
//! [`Orientation`](engineering::Orientation).
//!
//! # Primer on coordinate systems
//!
//! If you're new to working with coordinate systems and frames of reference, you may wonder what
//! coordinate systems there are in the first place, and how they differ. There are a wide variety
//! of ways to describe the locations of objects in space, all of which have their own slight
//! peculiarities about representation and conversion. At the time of writing, the coordinate
//! systems this crate supports are: [WGS84] (latitude and longitude), [ECEF] ("Earth-centered,
//! Earth-fixed"), [NED] ("North, East, Down"), [FRD] ("Front, Right, Down"), and [ENU] ("East,
//! North, Up").
//!
//! [WGS84] ([`Wgs84`]) and [ECEF] ([`Ecef`]) are both Earth-bound
//! coordinate systems that describe points in space on or near Earth. They do this by describing
//! positions relative to Earth's major and minor axes, often by making slightly simplifying
//! assumptions about the Earth's shape. WGS84 does this by using latitude and longitude (degrees
//! north/south of the equator and east-west of the prime meridian), while ECEF does it by placing
//! a coordinate system at the center of the earth and locating [the X, Y, and Z axes][axes]
//! towards specific points on the Earth's surface. One can convert between them [without too much
//! trouble][trouble].
//!
//! [NED] ([`NedLike`]), [FRD] ([`FrdLike`]), and [ENU] ([`EnuLike`]) on the other hand are "local"
//! coordinate systems that are
//! descriptions of relative positions to the location of the observer. [NED] and [ENU] are
//! Earth-bound in that they describe positions in terms of how far North, East, and Down (for NED)
//! or East, North, and Up (for ENU) they are relative to the observer. [FRD], meanwhile, is a
//! "body frame", and just describes positions relative to the observer's concept of Forward (eg,
//! the direction pointing in the same direction as the nose of a plane), Right (eg, the direction
//! 90º to the right when viewing along Forward), and Down (eg, down through the belly of the
//! plane). Converting between [FRD] and [NED] or [ENU] usually requires knowing the orientation of
//! the observer relative to North, East, and Down/Up, and converting between [NED]/[ENU] and
//! [ECEF] (or [WGS84]) requires also knowing the position of the observer in Earth-bound
//! coordinates.
//!
//! [WGS84]: https://en.wikipedia.org/wiki/World_Geodetic_System#WGS84
//! [ECEF]: https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
//! [NED]: https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates
//! [ENU]: https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_east,_north,_up_(ENU)_coordinates
//! [FRD]: https://en.wikipedia.org/wiki/Body_relative_direction
//! [axes]: https://en.wikipedia.org/wiki/Axes_conventions
//! [trouble]: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#Coordinate_system_conversion
//!
//! # Use of unsafe
//!
//! Sguaba requires you to use `unsafe` in order to construct most transformations between
//! coordinate systems (eg,
//! [`RigidBodyTransform::ecef_to_ned_at`](math::RigidBodyTransform::ecef_to_ned_at) or
//! [`Orientation::map_as_zero_in`](engineering::Orientation::map_as_zero_in)). This is because
//! once one of these transforms have been constructed, they allow you to freely convert between
//! the _types_ representing each coordinate system. Thus, if a transform is constructed with
//! incorrect parameters, such as giving a coordinate to `ecef_to_ned_at` that does not correspond
//! to the location of the origin of the `To` [`NedLike`] system, _type_ safety
//! would be violated. This is a slight abuse of Rust's `unsafe` mechanism, which tends to focus on
//! memory safety, but has proven to be valuable in highlighting areas where frame of reference
//! bugs are most likely to manifest.
//!
//! # Temporal drift
//!
//! Sguaba does not attempt to provide time-variant type safety. For example, if you have a
//! `Coordinate<PlaneNed>` for the position of a moving plane, there are really an infinite number
//! of NED systems, one for each point along the plane's trajectory. Trivially, (0, 0, 0) in the
//! plane's NED frame at time t = 0 is not at the same real-world location as (0, 0, 0) in the
//! frame at time t = 1. But Sguaba does not let you express this in the type system as you'd end
//! up with an infinite number of coordinate system types. Instead, the expectation is that your
//! code ensures that it does not mix up frames of reference from different points in time. This is
//! unfortunate, and a potential source of errors, but not one with an obvious remedy.
//!
//! Perhaps surprisingly, this is _also_ the case for non-local frames like ECEF and WGS84. Due to
//! plate tectonics, a spot measured relative to a point on earth's _surface_ technically "moves"
//! over time. Consider, for example, the Eiffel tower, which is located at
//! 48.85851298170608ºN, 2.2944746521697468ºE (according to Google Maps at least). Since the
//! Eurasian plate drifts over time, if you measured its location a millennium from now, it would
//! technically be at a different place relative to, say, the Statue of Liberty. So, how does that
//! affect its latitude and longitude? The answer is that "it depends". There isn't really just
//! _one_ ECEF/WGS84, there are multiple, each defined through an [International Terrestrial
//! Reference Frame (ITRF)][ITRF], and over time, WGS84 [is "updated"][WGSup] to use a newer ITRF.
//! At the time of writing, WGS84 is "really" [G2296], which was released on 7 January 2024, which
//! is in turn aligned to [ITRF2020]. Sguaba's [`Ecef`] and [`Wgs84`] types do _not_ attempt to capture
//! this time-variance, just as the [`NedLike`], [`EnuLike`], and [`FrdLike`] systems do not
//! attempt to capture that these systems also change over time as the observer's location changes.
//!
//! [ITRF]: https://en.wikipedia.org/wiki/International_Terrestrial_Reference_System_and_Frame
//! [WGSup]: https://en.wikipedia.org/wiki/World_Geodetic_System#Updates_and_new_standards
//! [G2296]: https://earth-info.nga.mil/php/download.php?file=WGS%2084(G2296).pdf
//! [ITRF2020]: https://itrf.ign.fr/en/solutions/ITRF2020
//!
//! # Examples
//!
//! Assume that a pilot of a plane observes something out of their window at a given bearing and
//! elevation angles (ie, measured in the plane's [FRD](systems::FrdLike)) and wants to know the
//! location of that thing in terms of Earth-bound Latitude and Longitude coordinates (ie,
//! [WGS84](systems::Wgs84)).
//!
//! ```
//! # use sguaba::{system, Bearing, Coordinate, engineering::Orientation, systems::Wgs84};
//! use uom::si::f64::{Angle, Length};
//! use uom::si::{angle::degree, length::meter};
//!
//! // FRD and NED systems are "local" coordinate systems, meaning a given coordinate in the FRD of
//! // one plane will have a completely different coordinate if it were to be expressed in the FRD
//! // of another. so, to guard against accidentally getting them mixed up, we construct a new type
//! // for this plane's FRD and NED:
//!
//! // the pilot observes things in FRD of the plane
//! system!(struct PlaneFrd using FRD);
//!
//! // the pilot's instruments indicate the plane's orientation in NED
//! system!(struct PlaneNed using NED);
//!
//! // what the pilot saw:
//! let observation = Coordinate::<PlaneFrd>::from_bearing(
//!     Bearing::builder()
//!       // clockwise from forward
//!       .azimuth(Angle::new::<degree>(20.))
//!       // upwards from straight-ahead
//!       .elevation(Angle::new::<degree>(10.)).expect("elevation is in [-90, 90]")
//!       .build(),
//!     Length::new::<meter>(400.), // at this range
//! );
//!
//! // where the plane was at the time (eg, from GPS):
//! let wgs84 = Wgs84::builder()
//!     .latitude(Angle::new::<degree>(12.)).expect("latitude is in [-90, 90]")
//!     .longitude(Angle::new::<degree>(30.))
//!     .altitude(Length::new::<meter>(1000.))
//!     .build();
//!
//! // where the plane was facing at the time (eg, from instrument panel);
//! // expressed in yaw, pitch, roll relative to North-East-Down:
//! let orientation_in_ned = Orientation::<PlaneNed>::from_tait_bryan_angles(
//!     Angle::new::<degree>(8.),  // yaw
//!     Angle::new::<degree>(45.), // pitch
//!     Angle::new::<degree>(0.),  // roll
//! );
//! ```
//!
//! From there, there are two possible paths forward, one using an API that
//! will appeal more to folks with an engineering background, and one that
//! will appeal more to a math-oriented crowd. We'll explore each in turn.
//!
//! ## Using the engineering-focused API
//!
//! In the ["engineering-focused" API](engineering), we can directly talk about an object's
//! orientation and its "pose" (ie, position + orientation) in the world. Using these, we can
//! transform between different coordinate systems to go from `PlaneFrd` to `PlaneNed` to
//! [`Ecef`] (cartesian world location) to [`Wgs84`]. Note that one
//! must know the observer's body orientation relative to NED to go from FRD to NED, and the
//! observer's ECEF position to go from NED to ECEF.
//!
//! ```
//! # use sguaba::{system, Bearing, builder::{bearing::Components as BearingComponents, wgs84::Components as Wgs84Components}, Coordinate, engineering::Orientation, math::RigidBodyTransform, systems::Wgs84};
//! # use uom::si::f64::{Angle, Length};
//! # use uom::si::{angle::degree, length::meter};
//! # system!(struct PlaneFrd using FRD);
//! # system!(struct PlaneNed using NED);
//! # let observation = Coordinate::<PlaneFrd>::from_bearing(
//! #     Bearing::build(BearingComponents {
//! #       azimuth: Angle::new::<degree>(20.),
//! #       elevation: Angle::new::<degree>(10.),
//! #     }).expect("elevation is in [-90, 90]"),
//! #     Length::new::<meter>(400.), // at this range
//! # );
//! # let wgs84 = Wgs84::build(Wgs84Components {
//! #     latitude: Angle::new::<degree>(12.),
//! #     longitude: Angle::new::<degree>(30.),
//! #     altitude: Length::new::<meter>(1000.)
//! # }).expect("latitude is in [-90, 90]");
//! # let orientation_in_ned = Orientation::<PlaneNed>::from_tait_bryan_angles(
//! #     Angle::new::<degree>(8.),  // yaw
//! #     Angle::new::<degree>(45.), // pitch
//! #     Angle::new::<degree>(0.),  // roll
//! # );
//! #
//! // to convert between NED and ECEF, we need a transform between the two.
//! // this transform depends on where on the globe you are, so it takes the WGS84 position:
//! // SAFETY: we're claiming that `wgs84` is the location of `PlaneNed`'s origin.
//! let ecef_to_plane_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&wgs84) };
//!
//! // to convert between FRD (which the observation was made in) and NED,
//! // we just need the plane's orientation, which we have from the instruments!
//! // SAFETY: we're claiming that the given NED orientation makes up the axes of `PlaneFrd`.
//! let plane_ned_to_plane_frd = unsafe { orientation_in_ned.map_as_zero_in::<PlaneFrd>() };
//!
//! // these transformations can be chained to go from ECEF to NED.
//! // this chaining would fail to compile if you got the arguments wrong!
//! let ecef_to_plane_frd = ecef_to_plane_ned.and_then(plane_ned_to_plane_frd);
//!
//! // this transform lets you go from ECEF to FRD, but transforms work both ways,
//! // so we can apply it in inverse to take our `Coordinate<PlaneFrd>` and produce
//! // a `Coordinate<Ecef>`:
//! let observation_in_ecef = ecef_to_plane_frd.inverse_transform(observation);
//!
//! // we can then turn that into WGS84 lat/lon/altitude!
//! println!("{:?}", observation_in_ecef.to_wgs84());
//! ```
//!
//! ## Using the math-focused API
//!
//! In the ["math-focused" API](math), everything is represented in terms of transforms between
//! coordinate systems and the components of those transforms. For example:
//!
//! ```
//! # use sguaba::{system, Bearing, builder::{bearing::Components as BearingComponents, wgs84::Components as Wgs84Components}, Coordinate, engineering::Orientation, math::RigidBodyTransform, systems::{Ecef, Wgs84}};
//! # use uom::si::f64::{Angle, Length};
//! # use uom::si::{angle::degree, length::meter};
//! # system!(struct PlaneFrd using FRD);
//! # system!(struct PlaneNed using NED);
//! # let observation = Coordinate::<PlaneFrd>::from_bearing(
//! #     Bearing::build(BearingComponents {
//! #       azimuth: Angle::new::<degree>(20.),
//! #       elevation: Angle::new::<degree>(10.),
//! #     }).expect("elevation is in [-90, 90]"),
//! #     Length::new::<meter>(400.), // at this range
//! # );
//! # let wgs84 = Wgs84::build(Wgs84Components {
//! #     latitude: Angle::new::<degree>(12.),
//! #     longitude: Angle::new::<degree>(30.),
//! #     altitude: Length::new::<meter>(1000.)
//! # }).expect("latitude is in [-90, 90]");
//! # let orientation_in_ned = Orientation::<PlaneNed>::from_tait_bryan_angles(
//! #     Angle::new::<degree>(8.),  // yaw
//! #     Angle::new::<degree>(45.), // pitch
//! #     Angle::new::<degree>(0.),  // roll
//! # );
//! #
//! // we need to find the ECEF<>NED transform for the plane's location
//! // SAFETY: we're claiming that `wgs84` is the location of `PlaneNed`'s origin.
//! let ecef_to_plane_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&wgs84) };
//! // the plane's orientation in NED is really a rotation and translation in ECEF
//! let pose_in_ecef = ecef_to_plane_ned * orientation_in_ned;
//! // that rotation and translation is exactly equal to the FRD of the plane
//! // we could also have just constructed this rotation directly instead of an `Orientation`
//! // SAFETY: `PlaneNed` is the orientation of the plane's FRD body axes (ie, `PlaneFrd`).
//! let ecef_to_frd = unsafe { pose_in_ecef.map_as_zero_in::<PlaneFrd>() };
//! // and we can apply that transform to the original observation to get it in ECEF
//! let observation_in_ecef: Coordinate<Ecef> = ecef_to_frd * observation;
//! // which we can then turn into WGS84 lat/lon/altitude!
//! println!("{:?}", observation_in_ecef.to_wgs84());
//! ```

use typenum::{P1, Z0};
use uom::{
    si::{f64::V, Quantity, ISQ, SI},
    ConstZero, Kind,
};

// for easier linking from docs above
#[cfg(doc)]
use systems::{Ecef, EnuLike, FrdLike, NedLike, Wgs84};

#[macro_use]
mod coordinate_systems;

mod coordinates;
mod directions;
mod geodetic;
mod std_wrappers;
mod util;
mod vectors;

pub mod engineering;
pub mod math;

pub(crate) type Point3 = nalgebra::Point3<f64>;
pub(crate) type Vector3 = nalgebra::Vector3<f64>;
pub(crate) type Quaternion = nalgebra::Quaternion<f64>;
pub(crate) type UnitQuaternion = nalgebra::Unit<Quaternion>;
pub(crate) type Isometry3 = nalgebra::Isometry3<f64>;

#[doc(hidden)]
pub type AngleForBearingTrait = uom::si::f64::Angle;

#[doc(hidden)]
pub const ZERO_FOR_BEARING: AngleForBearingTrait = AngleForBearingTrait::ZERO;

/// Type alias for a quantity whose unit is one-dimensional in length and has a zero- or
/// negative-valued time dimension (ie, [`uom::si::f64::Length`], [`uom::si::f64::Velocity`], and
/// [`uom::si::f64::Acceleration`]).
///
/// Only used internally.
type LengthPossiblyPer<Time> = Quantity<ISQ<P1, Z0, Time, Z0, Z0, Z0, Z0, dyn Kind>, SI<V>, V>;

// re-structure our impots slightly to better match user expectation
/// Well-known coordinate systems and conventions.
pub mod systems {
    pub use super::coordinate_systems::{
        BearingDefined, Ecef, EnuLike, EquivalentTo, FrdLike, NedLike, RightHandedXyzLike,
    };
    pub use super::coordinate_systems::{
        EnuComponents, FrdComponents, HasComponents, NedComponents, XyzComponents,
    };
    pub use super::geodetic::Wgs84;
}
pub use coordinate_systems::CoordinateSystem;
pub use coordinates::Coordinate;
pub use directions::Bearing;
pub mod builder {
    /// Used to indicate that a partially-constructed value is missing a given component.
    #[derive(Debug, Clone, Copy)]
    pub struct Unset;

    /// Used to indicate that a partially-constructed value has a given component set.
    #[derive(Debug, Clone, Copy)]
    pub struct Set;

    pub mod vector {
        pub use crate::vectors::Builder;
    }
    pub mod coordinate {
        pub use crate::coordinates::Builder;
    }
    pub mod bearing {
        pub use crate::directions::{
            Builder, Components, HasAzimuth, HasElevation, MissingAzimuth, MissingElevation,
        };
    }
    pub mod wgs84 {
        pub use crate::geodetic::{
            Builder, Components, HasAltitude, HasLatitude, HasLongitude, MissingAltitude,
            MissingLatitude, MissingLongitude,
        };
    }
    pub mod tait_bryan {
        pub use crate::math::tait_bryan_builder::{
            Complete, NeedsPitch, NeedsRoll, NeedsYaw, TaitBryanBuilder,
        };
    }
}

/// Convenience re-exports for working with different vector units
pub mod vector {
    pub use crate::Vector;

    /// Type alias for a length vector (meters)
    pub type LengthVector<In> = Vector<In, typenum::Z0>;

    /// Type alias for a velocity vector (meters per second)
    pub type VelocityVector<In> = Vector<In, typenum::N1>;

    /// Type alias for an acceleration vector (meters per second squared)
    pub type AccelerationVector<In> = Vector<In, typenum::N2>;
}

pub use vectors::Vector;
