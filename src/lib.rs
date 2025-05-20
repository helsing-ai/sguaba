//! This library provides hard-to-misuse rigid body transforms (aka "spatial math") for engineers
//! with other things to worry about than linear algebra.
//!
//! First and foremost, the library provides [`Coordinate`] and [`Vector`] types for representing
//! points and vectors in coordinate spaces respectively. They are all generic over a
//! [`CoordinateSystem`] so that coordinates from one system cannot (easily) be incorrectly misused
//! as though they were in a different one. The [`system!`] macro allows you to define additional
//! coordinate systems with particular semantics (eg, [`NedLike`](systems::NedLike) or
//! [`FrdLike`](systems::FrdLike)) such that you can distinguish between coordinates in, say,
//! `PlaneFrd` and `EmitterFrd`.
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
//! # Examples
//!
//! Assume that a pilot of a plane observes something out of their window at a given bearing and
//! elevation angles (ie, measured in the plane's [FRD](systems::FrdLike)) and wants to know the
//! location of that thing in terms of Earth-bound Latitude and Longitude coordinates (ie,
//! [WGS84](systems::Wgs84).
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
//!     Bearing::new(
//!       Angle::new::<degree>(20.), // clockwise from forward
//!       Angle::new::<degree>(10.), // upwards from straight-ahead
//!     ).expect("elevation is in [-90, 90]"),
//!     Length::new::<meter>(400.), // at this range
//! );
//!
//! // where the plane was at the time (eg, from GPS):
//! let wgs84 = Wgs84::new(
//!     Angle::new::<degree>(12.),
//!     Angle::new::<degree>(30.),
//!     Length::new::<meter>(1000.)
//! ).expect("latitude is in [-90, 90]");
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
//! [`Ecef`](systems::Ecef) (cartesian world location) to [`Wgs84`](systems::Wgs84). Note that one
//! must know the observer's body orientation relative to NED to go from FRD to NED, and the
//! observer's ECEF position to go from NED to ECEF.
//!
//! ```
//! # use sguaba::{system, Bearing, Coordinate, engineering::Orientation, math::RigidBodyTransform, systems::Wgs84};
//! # use uom::si::f64::{Angle, Length};
//! # use uom::si::{angle::degree, length::meter};
//! # system!(struct PlaneFrd using FRD);
//! # system!(struct PlaneNed using NED);
//! # let observation = Coordinate::<PlaneFrd>::from_bearing(
//! #     Bearing::new(
//! #       Angle::new::<degree>(20.), // clockwise from forward
//! #       Angle::new::<degree>(10.), // upwards from straight-ahead
//! #     ).expect("elevation is in [-90, 90]"),
//! #     Length::new::<meter>(400.), // at this range
//! # );
//! # let wgs84 = Wgs84::new(
//! #     Angle::new::<degree>(12.),
//! #     Angle::new::<degree>(30.),
//! #     Length::new::<meter>(1000.)
//! # ).expect("latitude is in [-90, 90]");
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
//! # use sguaba::{system, Bearing, Coordinate, engineering::Orientation, math::RigidBodyTransform, systems::{Ecef, Wgs84}};
//! # use uom::si::f64::{Angle, Length};
//! # use uom::si::{angle::degree, length::meter};
//! # system!(struct PlaneFrd using FRD);
//! # system!(struct PlaneNed using NED);
//! # let observation = Coordinate::<PlaneFrd>::from_bearing(
//! #     Bearing::new(
//! #       Angle::new::<degree>(20.), // clockwise from forward
//! #       Angle::new::<degree>(10.), // upwards from straight-ahead
//! #     ).expect("elevation is in [-90, 90]"),
//! #     Length::new::<meter>(400.), // at this range
//! # );
//! # let wgs84 = Wgs84::new(
//! #     Angle::new::<degree>(12.),
//! #     Angle::new::<degree>(30.),
//! #     Length::new::<meter>(1000.)
//! # ).expect("latitude is in [-90, 90]");
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

#[macro_use]
mod coordinate_systems;

mod coordinates;
mod directions;
mod geodedic;
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

// re-structure our impots slightly to better match user expectation
/// Well-known coordinate systems and conventions.
pub mod systems {
    pub use super::coordinate_systems::{
        BearingDefined, Ecef, EquivalentTo, FrdLike, NedLike, RightHandedXyzLike,
    };
    pub use super::geodedic::Wgs84;
}
pub use coordinate_systems::CoordinateSystem;
pub use coordinates::Coordinate;
pub use directions::Bearing;
pub use vectors::Vector;
