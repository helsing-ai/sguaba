use crate::Bearing;
use uom::si::f64::{Angle, Length};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(doc)]
use crate::{Coordinate, Vector};

/// Defines how a coordinate system behaves.
///
/// For example, two coordinate systems that implement `CoordinateSystem<Convention = NedLike>`
/// both behave [`NedLike`], and get appropriately-named accessors on types like [`Coordinate`]
/// and [`Vector`] (ie, `north`, `east`, and `down`).
///
/// While you _can_ implement this trait directly, prefer using [`system!`](crate::system).
pub trait CoordinateSystem {
    /// What standard coordinate system convention this coordinate system conforms to.
    type Convention;
}

/// Links a coordinate system convention to the type holding the constituent parts under proper
/// names.
pub trait HasComponents {
    type Components: Into<[Length; 3]>;
}

/// Indicates that the implementing coordinate system is exactly equivalent to
/// `OtherCoordinateSystem`.
///
/// More technically, `impl EquivalentTo<B> for A` should only exist if the transform from `A` to
/// `B` is the identify function.
///
/// This is useful when you have two coordinate systems that you know have exactly equivalent axes,
/// and you need the types to "work out". This can be the case, for instance, when two crates have
/// both separately declared a NED-like coordinate system centered on the same object, and you need
/// to move types between them.
///
/// When an `impl EquivalentTo<B> for A` exists, a number of the types in this library provide a
/// `cast` method that allows directly changing the coordinate system type parameter without any
/// other transformation argument.
///
/// You will generally want to implement `EquivalentTo` for both `A` and `B` so that casts can
/// happen in either direction. This should always be safe since if the transform is identity from
/// `A` to `B`, it better also be from `B` to `A`.
///
/// # Safety
///
/// This trait is unsafe to implement because it allows moving between types annotated with two
/// different coordinate systems without performing any transform on them. If a transform actually
/// _is_ needed (and thus this implementation is incorrect), this would violate type safety.
pub unsafe trait EquivalentTo<OtherCoordinateSystem> {}

/// All coordinate systems are equivalent to themselves.
unsafe impl<System> EquivalentTo<System> for System {}

/// Defines the meaning of "bearing" (ie, azimuth and elevation) in a coordinate system.
///
/// For example, for [`NedLike`] and [`FrdLike`] coordinate systems:
///
/// - azimuth is the angle counterclockwise about positive Z along the XY plane from the positive X
///   axis (ie, North or Forward); and
/// - elevation is the angle towards _negative_ Z from the XY plane
///   (ie, "up").
///
/// Note that the "counterclockwise" above is around Z-axis as viewed from positive Z, which in
/// [`NedLike`] and [`FrdLike`] is _down_, so viewed from "above" positive azimuth will look
/// clockwise (as expected).
///
/// And for the [`EnuLike`] coordinate system:
///
/// - azimuth is the angle clockwise about positive Z along the XY plane from positive Y; and
/// - elevation is the angle towards positive Z from the XY plane.
pub trait BearingDefined: Sized {
    /// Returns the spherical-coordinate polar and azimuthal angles equivalent to a bearing.
    fn bearing_to_spherical(bearing: Bearing<Self>) -> (Angle, Angle);

    /// Returns the bearing azimuth and elevation equivalent to a spherical-coordinate polar and
    /// azimuthal angle.
    ///
    /// Should return `None` if `polar` is not in [0°, 180°).
    fn spherical_to_bearing(
        polar: impl Into<Angle>,
        azimuth: impl Into<Angle>,
    ) -> Option<Bearing<Self>>;
}

/// Marks an NED-like coordinate system where the axes are North, East, and Down.
///
/// NEDs are always right-handed coordinate systems, and have earth bounded axes:
///
/// - Positive X is North.
/// - Positive Y is East.
/// - Positive Z is towards the center of the earth ("Down").
///
/// Note that two NED-like coordinate systems may have different "absolute" Earth-bound coordinates
/// for their origin. For example, two different aircraft may both define their observations in
/// NED-like coordinate systems, but for each aircraft, (0, 0, 0) corresponds to the location of
/// that aircraft. Thus, an object observed at, say, (10, 20, 30) in one aircraft's local NED-like
/// coordinate system will have completely different coordinates in the other aircraft's local
/// NED-like coordinate system.
///
/// Since NED has earth bounded axes, two observers that are located in the same place but face in
/// different directions, they will still have the same NED-like coordinates to a given emitter.
///
/// [Bearing](BearingDefined) in NED-like coordinate systems are defined as:
///
/// - azimuth is the angle clockwise as seen from above along the horizontal plane from North; and
/// - elevation is the angle upwards from the horizontal plane.
///
/// <https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates>
pub struct NedLike;

/// Components for Cartesian points in an [`NedLike`] coordinate system.
///
/// Usually provided to methods like [`Coordinate::build`] or [`Vector::build`].
#[derive(Debug, Clone, Copy)]
#[must_use]
pub struct NedComponents {
    pub north: Length,
    pub east: Length,
    pub down: Length,
}

impl From<NedComponents> for [Length; 3] {
    fn from(c: NedComponents) -> [Length; 3] {
        [c.north, c.east, c.down]
    }
}

impl HasComponents for NedLike {
    type Components = NedComponents;
}

/// Marks an FRD-like coordinate system where the axes are Front (or "Forward"), Right, and Down.
///
/// FRDs are right-handed coordinate systems, and have observer bounded axes:
///
/// - Positive X is in the direction of travel of the observer (eg, aircraft).
/// - Positive Y is to the right of the observer's center of mass, parallel to the line that runs
///   wingtip to wingtip.
/// - Positive Z is along the line that runs through the belly of the plane.
///
/// Note that two FRD-like coordinate systems may have different "absolute" Earth-bound coordinates
/// for their origin. For example, two different aircraft may both define their observations in
/// FRD-like coordinate systems, but for each aircraft, (0, 0, 0) corresponds to the center of mass
/// of that aircraft. Thus, an object observed at, say, (10, 20, 30) in one aircraft's local
/// FRD-like coordinate system will have completely different coordinates in the other aircraft's
/// local FRD-like coordinate system.
///
/// Further note that, unlike [`NedLike`], two FRD-like systems with a colocated origin may also be
/// rotated with respect to each other. In other words, two observers that are located in the same
/// place but face in different directions will measure _different_ FRD-like coordinates to a given
/// emitter.
///
/// [Bearing](BearingDefined) in FRD-like coordinate systems are defined as:
///
/// - azimuth is the angle clockwise as seen from above along the horizontal plane from forward;
///   and
/// - elevation is the angle upwards from the horizontal plane.
///
/// <https://en.wikipedia.org/wiki/Aircraft_principal_axes>
pub struct FrdLike;

/// Components for Cartesian points in an [`FrdLike`] coordinate system.
///
/// Usually provided to methods like [`Coordinate::build`] or [`Vector::build`].
#[derive(Debug, Clone, Copy)]
#[must_use]
pub struct FrdComponents {
    pub front: Length,
    pub right: Length,
    pub down: Length,
}

impl From<FrdComponents> for [Length; 3] {
    fn from(c: FrdComponents) -> [Length; 3] {
        [c.front, c.right, c.down]
    }
}

impl HasComponents for FrdLike {
    type Components = FrdComponents;
}

/// Marks an ENU-like coordinate system where the axes are East, North, and Up.
///
/// ENUs are always right-handed coordinate systems, and have earth bounded axes:
///
/// - Positive X is East.
/// - Positive Y is North.
/// - Positive Z is away from the center of the earth ("Up").
///
/// Note that two ENU-like coordinate systems may have different "absolute" Earth-bound coordinates
/// for their origin. For example, two different aircraft may both define their observations in
/// ENU-like coordinate systems, but for each aircraft, (0, 0, 0) corresponds to the location of
/// that aircraft. Thus, an object observed at, say, (10, 20, 30) in one aircraft's local ENU-like
/// coordinate system will have completely different coordinates in the other aircraft's local
/// ENU-like coordinate system.
///
/// Since ENU has earth bounded axes, two observers that are located in the same place but face in
/// different directions, they will still have the same ENU-like coordinates to a given emitter.
///
/// [Bearing](BearingDefined) in ENU-like coordinate systems are defined as:
///
/// - azimuth is the angle clockwise as seen from above along the horizontal plane from North; and
/// - elevation is the angle upwards from the horizontal plane.
///
/// <https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_east,_north,_up_(ENU)_coordinates>
pub struct EnuLike;

/// Components for Cartesian points in an [`EnuLike`] coordinate system.
///
/// Usually provided to methods like [`Coordinate::build`] or [`Vector::build`].
#[derive(Debug, Clone, Copy)]
#[must_use]
pub struct EnuComponents {
    pub east: Length,
    pub north: Length,
    pub up: Length,
}

impl From<EnuComponents> for [Length; 3] {
    fn from(c: EnuComponents) -> [Length; 3] {
        [c.east, c.north, c.up]
    }
}

impl HasComponents for EnuLike {
    type Components = EnuComponents;
}

/// Marks a coordinate system whose axes are simply named X, Y, and Z.
///
/// Unlike [`NedLike`] and [`FrdLike`], there is no intrinsic relationship between XYZ-like
/// coordinate systems. While two XYZ-like coordinate systems _may_ share definitions of X, Y, and
/// Z, that shared meaning is not communicated through this type.
pub struct RightHandedXyzLike;

/// Components for Cartesian points in an coordinate system without specific names for X, Y, and Z,
/// like [`RightHandedXyzLike`].
///
/// Usually provided to methods like [`Coordinate::build`] or [`Vector::build`].
#[derive(Debug, Clone, Copy)]
#[must_use]
pub struct XyzComponents {
    pub x: Length,
    pub y: Length,
    pub z: Length,
}

impl From<XyzComponents> for [Length; 3] {
    fn from(c: XyzComponents) -> [Length; 3] {
        [c.x, c.y, c.z]
    }
}

impl HasComponents for RightHandedXyzLike {
    type Components = XyzComponents;
}

/// Defines a new coordinate system and its conventions.
///
/// Note that the coordinate system is a zero-sized type used only to mark things like
/// [`Coordinate`] and [`Vector`] with what coordinate system they are in. A coordinate system does
/// not know its relation to any other coordinate system (or global positions like WGS84).
///
/// At present, this macro allows you to define the following kinds of coordinate systems:
///
/// [`NedLike`]
///
/// ```rust
/// # use sguaba::system;
/// system!(pub struct SensorNed using NED);
/// ```
///
/// [`EnuLike`]
///
/// ```rust
/// # use sguaba::system;
/// system!(pub struct SensorEnu using ENU);
/// ```
///
/// [`FrdLike`]
///
/// ```rust
/// # use sguaba::system;
/// system!(pub struct EmitterFrd using FRD);
/// ```
///
/// [`RightHandedXyzLike`]
///
/// ```rust
/// # use sguaba::system;
/// system!(pub struct PlaneEcef using right-handed XYZ);
/// ```
///
/// You can include doc comments and attributes directly in the invocation of `system!` to add docs
/// and derived traits to your type:
///
/// ```rust
/// sguaba::system! {
///     #[derive(Hash)]
///     pub(crate) struct SensorNed using NED
/// }
/// ```
#[macro_export]
macro_rules! system {
    ($(#[$attr:meta])* $vis:vis struct $name:ident using right-handed XYZ) => {
        $crate::system!($(#[$attr])* $vis struct $name as RightHandedXyzLike);
    };
    ($(#[$attr:meta])* $vis:vis struct $name:ident using NED) => {
        $crate::system!($(#[$attr])* $vis struct $name as NedLike);
        $crate::system!(_clockwise_from_x_and_negative_z_bearing, $name);
    };
    ($(#[$attr:meta])* $vis:vis struct $name:ident using FRD) => {
        $crate::system!($(#[$attr])* $vis struct $name as FrdLike);
        $crate::system!(_clockwise_from_x_and_negative_z_bearing, $name);
    };
    ($(#[$attr:meta])* $vis:vis struct $name:ident using ENU) => {
        $crate::system!($(#[$attr])* $vis struct $name as EnuLike);
        $crate::system!(_clockwise_from_y_and_positive_z_bearing, $name);
    };
    (_clockwise_from_x_and_negative_z_bearing, $name:ident) => {
        /// For this coordinate system:
        ///
        /// - azimuth is the angle counterclockwise about negative Z along the XY plane from the
        ///   positive X axis; and
        /// - elevation is the angle towards _negative_ Z from the XY plane.
        impl $crate::systems::BearingDefined for $name {
            fn bearing_to_spherical(bearing: $crate::Bearing<Self>)
            -> (
                $crate::AngleForBearingTrait,
                $crate::AngleForBearingTrait
            ) {
                // elevation is just shifted by 90°; the polar angle is with respect to positive Z,
                // not XY plane, meaning an elevation of 0° is a polar angle of 90°, and an
                // elevation of 90° (which is towards negative Z) is a polar angle of 180°. note
                // that this _differs_ from the definion of elevation in spherical coordinates
                // (https://en.wikipedia.org/wiki/Spherical_coordinate_system#Definition), which is
                // defined as 90° - polar angle.
                let polar = bearing.elevation() + $crate::AngleForBearingTrait::HALF_TURN/2.;

                // azimuth is defined the same in spherical coordinates and bearing (both are angle
                // clockwise from positive X in the XY plane).
                let azimuth = bearing.azimuth();

                (polar, azimuth)
            }
            fn spherical_to_bearing(
                polar: impl Into<$crate::AngleForBearingTrait>,
                azimuth: impl Into<$crate::AngleForBearingTrait>
            ) -> Option<$crate::Bearing<Self>> {
                // just the inverse of the above
                let elevation = polar.into() - $crate::AngleForBearingTrait::HALF_TURN/2.;
                #[allow(clippy::redundant_locals)]
                let azimuth = azimuth.into();

                Some($crate::Bearing::builder().azimuth(azimuth).elevation(elevation)?.build())
            }
        }
    };
    (_clockwise_from_y_and_positive_z_bearing, $name:ident) => {
        /// For this coordinate system:
        ///
        /// - azimuth is the angle clockwise about positive Z along the XY plane from the
        ///   positive Y axis; and
        /// - elevation is the angle towards _positive_ Z from the XY plane.
        impl $crate::systems::BearingDefined for $name {
            fn bearing_to_spherical(bearing: $crate::Bearing<Self>)
            -> (
                $crate::AngleForBearingTrait,
                $crate::AngleForBearingTrait
            ) {
                // elevation is flipped and shifted by 90°; the polar angle is with respect to
                // positive Z, not XY plane, meaning an elevation of 0° is a polar angle of 90°,
                // and an elevation of 90° (which is towards positive Z) is a polar angle of 0°.
                let polar = $crate::AngleForBearingTrait::HALF_TURN/2. - bearing.elevation();

                // azimuth is flipped and shifted by 90°; in spherical coordinates azimuth increases
                // counterclockwise about positive Z along the XY place from the positive X axis.
                let azimuth = $crate::AngleForBearingTrait::HALF_TURN/2. - bearing.azimuth();

                (polar, azimuth)
            }
            fn spherical_to_bearing(
                polar: impl Into<$crate::AngleForBearingTrait>,
                azimuth: impl Into<$crate::AngleForBearingTrait>
            ) -> Option<$crate::Bearing<Self>> {
                // just the inverse of the above
                let elevation = $crate::AngleForBearingTrait::HALF_TURN/2. - polar.into();
                let azimuth = $crate::AngleForBearingTrait::HALF_TURN/2. - azimuth.into();

                Some($crate::Bearing::builder().azimuth(azimuth).elevation(elevation)?.build())
            }
        }
    };
    {
        $(#[$attr:meta])*
        $vis:vis struct $name:ident
        as $convention:ident
    } => {
        $(#[$attr])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        $vis struct $name;

        impl $crate::CoordinateSystem for $name {
            type Convention = $crate::systems::$convention;
        }
    };
}

system! {
    /// The [Earth-centered, Earth-fixed (ECEF)][ecef] coordinate system.
    ///
    /// This is a right-handed [`RightHandedXyzLike`] coordinate system whose origin is the center of the
    /// earth. It has earth bounded axes:
    ///
    /// - Positive Z is towards the North pole (the international reference pole (IRP)).
    /// - Positive X is towards the prime meridian on the equator (0° lon; the IERS Reference Meridian).
    /// - Positive Y is towards 90°E on the equator.
    ///
    /// This system has global coordinates; two observers with arbitrary position and orientation
    /// that name the same coordinate in this system are referring to the same absolute Earth-bound
    /// position.
    ///
    /// [ecef]: https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    #[allow(clippy::upper_case_acronyms)]
    pub struct Ecef using right-handed XYZ
}

system! {
    // this is just a test to make sure "using NED" works.
    // we expect consuming crates to define their own strongly typed versions of these systems that
    // correspond to particular frames of reference (eg, `PlaneNed` or `EmitterNed`).
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub(crate) struct Ned using NED
}

system! {
    // ditto for "using FRD"
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub(crate) struct Frd using FRD
}

system! {
    // ditto for "using ENU"
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub(crate) struct Enu using ENU
}
