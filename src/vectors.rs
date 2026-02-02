use crate::builder::{Set, Unset};
use crate::coordinate_systems::HasComponents;
use crate::directions::Bearing;
use crate::engineering::Orientation;
use crate::{
    systems::{EnuLike, EquivalentTo, FrdLike, NedLike, RightHandedXyzLike},
    Coordinate, CoordinateSystem,
};
use crate::{LengthPossiblyPer, Vector3};
use std::fmt::{Display, Formatter};
use std::marker::PhantomData;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};
use std::{fmt, iter::Sum};
use typenum::{Integer, N1, N2, P2, Z0};
use uom::si::f64::{Acceleration, Angle, Length, Velocity};
use uom::si::{acceleration::meter_per_second_squared, length::meter, velocity::meter_per_second};
use uom::ConstZero;

#[cfg(any(test, feature = "approx"))]
use {
    crate::Point3,
    approx::{AbsDiffEq, RelativeEq},
};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(doc)]
use crate::{
    math::RigidBodyTransform,
    systems::BearingDefined,
    vector::{AccelerationVector, LengthVector, VelocityVector},
};

/// Defines a vector (ie, direction with magnitude) in the coordinate system specified by `In`.
///
/// You can construct one using [Cartesian](Vector::build) or
/// [spherical](Vector::from_spherical) components, or using [bearing +
/// range](Vector::from_bearing). You can also use the [`vector!`](crate::vector!) macro
/// for a concise constructor with named arguments.
///
/// Depending on the convention of the coordinate system (eg, [`NedLike`], [`FrdLike`], or
/// [`RightHandedXyzLike`]), you'll have different appropriately-named accessors for the vector's
/// cartesian components like [`Vector::ned_north`] or [`Vector::frd_front`].
///
/// # Working with different `Vector` units
///
/// sguaba supports vectors representing different physical quantities:
///
/// - **Length vectors** (`Vector<In>`, `Vector<In, Z0>`, or [`LengthVector`]) for positions and displacements
/// - **Velocity vectors** (`Vector<In, N1>` or [`VelocityVector`]) for velocities
/// - **Acceleration vectors** (`Vector<In, N2>` or [`AccelerationVector`]) for accelerations
///
/// All vector operations (addition, scaling, transforms) are supported for all units.
///
/// ## Length `Vector`
/// ```rust
/// # use sguaba::{system, vector, Vector};
/// # use uom::si::f64::Length;
/// # use uom::si::length::meter;
/// # system!(struct PlaneFrd using FRD);
///
/// let displacement = vector!(
///     f = Length::new::<meter>(10.0),
///     r = Length::new::<meter>(5.0),
///     d = Length::new::<meter>(0.0);
///     in PlaneFrd
/// );
/// ```
///
/// ## Velocity `Vector`
/// ```rust
/// # use sguaba::{system, vector, Vector, vector::VelocityVector};
/// # use uom::si::f64::Velocity;
/// # use uom::si::velocity::meter_per_second;
/// # system!(struct PlaneFrd using FRD);
///
/// let velocity = vector!(
///     f = Velocity::new::<meter_per_second>(100.0), // forward
///     r = Velocity::new::<meter_per_second>(0.0),   // right
///     d = Velocity::new::<meter_per_second>(5.0);   // down
///     in PlaneFrd
/// );
/// ```
///
/// ## Acceleration `Vector`
/// ```rust
/// # use sguaba::{system, vector, Vector, vector::AccelerationVector};
/// # use uom::si::f64::Acceleration;
/// # use uom::si::acceleration::meter_per_second_squared;
/// # system!(struct PlaneFrd using FRD);
/// # system!(struct PlaneNed using NED);
///
/// // Commanded acceleration (climb has negative d in FRD)
/// let acceleration = vector!(
///     f = Acceleration::new::<meter_per_second_squared>(2.0),  // speed up
///     r = Acceleration::new::<meter_per_second_squared>(0.0),
///     d = Acceleration::new::<meter_per_second_squared>(-1.0); // climb
///     in PlaneFrd
/// );
///
/// // Can use other acceleration units, such as standard gravity
/// let gravity = vector!(
///     n = Acceleration::new::<meter_per_second_squared>(0.0),
///     e = Acceleration::new::<meter_per_second_squared>(0.0),
///     d = Acceleration::new::<uom::si::acceleration::standard_gravity>(1.);
///     in PlaneNed
/// );
/// ```
///
/// ## Type safety with units
///
/// The type system prevents mixing incompatible units:
///
/// ```compile_fail
/// # use sguaba::{system, vector};
/// # use uom::si::f64::{Length, Velocity};
/// # use uom::si::{length::meter, velocity::meter_per_second};
/// # system!(struct PlaneFrd using FRD);
///
/// // Cannot put meters in a velocity vector - this will not compile
/// let bad_velocity = vector!(
///     f = Length::new::<meter>(10.0), // Length, not Velocity!
///     r = Velocity::new::<meter_per_second>(5.0),
///     d = Velocity::new::<meter_per_second>(0.0);
///     in PlaneFrd
/// );
/// ```
///
/// ```compile_fail
/// # use sguaba::{system, vector, Vector, vector::{VelocityVector, AccelerationVector}};
/// # use uom::si::f64::{Velocity, Acceleration};
/// # use uom::si::{velocity::meter_per_second, acceleration::meter_per_second_squared};
/// # system!(struct PlaneFrd using FRD);
///
/// // Cannot add length and acceleration vectors - this will not compile
/// let velocity = vector!(
///     f = Velocity::new::<meter_per_second>(10.0),
///     r = Velocity::new::<meter_per_second>(0.0),
///     d = Velocity::new::<meter_per_second>(0.0);
///     in PlaneFrd
/// );
/// let acceleration = vector!(
///     f = Acceleration::new::<meter_per_second_squared>(2.0),
///     r = Acceleration::new::<meter_per_second_squared>(0.0),
///     d = Acceleration::new::<meter_per_second_squared>(0.0);
///     in PlaneFrd
/// );
/// let bad_sum = velocity + acceleration; // Different Time parameters!
/// ```
///
/// Only [`LengthVector`] can be created from [`Coordinate`], as coordinates are inherently just
/// describing positions in a coordinate system, which are displacements (ie, lengths) from origin:
///
/// ```compile_fail
/// # use sguaba::{system, coordinate, vector, vector::{LengthVector, VelocityVector}};
/// # use uom::si::f64::{Length, Velocity};
/// # use uom::si::{length::meter, velocity::meter_per_second};
/// # system!(struct PlaneFrd using FRD);
///
/// let coordinate = coordinate!(
///     f = Length::new::<meter>(10.0),
///     r = Length::new::<meter>(5.0),
///     d = Length::new::<meter>(0.0);
///     in PlaneFrd
/// );
///
/// // This compiles fine
/// let _: LengthVector<PlaneFrd> = coordinate.into();
///
/// // This will not compile as that `impl From` does not exist
/// let _: VelocityVector<PlaneFrd> = coordinate.into();
/// ```
///
/// <div class="warning">
///
/// Note that this type implements `Deserialize` despite having `unsafe` constructors -- this is
/// because doing otherwise would be extremely unergonomic. However, when deserializing, the
/// coordinate system of the deserialized value is _not_ checked, so this is a foot-gun to be
/// mindful of.
///
/// </div>
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
// don't require In: Serialize/Deserialize since we skip it anyway
#[cfg_attr(feature = "serde", serde(bound = ""))]
// no need for the "inner": indirection
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct Vector<In, Time = Z0>
where
    Time: Integer,
{
    /// X, Y, Z in meters
    pub(crate) inner: Vector3,
    #[cfg_attr(feature = "serde", serde(skip))]
    system: PhantomData<In>,
    #[cfg_attr(feature = "serde", serde(skip))]
    unit: PhantomData<LengthPossiblyPer<Time>>,
}

/// Trait used to access a [`Vector`]'s Cartesian components in its native units.
///
/// Only used to abstract over `Time`; should never be used in user code.
///
/// This trait is sealed, and so also cannot be implemented by user code.
#[doc(hidden)]
pub trait LengthBasedComponents<In, Time>: private::Sealed
where
    Time: Integer,
{
    /// Directly construct from constituent components.
    fn from_cartesian(xyz: [LengthPossiblyPer<Time>; 3]) -> Self;
    /// Deconstruct into constituent components.
    fn to_cartesian(&self) -> [LengthPossiblyPer<Time>; 3];

    /// Cast constituent component unit to [`Length`].
    ///
    /// This is entirely intended for _internal_ use, such as to allow conversion to and from
    /// [`Coordinate`] (which is only ever [`Length`]-unit). You should strongly avoid using this
    /// directly, as it allows units to be confused.
    // NOTE(jon): while it's tempting to just re-assign the .value of the various uom types that
    // involve length, we're not _guaranteed_ that uom's inner representation for length, velocity,
    // and acceleration is the same, so we have to do the whole dance and expect it to be elided by
    // the compiler.
    fn recast_to_length(v: LengthPossiblyPer<Time>) -> Length;
}

impl<In> LengthBasedComponents<In, Z0> for Vector<In, Z0> {
    fn from_cartesian([x, y, z]: [Length; 3]) -> Self {
        Self::from_nalgebra_vector(Vector3::new(
            x.get::<meter>(),
            y.get::<meter>(),
            z.get::<meter>(),
        ))
    }
    fn to_cartesian(&self) -> [Length; 3] {
        [
            Length::new::<meter>(self.inner.x),
            Length::new::<meter>(self.inner.y),
            Length::new::<meter>(self.inner.z),
        ]
    }
    fn recast_to_length(v: LengthPossiblyPer<Z0>) -> Length {
        std::convert::identity(v)
    }
}

impl<In> LengthBasedComponents<In, N1> for Vector<In, N1> {
    fn from_cartesian([x, y, z]: [Velocity; 3]) -> Self {
        Self::from_nalgebra_vector(Vector3::new(
            x.get::<meter_per_second>(),
            y.get::<meter_per_second>(),
            z.get::<meter_per_second>(),
        ))
    }
    fn to_cartesian(&self) -> [Velocity; 3] {
        [
            Velocity::new::<meter_per_second>(self.inner.x),
            Velocity::new::<meter_per_second>(self.inner.y),
            Velocity::new::<meter_per_second>(self.inner.z),
        ]
    }
    fn recast_to_length(v: LengthPossiblyPer<N1>) -> Length {
        Length::new::<meter>(v.get::<meter_per_second>())
    }
}

impl<In> LengthBasedComponents<In, N2> for Vector<In, N2> {
    fn from_cartesian([x, y, z]: [Acceleration; 3]) -> Self {
        Self::from_nalgebra_vector(Vector3::new(
            x.get::<meter_per_second_squared>(),
            y.get::<meter_per_second_squared>(),
            z.get::<meter_per_second_squared>(),
        ))
    }
    fn to_cartesian(&self) -> [Acceleration; 3] {
        [
            Acceleration::new::<meter_per_second_squared>(self.inner.x),
            Acceleration::new::<meter_per_second_squared>(self.inner.y),
            Acceleration::new::<meter_per_second_squared>(self.inner.z),
        ]
    }
    fn recast_to_length(v: LengthPossiblyPer<N2>) -> Length {
        Length::new::<meter>(v.get::<meter_per_second_squared>())
    }
}

mod private {
    pub trait Sealed {}

    impl<In> Sealed for super::Vector<In, typenum::Z0> {}
    impl<In> Sealed for super::Vector<In, typenum::N1> {}
    impl<In> Sealed for super::Vector<In, typenum::N2> {}
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<In, Time: Integer> Clone for Vector<In, Time> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<In, Time: Integer> Copy for Vector<In, Time> {}

/// Quickly construct a [`Vector`] using named components.
///
/// This macro allows constructing [`Vector`]s for a coordinate system by naming the arguments
/// according to its [`CoordinateSystem::Convention`]. Using the wrong named arguments (eg, trying
/// to make a [`NedLike`] [`Vector`] using `f = `) will produce an error at compile-time.
///
/// For [`NedLike`], use:
///
/// ```rust
/// # use sguaba::{vector, system, Vector};
/// # use uom::si::f64::Length;
/// # use uom::si::length::meter;
/// # system!(struct Ned using NED);
/// # let _: Vector<Ned> =
/// vector! {
///     n = Length::new::<meter>(1.),
///     e = Length::new::<meter>(2.),
///     d = Length::new::<meter>(3.),
/// }
/// # ;
/// ```
///
/// For [`FrdLike`], use:
///
/// ```rust
/// # use sguaba::{vector, system, Vector};
/// # use uom::si::f64::Length;
/// # use uom::si::length::meter;
/// # system!(struct Frd using FRD);
/// # let _: Vector<Frd> =
/// vector! {
///     f = Length::new::<meter>(1.),
///     r = Length::new::<meter>(2.),
///     d = Length::new::<meter>(3.),
/// }
/// # ;
/// ```
///
/// For [`RightHandedXyzLike`], use:
///
/// ```rust
/// # use sguaba::{vector, systems::Ecef, Vector};
/// # use uom::si::f64::Length;
/// # use uom::si::length::meter;
/// # let _: Vector<Ecef> =
/// vector! {
///     x = Length::new::<meter>(1.),
///     y = Length::new::<meter>(2.),
///     z = Length::new::<meter>(3.),
/// }
/// # ;
/// ```
///
/// The macro also allows explicitly specifying the coordinate system `In` for the returned
/// [`Vector`] (which is otherwise inferred) by suffixing the component list with `; in System`,
/// like so:
///
/// ```rust
/// # use sguaba::{vector, system, Vector};
/// # use uom::si::f64::Length;
/// # use uom::si::length::meter;
/// system!(struct Frd using FRD);
/// vector! {
///     f = Length::new::<meter>(1.),
///     r = Length::new::<meter>(2.),
///     d = Length::new::<meter>(3.);
///     in Frd
/// }
/// # ;
/// ```
///
/// The macro automatically determines the vector unit type based on the units provided:
///
/// ```rust
/// # use sguaba::{system, vector};
/// # use uom::si::f64::{Length, Velocity, Acceleration};
/// # use uom::si::{length::meter, velocity::meter_per_second, acceleration::meter_per_second_squared};
/// # system!(struct PlaneFrd using FRD);
///
/// // Creates Vector<PlaneFrd, Z0> / LengthVector (length vector)
/// let position = vector!(
///     f = Length::new::<meter>(1.0),
///     r = Length::new::<meter>(0.0),
///     d = Length::new::<meter>(0.0);
///     in PlaneFrd
/// );
///
/// // Creates Vector<PlaneFrd, N1> / VelocityVector (velocity vector)
/// let velocity = vector!(
///     f = Velocity::new::<meter_per_second>(5.0),
///     r = Velocity::new::<meter_per_second>(0.0),
///     d = Velocity::new::<meter_per_second>(0.0);
///     in PlaneFrd
/// );
///
/// // Creates Vector<PlaneFrd, N2> / AccelerationVector (acceleration vector)
/// let acceleration = vector!(
///     f = Acceleration::new::<meter_per_second_squared>(2.0),
///     r = Acceleration::new::<meter_per_second_squared>(0.0),
///     d = Acceleration::new::<meter_per_second_squared>(0.0);
///     in PlaneFrd
/// );
/// ```
///
#[macro_export]
macro_rules! vector {
    ($x:tt = $xx:expr, $y:tt = $yy:expr, $z:tt = $zz:expr $(,)?) => {
        vector!($x = $xx, $y = $yy, $z = $zz; in _)
    };
    (n = $n:expr, e = $e:expr, d = $d:expr; in $in:ty) => {
        $crate::Vector::<$in, _>::build($crate::systems::NedComponents {
            north: $n.into(),
            east: $e.into(),
            down: $d.into(),
        })
    };
    (e = $e:expr, n = $n:expr, u = $u:expr; in $in:ty) => {
        $crate::Vector::<$in, _>::build($crate::systems::EnuComponents {
            east: $e.into(),
            north: $n.into(),
            up: $u.into(),
        })
    };
    (f = $f:expr, r = $r:expr, d = $d:expr; in $in:ty) => {
        $crate::Vector::<$in, _>::build($crate::systems::FrdComponents {
            front: $f.into(),
            right: $r.into(),
            down: $d.into(),
        })
    };
    (x = $x:expr, y = $y:expr, z = $z:expr; in $in:ty) => {
        $crate::Vector::<$in, _>::build($crate::systems::XyzComponents {
            x: $x.into(),
            y: $y.into(),
            z: $z.into(),
        })
    };
}

impl<In, Time> Vector<In, Time>
where
    Time: Integer,
    Self: LengthBasedComponents<In, Time>,
{
    pub(crate) fn from_nalgebra_vector(value: Vector3) -> Self {
        Self {
            inner: value,
            system: PhantomData::<In>,
            unit: PhantomData::<LengthPossiblyPer<Time>>,
        }
    }

    /// Constructs a [`Vector`] at the given (x, y, z) Cartesian point in the [`CoordinateSystem`]
    /// `In`.
    pub fn build(components: <In::Convention as HasComponents<Time>>::Components) -> Self
    where
        In: CoordinateSystem,
        In::Convention: HasComponents<Time>,
    {
        let [x, y, z] = components.into();
        #[allow(deprecated)]
        Self::from_cartesian(x, y, z)
    }

    /// Provides a constructor for a [`Coordinate`] in the [`CoordinateSystem`] `In`.
    pub fn builder() -> Builder<In, Unset, Unset, Unset, Time>
    where
        In: CoordinateSystem,
    {
        Builder::default()
    }

    /// Constructs a vector with the given (x, y, z) cartesian components in the
    /// [`CoordinateSystem`] `In`.
    ///
    /// Prefer [`Vector::builder`], [`Vector::build`], or [`vector`] to avoid risk of argument
    /// order confusion. This function will be removed in a future version of Sguaba in favor of
    /// those.
    ///
    /// <div class="warning">
    ///
    /// This method is permanently deprecated because it is particularly vulnerable to argument
    /// order confusion (eg, accidentally passing in the "down" component of a FRD vector
    /// first instead of last). Methods like [`Vector::builder`] and the [`vector!`] macro
    /// should be preferred instead, as they do not have this problem. However, this method is
    /// left for use-cases where those alternatives cannot be used, such as when writing code
    /// that is fully generic over the coordinate system, and thus cannot use the safer
    /// constructors provided by those APIs. If this applies to you, make sure you apply due
    /// diligence when writing out the argument ordering.
    ///
    /// </div>
    ///
    /// The meaning of `x`, `y`, and `z` is dictated by the "convention" of `In`. For example, in
    /// [`NedLike`], `x` is North, `y` is East, and `z` is "down" (ie, in the direction of
    /// gravity).
    #[deprecated = "prefer `Vector::builder` to avoid risk of argument order confusion"]
    pub fn from_cartesian(
        x: impl Into<LengthPossiblyPer<Time>>,
        y: impl Into<LengthPossiblyPer<Time>>,
        z: impl Into<LengthPossiblyPer<Time>>,
    ) -> Self {
        LengthBasedComponents::from_cartesian([x.into(), y.into(), z.into()])
    }

    /// Constructs a vector with the given (r, θ, φ) spherical components in the
    /// [`CoordinateSystem`] `In`.
    ///
    /// This constructor follows the physics convention for [spherical coordinates][sph], which
    /// defines
    ///
    /// - r as the radial distance, meaning the slant distance to origin;
    /// - θ (theta) as the polar angle meaning the angle with respect to positive polar axis (Z);
    ///   and
    /// - φ (phi) as the azimuthal angle, meaning the angle of rotation from the initial meridian
    ///   plane (positive X towards positive Y).
    ///
    /// The axes and planes above are in turn dictated by the [`CoordinateSystem::Convention`] of
    /// `In`. For example, in [`NedLike`], the polar angle is the angle relative to straight down
    /// and the azimuthal angle is the angle from North. In [`FrdLike`], the polar angle is the
    /// angle relative to "down" and the azimuthal angle is the angle from forward.
    ///
    /// <div class="warning">
    ///
    /// Note that this convention does not necessarily match the conventions of each coordinate
    /// system. For example, in [`NedLike`] and [`FrdLike`] coordinate systems, we often talk about
    /// "azimuth and elevation", but those are distinct from the spherical coordinates. In that
    /// context, the definition of azimuth (luckily) matches that of spherical coordinates, but
    /// elevation tends to be defined as the angle from the positive X axis rather than from the
    /// positive Z axis. If you want to define a coordinate based on azimuth and elevation, use
    /// [`Vector::from_bearing`].
    ///
    /// </div>
    ///
    /// # Examples
    ///
    /// ```rust
    /// use approx::assert_relative_eq;
    /// use sguaba::{system, vector, Vector};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct Ned using NED);
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_spherical(unit, Angle::new::<degree>(0.), Angle::new::<degree>(0.)),
    ///     vector!(n = zero, e = zero, d = unit),
    /// );
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_spherical(unit, Angle::new::<degree>(90.), Angle::new::<degree>(0.)),
    ///     vector!(n = unit, e = zero, d = zero),
    /// );
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_spherical(unit, Angle::new::<degree>(90.), Angle::new::<degree>(90.)),
    ///     vector!(n = zero, e = unit, d = zero),
    /// );
    /// ```
    ///
    /// [sph]: https://en.wikipedia.org/wiki/Spherical_coordinate_system
    pub fn from_spherical(
        radius: impl Into<LengthPossiblyPer<Time>>,
        polar: impl Into<Angle>,
        azimuth: impl Into<Angle>,
    ) -> Self {
        let radius: LengthPossiblyPer<Time> = radius.into();
        let azimuth = azimuth.into();
        let polar = polar.into();

        let x = radius * polar.sin().value * azimuth.cos().value;
        let y = radius * polar.sin().value * azimuth.sin().value;
        let z = radius * polar.cos().value;

        #[allow(deprecated)]
        Self::from_cartesian(x, y, z)
    }

    /// Constructs a vector with the given azimuth, elevation, and range in the
    /// [`CoordinateSystem`] `In`.
    ///
    /// This constructor is based on the [bearing] as it defined by the implementation of
    /// [`BearingDefined`] for `In`. For [`NedLike`] and [`FrdLike`] coordinate systems, this is:
    ///
    /// - azimuth is the angle clockwise as seen from "up" along the XY plane from the positive X
    ///   axis (eg, North or Forward); and
    /// - elevation is the angle towards Z from the XY plane; and
    /// - r is the radial distance, meaning the slant distance to origin.
    ///
    /// See also [horizontal coordinate systems][azel].
    ///
    /// # Examples
    ///
    /// ```rust
    /// use approx::assert_relative_eq;
    /// use sguaba::{system, vector, Bearing, Vector};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct Ned using NED);
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_bearing(
    ///       Bearing::builder()
    ///         .azimuth(Angle::new::<degree>(0.))
    ///         .elevation(Angle::new::<degree>(0.)).expect("elevation is in-range")
    ///         .build(),
    ///       unit
    ///     ),
    ///     vector!(n = unit, e = zero, d = zero),
    /// );
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_bearing(
    ///       Bearing::builder()
    ///         .azimuth(Angle::new::<degree>(90.))
    ///         .elevation(Angle::new::<degree>(0.)).expect("elevation is in-range")
    ///         .build(),
    ///       unit
    ///     ),
    ///     vector!(n = zero, e = unit, d = zero),
    /// );
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_bearing(
    ///       Bearing::builder()
    ///         .azimuth(Angle::new::<degree>(90.))
    ///         .elevation(Angle::new::<degree>(90.)).expect("elevation is in-range")
    ///         .build(),
    ///       unit
    ///     ),
    ///     vector!(n = zero, e = zero, d = -unit),
    /// );
    /// ```
    ///
    /// [bearing]: https://en.wikipedia.org/wiki/Bearing_%28navigation%29
    /// [azel]: https://en.wikipedia.org/wiki/Horizontal_coordinate_system
    pub fn from_bearing(bearing: Bearing<In>, range: impl Into<LengthPossiblyPer<Time>>) -> Self
    where
        In: crate::systems::BearingDefined,
    {
        let (polar, azimuth) = In::bearing_to_spherical(bearing);
        Self::from_spherical(range.into(), polar, azimuth)
    }

    /// Changes the coordinate system of the vector to `<NewIn>` with no changes to the components.
    ///
    /// This is useful useful when the transform from one coordinate system to another only
    /// includes translation and not rotation, since vectors are unaffected by translation (see
    /// [`RigidBodyTransform::transform`]) and thus can be equivalently used in both.
    ///
    /// It can also be useful when you have two coordinate systems that you know have exactly
    /// equivalent axes, and you need the types to "work out". This can be the case, for instance,
    /// when two crates have both separately declared a NED-like coordinate system centered on the
    /// same object, and you need to move vectors between them. In such cases, however, prefer
    /// implementing [`EquivalentTo`] and using [`Vector::cast`], as it is harder to accidentally
    /// misuse.
    ///
    /// This is not how you _generally_ want to convert between coordinate systems. For that,
    /// you'll want to use [`RigidBodyTransform`].
    ///
    /// This is exactly equivalent to re-constructing the vector with the same component values
    /// using `<NewIn>` instead of `<In, Time>`, just more concise and legible. That is, it is exactly
    /// equal to:
    ///
    /// ```
    /// use sguaba::{system, vector, Bearing, Vector};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct PlaneNedFromCrate1 using NED);
    /// system!(struct PlaneNedFromCrate2 using NED);
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// let vector_in_1 = vector!(n = unit, e = zero, d = unit; in PlaneNedFromCrate1);
    ///
    /// assert_eq!(
    ///     vector! {
    ///       n = vector_in_1.ned_north(),
    ///       e = vector_in_1.ned_east(),
    ///       d = vector_in_1.ned_down();
    ///       in PlaneNedFromCrate2
    ///     },
    ///     vector_in_1.with_same_components_in::<PlaneNedFromCrate2>()
    /// );
    /// ```
    #[must_use]
    pub fn with_same_components_in<NewIn>(self) -> Vector<NewIn, Time> {
        Vector {
            inner: self.inner,
            system: PhantomData::<NewIn>,
            unit: self.unit,
        }
    }

    /// Casts the coordinate system type parameter of the vector to the equivalent coordinate
    /// system `NewIn`.
    ///
    /// See [`EquivalentTo`] for details on when this is useful (and safe).
    ///
    /// Note that this performs no transform on the vector's components, as that should be
    /// unnecessary when `EquivalentTo` is implemented.
    ///
    /// ```
    /// use sguaba::{system, vector, systems::EquivalentTo, Vector};
    /// use uom::si::{f64::Length, length::meter};
    ///
    /// system!(struct PlaneNedFromCrate1 using NED);
    /// system!(struct PlaneNedFromCrate2 using NED);
    ///
    /// // SAFETY: these are truly the same thing just defined in different places
    /// unsafe impl EquivalentTo<PlaneNedFromCrate1> for PlaneNedFromCrate2 {}
    /// unsafe impl EquivalentTo<PlaneNedFromCrate2> for PlaneNedFromCrate1 {}
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// let vector_in_1 = vector!(n = unit, e = zero, d = unit; in PlaneNedFromCrate1);
    ///
    /// assert_eq!(
    ///     vector! {
    ///       n = vector_in_1.ned_north(),
    ///       e = vector_in_1.ned_east(),
    ///       d = vector_in_1.ned_down();
    ///       in PlaneNedFromCrate2
    ///     },
    ///     vector_in_1.cast::<PlaneNedFromCrate2>()
    /// );
    /// ```
    #[must_use]
    pub fn cast<NewIn>(self) -> Vector<NewIn, Time>
    where
        In: EquivalentTo<NewIn>,
    {
        self.with_same_components_in::<NewIn>()
    }

    /// Returns a unit vector with the same direction as this vector.
    #[must_use]
    pub fn normalized(&self) -> Self {
        Self::from_nalgebra_vector(self.inner.normalize())
    }

    /// Computes the dot (scalar) product between this vector and another.
    ///
    /// Note that this method's return value is unitless since the unit of the dot product is not
    /// meaningful in terms of the units of the underlying components.
    #[must_use]
    pub fn dot(&self, rhs: &Self) -> f64 {
        self.inner.dot(&rhs.inner)
    }

    /// Linearly interpolate between this vector and another vector.
    ///
    /// Specifically, returns `self * (1.0 - t) + rhs * t`, i.e., the linear blend of the
    /// two vectors using the scalar value `t`.
    ///
    /// The value for `t` is not restricted to the range [0, 1].
    #[must_use]
    pub fn lerp(&self, rhs: &Self, t: f64) -> Self {
        Self {
            inner: self.inner.lerp(&rhs.inner, t),
            system: self.system,
            unit: self.unit,
        }
    }

    /// Computes the bearing of the vector as if the vector starts at the origin of `In`.
    ///
    /// Returns `None` if the vector has zero length, as the azimuth is then ill-defined.
    ///
    /// Returns an azimuth of zero if the vector points directly along the Z axis.
    #[must_use]
    pub fn bearing_at_origin(&self) -> Option<Bearing<In>>
    where
        In: crate::systems::BearingDefined,
    {
        // This is obviously a little sneaky since we're coercing to meters, but since we're
        // coercing all the components the same way, the resulting bearing should remain the same.
        let [x, y, z] = self.to_cartesian();
        let x = Vector::recast_to_length(x);
        let y = Vector::recast_to_length(y);
        let z = Vector::recast_to_length(z);
        #[allow(deprecated)]
        Coordinate::from_cartesian(x, y, z).bearing_from_origin()
    }

    /// Computes the orientation of the vector as if the vector starts at the origin of `In`.
    ///
    /// Since vectors do not include roll information, the desired roll must be passed in. It's
    /// worth reading the documentation on [`Orientation`] for a reminder about the meaning of roll
    /// here. Very briefly, it is intrinsic, and 0º roll means the object's positive Z axis is
    /// aligned with `In`'s positive Z axis.
    ///
    /// Returns `None` if the vector has zero length, as the yaw is then ill-defined.
    ///
    /// Returns a yaw of zero if the vector points directly along the Z axis.
    #[must_use]
    pub fn orientation_at_origin(&self, roll: impl Into<Angle>) -> Option<Orientation<In>> {
        // The vector is effectively an axis-angle representation of the Tait-Bryan orientation
        // we're after (with the angle of rotation being 0º). But, when angle is 0º, the axis-angle
        // representation ends up being a noop from what I can tell. Instead, we compute this from
        // first principles. Note that we cast to the raw length, but that's okay since we only
        // care about the relative magnitudes for these as we're computing angles.
        let [x, y, z] = self.to_cartesian();
        let x = Vector::recast_to_length(x);
        let y = Vector::recast_to_length(y);
        let z = Vector::recast_to_length(z);

        if x == Length::ZERO && y == Length::ZERO && z == Length::ZERO {
            return None;
        }

        // Per `Orientation::from_tait_bryan_angles`, yaw is rotation about the Z axis with an
        // object at 0º yaw facing along the positive X axis. Thus, it is rotation on the XY plane
        // (and uses only the x and y coordinates). Positive yaw is counter-clockwise rotation
        // about Z (ie, towards +Y from +X), and thus we want the sin of the angle between X and Y
        // with no sign flipping.
        //
        // Also note that atan2 guarantees that it returns 0 if both components are 0.
        let yaw = y.atan2(x);
        // Pitch is the rotation about the Y axis, with 0º pitch being aligned with the yaw axis
        // (since we're using intrinsic rotations). Per the right-hand rule, positive pitch
        // rotates from +X toward -Z, so we negate z to get the correct sign.
        let pitch = (-z).atan2((x.powi(P2::new()) + y.powi(P2::new())).sqrt());
        // And the roll is passed in.
        let roll = roll.into();

        Some(
            Orientation::tait_bryan_builder()
                .yaw(yaw)
                .pitch(pitch)
                .roll(roll)
                .build(),
        )
    }

    /// Constructs a vector in coordinate system `In` whose components are all 0.
    ///
    /// This is the [additive identity][id] for `In` under vector addition, meaning adding this
    /// vector to any coordinate or vector in `In` will yield the origin coordinate or vector
    /// unchanged.
    ///
    /// [id]: https://en.wikipedia.org/wiki/Zero_element#Additive_identities
    #[must_use]
    pub fn zero() -> Self {
        Self {
            inner: Vector3::zeros(),
            system: PhantomData::<In>,
            unit: PhantomData::<LengthPossiblyPer<Time>>,
        }
    }
}

impl<In> From<Coordinate<In>> for Vector<In, Z0> {
    fn from(value: Coordinate<In>) -> Self {
        Self::from_nalgebra_vector(value.point.coords)
    }
}

impl<In, Time> Default for Vector<In, Time>
where
    Time: Integer,
    Self: LengthBasedComponents<In, Time>,
{
    fn default() -> Self {
        Self::zero()
    }
}

macro_rules! accessors {
    {
        $convention:ident
        using $x:ident, $y:ident, $z:ident
        // TODO: https://github.com/rust-lang/rust/issues/124225
        + $x_ax:ident, $y_ax:ident, $z_ax:ident
    } => {
        impl<In, Time: typenum::Integer> Vector<In, Time> where In: CoordinateSystem<Convention = $convention> {
            #[must_use]
            pub fn $x(&self) -> Length { Length::new::<meter>(self.inner.x) }
            #[must_use]
            pub fn $y(&self) -> Length { Length::new::<meter>(self.inner.y) }
            #[must_use]
            pub fn $z(&self) -> Length { Length::new::<meter>(self.inner.z) }

            #[must_use]
            pub fn $x_ax() -> Vector<In, Time> {
                Self {
                    inner: *Vector3::x_axis(),
                    system: core::marker::PhantomData::<In>,
                    unit: core::marker::PhantomData::<LengthPossiblyPer<Time>>,
                }
            }
            #[must_use]
            pub fn $y_ax() -> Vector<In, Time> {
                Self {
                    inner: *Vector3::y_axis(),
                    system: core::marker::PhantomData::<In>,
                    unit: core::marker::PhantomData::<LengthPossiblyPer<Time>>,
                }
            }
            #[must_use]
            pub fn $z_ax() -> Vector<In, Time> {
                Self {
                    inner: *Vector3::z_axis(),
                    system: core::marker::PhantomData::<In>,
                    unit: core::marker::PhantomData::<LengthPossiblyPer<Time>>,
                }
            }
        }
    };
}

accessors!(RightHandedXyzLike using x, y, z + x_axis, y_axis, z_axis);
accessors!(NedLike using ned_north, ned_east, ned_down + ned_north_axis, ned_east_axis, ned_down_axis);
accessors!(FrdLike using frd_front, frd_right, frd_down + frd_front_axis, frd_right_axis, frd_down_axis);
accessors!(EnuLike using enu_east, enu_north, enu_up + enu_east_axis, enu_north_axis, enu_up_axis);

impl<In> Vector<In, Z0> {
    /// Returns the cartesian components of this vector in XYZ order.
    ///
    /// To turn this into a simple (ie, unitless) `[f64; 3]`, use [`array::map`] combined with
    /// `.get::<meter>()`.
    #[doc(alias = "components")]
    #[must_use]
    pub fn to_cartesian(&self) -> [Length; 3] {
        LengthBasedComponents::to_cartesian(self)
    }

    /// Computes the magnitude of the vector (ie, its length).
    #[doc(alias = "norm")]
    #[doc(alias = "distance")]
    #[doc(alias = "length")]
    #[must_use]
    pub fn magnitude(&self) -> Length {
        Length::new::<meter>(self.inner.norm())
    }
}

impl<In> Vector<In, N1> {
    /// Returns the cartesian components of this velocity vector in XYZ order.
    ///
    /// To turn this into a simple (ie, unitless) `[f64; 3]`, use [`array::map`] combined with
    /// `.get::<meter_per_second>()`.
    #[doc(alias = "components")]
    #[must_use]
    pub fn to_cartesian(&self) -> [Velocity; 3] {
        LengthBasedComponents::to_cartesian(self)
    }

    /// Computes the magnitude of the vector (ie, its velocity).
    #[doc(alias = "norm")]
    #[doc(alias = "speed")]
    #[must_use]
    pub fn magnitude(&self) -> Velocity {
        Velocity::new::<meter_per_second>(self.inner.norm())
    }
}

impl<In> Vector<In, N2> {
    /// Returns the cartesian components of this vector in XYZ order.
    ///
    /// To turn this into a simple (ie, unitless) `[f64; 3]`, use [`array::map`] combined with
    /// `.get::<meter_per_second_squared>()`.
    #[doc(alias = "components")]
    #[must_use]
    pub fn to_cartesian(&self) -> [Acceleration; 3] {
        LengthBasedComponents::to_cartesian(self)
    }

    /// Computes the magnitude of the vector (ie, its acceleration).
    #[doc(alias = "norm")]
    #[doc(alias = "acceleration")]
    #[must_use]
    pub fn magnitude(&self) -> Acceleration {
        Acceleration::new::<meter_per_second_squared>(self.inner.norm())
    }
}

impl<In, Time: Integer> Neg for Vector<In, Time> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            inner: -self.inner,
            system: self.system,
            unit: self.unit,
        }
    }
}

impl<In, Time: Integer> Add<Self> for Vector<In, Time> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            inner: self.inner + rhs.inner,
            system: self.system,
            unit: self.unit,
        }
    }
}

impl<In, Time: Integer> AddAssign<Self> for Vector<In, Time> {
    fn add_assign(&mut self, rhs: Self) {
        self.inner += rhs.inner;
    }
}

impl<In, Time: Integer> Sub<Self> for Vector<In, Time> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            inner: self.inner - rhs.inner,
            system: self.system,
            unit: self.unit,
        }
    }
}

impl<In, Time: Integer> SubAssign<Self> for Vector<In, Time> {
    fn sub_assign(&mut self, rhs: Self) {
        self.inner -= rhs.inner;
    }
}

impl<In, Time: Integer> Sum for Vector<In, Time>
where
    Self: LengthBasedComponents<In, Time>,
{
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::zero(), |sum, v| sum + v)
    }
}

// multiplying an acceleration vector by time gives you a velocity vector
impl<In> Mul<uom::si::f64::Time> for Vector<In, N2> {
    type Output = Vector<In, N1>;

    fn mul(self, duration: uom::si::f64::Time) -> Self::Output {
        LengthBasedComponents::from_cartesian(self.to_cartesian().map(|v| v * duration))
    }
}
// multiplying a velocity vector by time gives you a length vector
impl<In> Mul<uom::si::f64::Time> for Vector<In, N1> {
    type Output = Vector<In, Z0>;

    fn mul(self, duration: uom::si::f64::Time) -> Self::Output {
        LengthBasedComponents::from_cartesian(self.to_cartesian().map(|v| v * duration))
    }
}

impl<In, Time: Integer> Mul<f64> for Vector<In, Time> {
    type Output = Self;

    fn mul(self, scalar: f64) -> Self::Output {
        Self {
            inner: self.inner * scalar,
            system: self.system,
            unit: self.unit,
        }
    }
}

impl<In, Time: Integer> Div<f64> for Vector<In, Time> {
    type Output = Self;

    fn div(self, scalar: f64) -> Self::Output {
        Self {
            inner: self.inner / scalar,
            system: self.system,
            unit: self.unit,
        }
    }
}

impl<In, Time: Integer> Div<Length> for Vector<In, Time> {
    type Output = Self;

    fn div(self, rhs: Length) -> Self::Output {
        self / rhs.get::<meter>()
    }
}

impl<In, Time: Integer> Mul<Length> for Vector<In, Time> {
    type Output = Self;

    fn mul(self, rhs: Length) -> Self::Output {
        self * rhs.get::<meter>()
    }
}

impl<In, Time: Integer> PartialEq<Self> for Vector<In, Time> {
    fn eq(&self, other: &Self) -> bool {
        self.inner.eq(&other.inner)
    }
}

impl<In, Time: Integer> Display for Vector<In, Time> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.inner)
    }
}

#[cfg(any(test, feature = "approx"))]
impl<In, Time: Integer> AbsDiffEq<Self> for Vector<In, Time> {
    type Epsilon = <f64 as AbsDiffEq>::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        // NOTE(jon): this value is in Meters, and realistically we're fine with .1m precision
        0.1
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.inner.abs_diff_eq(&other.inner, epsilon)
    }
}

#[cfg(any(test, feature = "approx"))]
impl<In, Time: Integer> RelativeEq for Vector<In, Time> {
    fn default_max_relative() -> Self::Epsilon {
        Point3::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        self.inner.relative_eq(&other.inner, epsilon, max_relative)
    }
}

/// [Builder] for a [`Vector`].
///
/// Construct one through [`Vector::builder`], and finalize with [`Builder::build`].
///
/// [Builder]: https://rust-unofficial.github.io/patterns/patterns/creational/builder.html
#[derive(Debug)]
#[must_use]
pub struct Builder<In, X, Y, Z, Time = Z0>
where
    Time: Integer,
{
    under_construction: Vector<In, Time>,
    set: (PhantomData<X>, PhantomData<Y>, PhantomData<Z>),
}
impl<In, Time> Default for Builder<In, Unset, Unset, Unset, Time>
where
    Time: Integer,
    Vector<In, Time>: LengthBasedComponents<In, Time>,
{
    fn default() -> Self {
        Self {
            under_construction: Vector::default(),
            set: (PhantomData, PhantomData, PhantomData),
        }
    }
}

impl<In, Time: Integer> Builder<In, Set, Set, Set, Time> {
    /// Constructs a [`Vector`] at the currently set (x, y, z) Cartesian point in the
    /// [`CoordinateSystem`] `In`.
    #[must_use]
    pub fn build(self) -> Vector<In, Time> {
        self.under_construction
    }
}

macro_rules! constructor {
    ($like:ident, [$x:ident, $y:ident, $z:ident]) => {
        impl<In, X, Y, Z, Time> Builder<In, X, Y, Z, Time>
        where
            In: CoordinateSystem<Convention = $like>,
            Time: typenum::Integer,
        {
            /// Sets the X component of this [`Vector`]-to-be.
            pub fn $x(mut self, length: impl Into<Length>) -> Builder<In, Set, Y, Z, Time> {
                self.under_construction.inner.x = length.into().get::<meter>();
                Builder {
                    under_construction: self.under_construction,
                    set: (PhantomData::<Set>, self.set.1, self.set.2),
                }
            }

            /// Sets the Y component of this [`Vector`]-to-be.
            pub fn $y(mut self, length: impl Into<Length>) -> Builder<In, X, Set, Z, Time> {
                self.under_construction.inner.y = length.into().get::<meter>();
                Builder {
                    under_construction: self.under_construction,
                    set: (self.set.0, PhantomData::<Set>, self.set.2),
                }
            }

            /// Sets the Z component of this [`Vector`]-to-be.
            pub fn $z(mut self, length: impl Into<Length>) -> Builder<In, X, Y, Set, Time> {
                self.under_construction.inner.z = length.into().get::<meter>();
                Builder {
                    under_construction: self.under_construction,
                    set: (self.set.0, self.set.1, PhantomData::<Set>),
                }
            }
        }
    };
}
constructor!(RightHandedXyzLike, [x, y, z]);
constructor!(NedLike, [ned_north, ned_east, ned_down]);
constructor!(FrdLike, [frd_front, frd_right, frd_down]);
constructor!(EnuLike, [enu_east, enu_north, enu_up]);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::systems::EquivalentTo;
    use crate::{coordinate, system, vector};
    use approx::assert_abs_diff_eq;
    use typenum::{N1, N2, Z0};
    use uom::si::f64::{Acceleration, Angle, Length, Velocity};
    use uom::si::{
        acceleration::meter_per_second_squared, angle::degree, length::meter,
        velocity::meter_per_second,
    };

    // Define test coordinate systems
    system!(struct TestFrd using FRD);
    system!(struct TestNed using NED);
    system!(struct TestEnu using ENU);
    system!(struct TestXyz using right-handed XYZ);

    // Define equivalent coordinate systems for testing
    system!(struct TestFrd2 using FRD);
    unsafe impl EquivalentTo<TestFrd> for TestFrd2 {}
    unsafe impl EquivalentTo<TestFrd2> for TestFrd {}

    // Helper functions for creating units
    fn m(meters: f64) -> Length {
        Length::new::<meter>(meters)
    }

    fn mps(meters_per_second: f64) -> Velocity {
        Velocity::new::<meter_per_second>(meters_per_second)
    }

    fn mps2(meters_per_second_squared: f64) -> Acceleration {
        Acceleration::new::<meter_per_second_squared>(meters_per_second_squared)
    }

    // Tests for Length vectors (Time = Z0)
    mod length_vectors {
        use super::*;

        #[test]
        fn constructor_from_cartesian_works() {
            #[allow(deprecated)]
            let v = Vector::<TestFrd, Z0>::from_cartesian(m(1.0), m(2.0), m(3.0));
            assert_eq!(v.to_cartesian(), [m(1.0), m(2.0), m(3.0)]);
        }

        #[test]
        fn builder_pattern_works() {
            let v = Vector::<TestFrd>::builder()
                .frd_front(m(1.0))
                .frd_right(m(2.0))
                .frd_down(m(3.0))
                .build();
            assert_eq!(v.frd_front(), m(1.0));
            assert_eq!(v.frd_right(), m(2.0));
            assert_eq!(v.frd_down(), m(3.0));
        }

        #[test]
        fn vector_macro_works() {
            let v = vector!(f = m(1.0), r = m(2.0), d = m(3.0); in TestFrd);
            assert_eq!(v.frd_front(), m(1.0));
            assert_eq!(v.frd_right(), m(2.0));
            assert_eq!(v.frd_down(), m(3.0));
        }

        #[test]
        fn build_with_components_works() {
            use crate::systems::FrdComponents;
            let v = Vector::<TestFrd>::build(FrdComponents {
                front: m(1.0),
                right: m(2.0),
                down: m(3.0),
            });
            assert_eq!(v.frd_front(), m(1.0));
            assert_eq!(v.frd_right(), m(2.0));
            assert_eq!(v.frd_down(), m(3.0));
        }

        #[test]
        fn from_spherical_works() {
            let v = Vector::<TestFrd>::from_spherical(
                m(10.0),
                Angle::new::<degree>(90.0), // polar angle (90 degrees)
                Angle::new::<degree>(0.0),  // azimuth angle (0 degrees)
            );
            assert_abs_diff_eq!(v.frd_front().get::<meter>(), 10.0, epsilon = 1e-10);
            assert_abs_diff_eq!(v.frd_right().get::<meter>(), 0.0, epsilon = 1e-10);
            assert_abs_diff_eq!(v.frd_down().get::<meter>(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn magnitude_works() {
            let v = vector!(f = m(3.0), r = m(4.0), d = m(0.0); in TestFrd);
            assert_eq!(v.magnitude(), m(5.0));
        }

        #[test]
        fn axis_vectors_work() {
            let front = Vector::<TestFrd>::frd_front_axis();
            let right = Vector::<TestFrd>::frd_right_axis();
            let down = Vector::<TestFrd>::frd_down_axis();

            assert_eq!(front.frd_front(), m(1.0));
            assert_eq!(front.frd_right(), m(0.0));
            assert_eq!(front.frd_down(), m(0.0));

            assert_eq!(right.frd_front(), m(0.0));
            assert_eq!(right.frd_right(), m(1.0));
            assert_eq!(right.frd_down(), m(0.0));

            assert_eq!(down.frd_front(), m(0.0));
            assert_eq!(down.frd_right(), m(0.0));
            assert_eq!(down.frd_down(), m(1.0));
        }

        #[test]
        fn arithmetic_operations_work() {
            let v1 = vector!(f = m(1.0), r = m(2.0), d = m(3.0); in TestFrd);
            let v2 = vector!(f = m(4.0), r = m(5.0), d = m(6.0); in TestFrd);

            let sum = v1 + v2;
            assert_eq!(sum.to_cartesian(), [m(5.0), m(7.0), m(9.0)]);

            let diff = v2 - v1;
            assert_eq!(diff.to_cartesian(), [m(3.0), m(3.0), m(3.0)]);

            let neg = -v1;
            assert_eq!(neg.to_cartesian(), [m(-1.0), m(-2.0), m(-3.0)]);

            let scaled = v1 * 2.0;
            assert_eq!(scaled.to_cartesian(), [m(2.0), m(4.0), m(6.0)]);
        }

        #[test]
        fn zero_vector_works() {
            let v = Vector::<TestFrd>::zero();
            assert_eq!(v.to_cartesian(), [m(0.0), m(0.0), m(0.0)]);
        }

        #[test]
        fn lerp_works() {
            let v1 = vector!(f = m(0.0), r = m(0.0), d = m(0.0); in TestFrd);
            let v2 = vector!(f = m(10.0), r = m(20.0), d = m(30.0); in TestFrd);

            let mid = v1.lerp(&v2, 0.5);
            assert_eq!(mid.to_cartesian(), [m(5.0), m(10.0), m(15.0)]);
        }

        #[test]
        fn cast_equivalent_coordinate_systems() {
            let v_frd = vector!(f = m(1.0), r = m(2.0), d = m(3.0); in TestFrd);
            let v_frd2: Vector<TestFrd2> = v_frd.cast();

            assert_eq!(v_frd2.to_cartesian(), v_frd.to_cartesian());
        }

        #[test]
        fn with_same_components_in_works() {
            let v_frd = vector!(f = m(1.0), r = m(2.0), d = m(3.0); in TestFrd);
            let v_ned: Vector<TestNed> = v_frd.with_same_components_in();

            // Same underlying values, different coordinate system
            assert_eq!(v_ned.to_cartesian(), v_frd.to_cartesian());
        }

        #[test]
        fn ned_accessors_work() {
            let v = vector!(n = m(1.0), e = m(2.0), d = m(3.0); in TestNed);
            assert_eq!(v.ned_north(), m(1.0));
            assert_eq!(v.ned_east(), m(2.0));
            assert_eq!(v.ned_down(), m(3.0));

            let north = Vector::<TestNed>::ned_north_axis();
            let east = Vector::<TestNed>::ned_east_axis();
            let down = Vector::<TestNed>::ned_down_axis();

            assert_eq!(north.ned_north(), m(1.0));
            assert_eq!(east.ned_east(), m(1.0));
            assert_eq!(down.ned_down(), m(1.0));
        }

        #[test]
        fn enu_accessors_work() {
            let v = vector!(e = m(1.0), n = m(2.0), u = m(3.0); in TestEnu);
            assert_eq!(v.enu_east(), m(1.0));
            assert_eq!(v.enu_north(), m(2.0));
            assert_eq!(v.enu_up(), m(3.0));
        }

        #[test]
        fn xyz_accessors_work() {
            let v = vector!(x = m(1.0), y = m(2.0), z = m(3.0); in TestXyz);
            assert_eq!(v.x(), m(1.0));
            assert_eq!(v.y(), m(2.0));
            assert_eq!(v.z(), m(3.0));
        }

        #[test]
        fn orientation_at_origin_zero_vector_returns_none() {
            let v = Vector::<TestXyz>::zero();
            assert_eq!(v.orientation_at_origin(Angle::ZERO), None);
        }

        #[test]
        fn orientation_at_origin_positive_x_axis() {
            // Vector pointing along +X should have yaw=0º, pitch=0º
            let v = vector!(x = m(1.0), y = m(0.0), z = m(0.0); in TestXyz);
            let orientation = v
                .orientation_at_origin(Angle::ZERO)
                .expect("non-zero vector");
            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();

            assert_abs_diff_eq!(yaw.get::<degree>(), 0.0, epsilon = 1e-10);
            assert_abs_diff_eq!(pitch.get::<degree>(), 0.0, epsilon = 1e-10);
            assert_abs_diff_eq!(roll.get::<degree>(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn orientation_at_origin_positive_y_axis() {
            // Vector pointing along +Y should have yaw=90º, pitch=0º
            let v = vector!(x = m(0.0), y = m(1.0), z = m(0.0); in TestXyz);
            let orientation = v
                .orientation_at_origin(Angle::ZERO)
                .expect("non-zero vector");
            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();

            assert_abs_diff_eq!(yaw.get::<degree>(), 90.0, epsilon = 1e-10);
            assert_abs_diff_eq!(pitch.get::<degree>(), 0.0, epsilon = 1e-10);
            assert_abs_diff_eq!(roll.get::<degree>(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn orientation_at_origin_positive_z_axis() {
            // Vector pointing along +Z should have yaw=0º (arbitrary), pitch=-90º
            let v = vector!(x = m(0.0), y = m(0.0), z = m(1.0); in TestXyz);
            let orientation = v
                .orientation_at_origin(Angle::ZERO)
                .expect("non-zero vector");
            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();

            assert_abs_diff_eq!(yaw.get::<degree>(), 0.0, epsilon = 1e-10);
            assert_abs_diff_eq!(pitch.get::<degree>(), -90.0, epsilon = 1e-10);
            assert_abs_diff_eq!(roll.get::<degree>(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn orientation_at_origin_negative_x_axis() {
            // Vector pointing along -X should have yaw=180º or -180º, pitch=0º
            let v = vector!(x = m(-1.0), y = m(0.0), z = m(0.0); in TestXyz);
            let orientation = v
                .orientation_at_origin(Angle::ZERO)
                .expect("non-zero vector");
            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();

            // atan2(0, -1) = 180º or -180º depending on convention
            assert_abs_diff_eq!(yaw.get::<degree>().abs(), 180.0, epsilon = 1e-10);
            assert_abs_diff_eq!(pitch.get::<degree>(), 0.0, epsilon = 1e-10);
            assert_abs_diff_eq!(roll.get::<degree>(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn orientation_at_origin_45_degree_xy_plane() {
            // Vector at 45º in XY plane should have yaw=45º, pitch=0º
            let v = vector!(x = m(1.0), y = m(1.0), z = m(0.0); in TestXyz);
            let orientation = v
                .orientation_at_origin(Angle::ZERO)
                .expect("non-zero vector");
            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();

            assert_abs_diff_eq!(yaw.get::<degree>(), 45.0, epsilon = 1e-10);
            assert_abs_diff_eq!(pitch.get::<degree>(), 0.0, epsilon = 1e-10);
            assert_abs_diff_eq!(roll.get::<degree>(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn orientation_at_origin_45_degree_xz_plane() {
            // Vector at 45º in XZ plane should have yaw=0º, pitch=-45º
            let v = vector!(x = m(1.0), y = m(0.0), z = m(1.0); in TestXyz);
            let orientation = v
                .orientation_at_origin(Angle::ZERO)
                .expect("non-zero vector");
            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();

            assert_abs_diff_eq!(yaw.get::<degree>(), 0.0, epsilon = 1e-10);
            assert_abs_diff_eq!(pitch.get::<degree>(), -45.0, epsilon = 1e-10);
            assert_abs_diff_eq!(roll.get::<degree>(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn orientation_at_origin_general_3d_vector() {
            // Vector at (3, 4, 5) - verify the full computation
            // Expected yaw = atan2(4, 3) ≈ 53.13º
            // Expected pitch = atan2(-5, sqrt(3² + 4²)) = atan2(-5, 5) = -45º
            let v = vector!(x = m(3.0), y = m(4.0), z = m(5.0); in TestXyz);
            let orientation = v
                .orientation_at_origin(Angle::ZERO)
                .expect("non-zero vector");
            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();

            assert_abs_diff_eq!(yaw.get::<degree>(), 53.13010235415598, epsilon = 1e-10);
            assert_abs_diff_eq!(pitch.get::<degree>(), -45.0, epsilon = 1e-10);
            assert_abs_diff_eq!(roll.get::<degree>(), 0.0, epsilon = 1e-10);

            // Sanity-check that if we create an observation in that direction,
            // it matches the original vector.
            let frd = vector!(f = v.magnitude(), r = m(0.), d = m(0.); in TestFrd);
            let pose = crate::engineering::Pose::new(Coordinate::origin(), orientation);
            let pose = unsafe { pose.map_as_zero_in::<TestFrd>() };
            let v2 = pose.inverse_transform(frd);
            assert_abs_diff_eq!(v, v2);
        }

        #[test]
        fn orientation_at_origin_ned() {
            // In NED:
            //
            // - north is +X
            // - +Z is down
            // - positive yaw is from-north-towards-east
            // - positive pitch is towards -Z (as always), so "up"
            //
            // so a 45º-nose-up east vector should have yaw=90º and pitch=45º
            let v = vector!(n = m(0.0), e = m(1.0), d = m(-1.0); in TestNed);
            let orientation = v
                .orientation_at_origin(Angle::ZERO)
                .expect("non-zero vector");
            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();

            assert_abs_diff_eq!(yaw.get::<degree>(), 90.0, epsilon = 1e-10);
            assert_abs_diff_eq!(pitch.get::<degree>(), 45.0, epsilon = 1e-10);
            assert_abs_diff_eq!(roll.get::<degree>(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn orientation_at_origin_enu_east_vector() {
            // In ENU:
            //
            // - east is +X
            // - +Z is up
            // - positive yaw is from-east-towards-north
            // - positive pitch is towards -Z (as always), so "down"
            //
            // so a 45º-nose-up north vector should have yaw=90º and pitch=-45º
            let v = vector!(e = m(0.0), n = m(1.0), u = m(1.0); in TestEnu);
            let orientation = v
                .orientation_at_origin(Angle::ZERO)
                .expect("non-zero vector");
            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();

            assert_abs_diff_eq!(yaw.get::<degree>(), 90.0, epsilon = 1e-10);
            assert_abs_diff_eq!(pitch.get::<degree>(), -45.0, epsilon = 1e-10);
            assert_abs_diff_eq!(roll.get::<degree>(), 0.0, epsilon = 1e-10);
        }
    }

    // Tests for Velocity vectors (Time = N1)
    mod velocity_vectors {
        use super::*;

        #[test]
        fn constructor_from_cartesian_works() {
            #[allow(deprecated)]
            let v = Vector::<TestFrd, N1>::from_cartesian(mps(1.0), mps(2.0), mps(3.0));
            assert_eq!(v.to_cartesian(), [mps(1.0), mps(2.0), mps(3.0)]);
        }

        #[test]
        fn build_with_components_works() {
            use crate::systems::FrdComponents;
            let v = Vector::<TestFrd, N1>::build(FrdComponents {
                front: mps(1.0),
                right: mps(2.0),
                down: mps(3.0),
            });
            let cartesian = v.to_cartesian();
            assert_eq!(cartesian, [mps(1.0), mps(2.0), mps(3.0)]);
        }

        #[test]
        fn magnitude_works() {
            let v = vector!(f = mps(3.0), r = mps(4.0), d = mps(0.0); in TestFrd);
            assert_eq!(v.magnitude(), mps(5.0));
        }

        #[test]
        fn from_spherical_works() {
            let v = Vector::<TestFrd, N1>::from_spherical(
                mps(10.0),
                Angle::new::<degree>(90.0), // polar angle
                Angle::new::<degree>(0.0),  // azimuth angle
            );
            let cartesian = v.to_cartesian();
            assert_abs_diff_eq!(
                cartesian[0].get::<meter_per_second>(),
                10.0,
                epsilon = 1e-10
            );
            assert_abs_diff_eq!(cartesian[1].get::<meter_per_second>(), 0.0, epsilon = 1e-10);
            assert_abs_diff_eq!(cartesian[2].get::<meter_per_second>(), 0.0, epsilon = 1e-10);
        }

        #[test]
        fn axis_vectors_work() {
            let front = Vector::<TestFrd, N1>::frd_front_axis();
            let right = Vector::<TestFrd, N1>::frd_right_axis();
            let down = Vector::<TestFrd, N1>::frd_down_axis();

            let front_cart = front.to_cartesian();
            let right_cart = right.to_cartesian();
            let down_cart = down.to_cartesian();

            assert_eq!(front_cart, [mps(1.0), mps(0.0), mps(0.0)]);
            assert_eq!(right_cart, [mps(0.0), mps(1.0), mps(0.0)]);
            assert_eq!(down_cart, [mps(0.0), mps(0.0), mps(1.0)]);
        }

        #[test]
        fn arithmetic_operations_work() {
            let v1 = vector!(f = mps(1.0), r = mps(2.0), d = mps(3.0); in TestFrd);
            let v2 = vector!(f = mps(4.0), r = mps(5.0), d = mps(6.0); in TestFrd);

            let sum = v1 + v2;
            assert_eq!(sum.to_cartesian(), [mps(5.0), mps(7.0), mps(9.0)]);

            let diff = v2 - v1;
            assert_eq!(diff.to_cartesian(), [mps(3.0), mps(3.0), mps(3.0)]);

            let neg = -v1;
            assert_eq!(neg.to_cartesian(), [mps(-1.0), mps(-2.0), mps(-3.0)]);

            let scaled = v1 * 2.0;
            assert_eq!(scaled.to_cartesian(), [mps(2.0), mps(4.0), mps(6.0)]);
        }

        #[test]
        fn zero_vector_works() {
            let v = Vector::<TestFrd, N1>::zero();
            assert_eq!(v.to_cartesian(), [mps(0.0), mps(0.0), mps(0.0)]);
        }

        #[test]
        fn lerp_works() {
            let v1 = vector!(f = mps(0.0), r = mps(0.0), d = mps(0.0); in TestFrd);
            let v2 = vector!(f = mps(10.0), r = mps(20.0), d = mps(30.0); in TestFrd);

            let mid = v1.lerp(&v2, 0.5);
            assert_eq!(mid.to_cartesian(), [mps(5.0), mps(10.0), mps(15.0)]);
        }

        #[test]
        fn with_same_components_in_works() {
            let v_frd = vector!(f = mps(1.0), r = mps(2.0), d = mps(3.0); in TestFrd);
            let v_ned: Vector<TestNed, N1> = v_frd.with_same_components_in();

            assert_eq!(v_ned.to_cartesian(), v_frd.to_cartesian());
        }

        #[test]
        fn cast_equivalent_coordinate_systems() {
            let v_frd = vector!(f = mps(1.0), r = mps(2.0), d = mps(3.0); in TestFrd);
            let v_frd2: Vector<TestFrd2, N1> = v_frd.cast();

            assert_eq!(v_frd2.to_cartesian(), v_frd.to_cartesian());
        }
    }

    // Tests for Acceleration vectors (Time = N2)
    mod acceleration_vectors {
        use super::*;

        #[test]
        fn constructor_from_cartesian_works() {
            #[allow(deprecated)]
            let v = Vector::<TestFrd, N2>::from_cartesian(mps2(1.0), mps2(2.0), mps2(3.0));
            assert_eq!(v.to_cartesian(), [mps2(1.0), mps2(2.0), mps2(3.0)]);
        }

        #[test]
        fn build_with_components_works() {
            use crate::systems::FrdComponents;
            let v = Vector::<TestFrd, N2>::build(FrdComponents {
                front: mps2(1.0),
                right: mps2(2.0),
                down: mps2(3.0),
            });
            let cartesian = v.to_cartesian();
            assert_eq!(cartesian, [mps2(1.0), mps2(2.0), mps2(3.0)]);
        }

        #[test]
        fn magnitude_works() {
            let v = vector!(f = mps2(3.0), r = mps2(4.0), d = mps2(0.0); in TestFrd);
            assert_eq!(v.magnitude(), mps2(5.0));
        }

        #[test]
        fn from_spherical_works() {
            let v = Vector::<TestFrd, N2>::from_spherical(
                mps2(10.0),
                Angle::new::<degree>(90.0), // polar angle
                Angle::new::<degree>(0.0),  // azimuth angle
            );
            let cartesian = v.to_cartesian();
            assert_abs_diff_eq!(
                cartesian[0].get::<meter_per_second_squared>(),
                10.0,
                epsilon = 1e-10
            );
            assert_abs_diff_eq!(
                cartesian[1].get::<meter_per_second_squared>(),
                0.0,
                epsilon = 1e-10
            );
            assert_abs_diff_eq!(
                cartesian[2].get::<meter_per_second_squared>(),
                0.0,
                epsilon = 1e-10
            );
        }

        #[test]
        fn axis_vectors_work() {
            let front = Vector::<TestFrd, N2>::frd_front_axis();
            let right = Vector::<TestFrd, N2>::frd_right_axis();
            let down = Vector::<TestFrd, N2>::frd_down_axis();

            let front_cart = front.to_cartesian();
            let right_cart = right.to_cartesian();
            let down_cart = down.to_cartesian();

            assert_eq!(front_cart, [mps2(1.0), mps2(0.0), mps2(0.0)]);
            assert_eq!(right_cart, [mps2(0.0), mps2(1.0), mps2(0.0)]);
            assert_eq!(down_cart, [mps2(0.0), mps2(0.0), mps2(1.0)]);
        }

        #[test]
        fn arithmetic_operations_work() {
            let v1 = vector!(f = mps2(1.0), r = mps2(2.0), d = mps2(3.0); in TestFrd);
            let v2 = vector!(f = mps2(4.0), r = mps2(5.0), d = mps2(6.0); in TestFrd);

            let sum = v1 + v2;
            assert_eq!(sum.to_cartesian(), [mps2(5.0), mps2(7.0), mps2(9.0)]);

            let diff = v2 - v1;
            assert_eq!(diff.to_cartesian(), [mps2(3.0), mps2(3.0), mps2(3.0)]);

            let neg = -v1;
            assert_eq!(neg.to_cartesian(), [mps2(-1.0), mps2(-2.0), mps2(-3.0)]);

            let scaled = v1 * 2.0;
            assert_eq!(scaled.to_cartesian(), [mps2(2.0), mps2(4.0), mps2(6.0)]);
        }

        #[test]
        fn zero_vector_works() {
            let v = Vector::<TestFrd, N2>::zero();
            assert_eq!(v.to_cartesian(), [mps2(0.0), mps2(0.0), mps2(0.0)]);
        }

        #[test]
        fn lerp_works() {
            let v1 = vector!(f = mps2(0.0), r = mps2(0.0), d = mps2(0.0); in TestFrd);
            let v2 = vector!(f = mps2(10.0), r = mps2(20.0), d = mps2(30.0); in TestFrd);

            let mid = v1.lerp(&v2, 0.5);
            assert_eq!(mid.to_cartesian(), [mps2(5.0), mps2(10.0), mps2(15.0)]);
        }

        #[test]
        fn with_same_components_in_works() {
            let v_frd = vector!(f = mps2(1.0), r = mps2(2.0), d = mps2(3.0); in TestFrd);
            let v_ned: Vector<TestNed, N2> = v_frd.with_same_components_in();

            assert_eq!(v_ned.to_cartesian(), v_frd.to_cartesian());
        }

        #[test]
        fn cast_equivalent_coordinate_systems() {
            let v_frd = vector!(f = mps2(1.0), r = mps2(2.0), d = mps2(3.0); in TestFrd);
            let v_frd2: Vector<TestFrd2, N2> = v_frd.cast();

            assert_eq!(v_frd2.to_cartesian(), v_frd.to_cartesian());
        }
    }

    // Tests for coordinate/vector interconversion
    mod coordinate_vector_conversions {
        use super::*;

        #[test]
        fn coordinate_to_vector_conversion_works() {
            let coord = coordinate!(f = m(1.0), r = m(2.0), d = m(3.0); in TestFrd);
            let vec: Vector<TestFrd> = coord.into();

            assert_eq!(vec.to_cartesian(), [m(1.0), m(2.0), m(3.0)]);
        }

        #[test]
        fn multiply_acceleration_by_time_gives_velocity() {
            let acc = vector!(f = mps2(1.0), r = mps2(2.0), d = mps2(3.0); in TestFrd);
            let vel: Vector<TestFrd, N1> =
                acc * uom::si::f64::Time::new::<uom::si::time::second>(2.0);
            assert_eq!(vel.to_cartesian(), [mps(2.0), mps(4.0), mps(6.0)]);
        }

        #[test]
        fn multiply_velocity_by_time_gives_length() {
            let vel = vector!(f = mps(1.0), r = mps(2.0), d = mps(3.0); in TestFrd);
            let len: Vector<TestFrd, Z0> =
                vel * uom::si::f64::Time::new::<uom::si::time::second>(2.0);
            assert_eq!(len.to_cartesian(), [m(2.0), m(4.0), m(6.0)]);
        }
    }
}
