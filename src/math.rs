//! Spatial operations expressed in mathematical language.
//!
//! This module provides type-safe wrappers around the mathematical constructs that underpin rigid
//! body transforms (quaternions, isometries, etc.). The mathematical representation gives maximal
//! power and flexibility to construct, combine, and use these transforms, but can be hard to grok
//! for people without a mathematics-oriented background. For an interface better suited for those
//! with a background in engineering, see [`mod engineering`](crate::engineering).
//!
//! The main type provided by this module is [`RigidBodyTransform`], which describes the
//! isometry (ie, rotation and translation) between two coordinate systems (see also
//! [`CoordinateSystem`]). [`RigidBodyTransform`] implements the various mathematical operations
//! you would expect such that you can multiply them together to combine them, take one's inverse,
//! and multiply with a [`Coordinate`] or [`Vector`] to apply the isometry to transform between
//! coordinate systems.
//!
//! This module also provides the [`Rotation`] type to represents unit quaternion based transforms
//! between coordinate systems (ie, they need no translation to convert between them).

use crate::coordinate_systems::Ecef;
use crate::coordinates::Coordinate;
use crate::geodedic::Wgs84;
use crate::systems::EquivalentTo;
use crate::vectors::Vector;
use crate::Bearing;
use crate::{systems::NedLike, CoordinateSystem, Isometry3, UnitQuaternion};
use nalgebra::{Matrix3, Rotation3, Translation3};
use std::convert::From;
use std::fmt;
use std::fmt::{Display, Formatter};
use std::marker::PhantomData;
use std::ops::{Mul, Neg};
use uom::si::angle::radian;
use uom::si::f64::Angle;

#[cfg(any(test, feature = "approx"))]
use approx::{AbsDiffEq, RelativeEq};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(doc)]
use crate::{engineering, systems::FrdLike};

/// Defines a [rotation transform] between two [`CoordinateSystem`]s.
///
/// There are generally two ways to construct a rotation:
///
/// 1. by using [`engineering::Orientation::map_as_zero_in`] when you already have an
///    [`engineering::Orientation`];
/// 2. by using [Tait-Bryan angles](Rotation::from_tait_bryan_angles) to construct
///    arbitrary rotations.
///
/// Mathematically speaking, this is a type-safe wrapper around a [unit quaternion].
///
/// <div class="warning">
///
/// Note that this type implements `Deserialize` despite having `unsafe` constructors -- this is
/// because doing otherwise would be extremely unergonomic. However, when deserializing, the
/// coordinate system of the deserialized value is _not_ checked, so this is a foot-gun to be
/// mindful of.
///
/// </div>
///
/// <div class="warning">
///
/// Rotations can be chained with other transformations using `*` (ie, the [`Mul`] trait). However,
/// note that the order of the operands to `*` matter, and do not match the mathematical
/// convention. Specifically, matrix multiply for transforms (like rotations) traditionally have
/// the transform on the left and the vector to transform on the right. However, doing so here
/// would lead to a type signature of
///
/// ```rust,ignore
/// let _: Coordinate<To> = Rotation<From, To> * Coordinate<From>;
/// ```
///
/// Which violates the expectation that a matrix multiply eliminates the "middle" component (ie,
/// (m × n)(n × p) = (m × p)). So, we require that the rotation is on the _right_ to go from
/// `From` into `To`, and that the rotation is on the _left_ to go from `To` into `From` (ie, for
/// the inverse transform).
///
/// </div>
///
/// [rotation transform]: https://en.wikipedia.org/wiki/Rotation
/// [unit quaternion]: https://en.wikipedia.org/wiki/Versor
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
// don't require From/To: Serialize/Deserialize since we skip it anyway
#[cfg_attr(feature = "serde", serde(bound = ""))]
// no need for the "inner": indirection
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct Rotation<From, To> {
    /// This is _actually_ the rotation from `To` into `From`, because that's a much more natural
    /// constructor. This means that what we store here is equivalent to a rotation matrix whose
    /// rows are `From` and columns are `To` (rather than the other way around). If we want convert
    /// a vector `x` from `From` to `To`, we can't do a straight matrix multiplication of the
    /// (equivalent) rotation matrix in `inner` with `x`, because that would yield (From x To) *
    /// (From x 1), which doesn't math. Instead, we need to matrix multiply the _inverse_ (which is
    /// transpose for a rotation matrix) of `inner` with `x`, which does work:
    ///
    /// ```text
    /// (From x To)-1 * (From x 1)
    /// (To x From) * (From x 1)
    /// (To x 1)
    /// ```
    ///
    /// In other words, we have to apply `inverse_transform`, not `transform`, for all conversions
    /// `From` -> `To`. If we want to go the other way (`To` -> `From`), we need to apply
    /// `transform`. You'll see this across the `impl Mul`s further down.
    pub(crate) inner: UnitQuaternion,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) from: PhantomData<From>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) to: PhantomData<To>,
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<From, To> Clone for Rotation<From, To> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<From, To> Copy for Rotation<From, To> {}

impl<From, To> PartialEq<Self> for Rotation<From, To> {
    fn eq(&self, other: &Self) -> bool {
        self.inner.eq(&other.inner)
    }
}

impl<From, To> Display for Rotation<From, To> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "Quaternion: {}", self.inner)
    }
}

impl<To> Rotation<Ecef, To>
where
    To: CoordinateSystem<Convention = NedLike>,
{
    /// Constructs the rotation from [`Ecef`] to [`NedLike`] at the given latitude and longitude.
    ///
    /// The lat/lon is needed because the orientation of North with respect to ECEF depends on
    /// where on the globe you are.
    ///
    /// See also
    /// <https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates>.
    ///
    /// # Safety
    ///
    /// This function only produces a valid transform from [`Ecef`] to the NED-like `To` if
    /// [`Coordinate::<To>::origin()`](Coordinate::origin) lies at the provided lat/lon. If that is
    /// not the case, the returned `Rotation` will allow moving from [`Ecef`] to `To` without the
    /// appropriate transformation of the components of the input, leading to outputs that are
    /// typed to be in `To` but are in fact not.
    ///
    /// Furthermore, this function only applies rotation and _not_ translation, and is therefore
    /// not appropriate for turning [`Ecef`] into `To` on its own. See
    /// [`RigidBodyTransform::ecef_to_ned_at`].
    unsafe fn ecef_to_ned_at(
        latitude: impl Into<Angle>,
        longitude: impl Into<Angle>,
    ) -> Rotation<Ecef, To> {
        let phi = latitude.into().get::<radian>();
        let lambda = longitude.into().get::<radian>();

        let sin_phi = phi.sin();
        let cos_phi = phi.cos();
        let sin_lambda = lambda.sin();
        let cos_lambda = lambda.cos();

        // Equation 3 has column E, N, U
        // With N, E, D = N, E, -U
        // this yields
        let matrix = Matrix3::new(
            -cos_lambda * sin_phi,
            -sin_lambda,
            -cos_lambda * cos_phi,
            -sin_lambda * sin_phi,
            cos_lambda,
            -sin_lambda * cos_phi,
            cos_phi,
            0.,
            -sin_phi,
        );
        let rot = Rotation3::from_matrix(&matrix);

        Rotation {
            inner: UnitQuaternion::from_rotation_matrix(&rot),
            from: PhantomData,
            to: PhantomData,
        }
    }
}

impl<From, To> Rotation<From, To> {
    /// Constructs a rotation between two [`CoordinateSystem`]s using ([intrinsic]) yaw, pitch, and
    /// roll [Tait-Bryan angles][tb].
    ///
    /// Perhaps counter-intuitively, this is the rotation _of_ `From` _in_ `To`. That is, it is the
    /// rotation that must be applied to points in `To` to place them in `From`.
    ///
    /// If your brain is more engineering-oriented, prefer the alternatives listed in the [type
    /// docs](Rotation).
    ///
    /// Perhaps counter-intuitively, the angles provided here should represent the rotation that
    /// must be applied to an object with zero [`Bearing`] **in `To`** to arrivate at a [`Bearing`]
    /// with the given angles **in `From`**. This is because we take the _inverse_ of this rotation
    /// to go from `From` into `To`, which in turn stems from mathematical norms.
    ///
    /// The three rotations are defined as follows:
    ///
    /// - yaw is rotation about the Z axis of `In`;
    /// - pitch is rotation about the Y axis; and
    /// - roll is rotation about the X axis.
    ///
    /// Since we are using [intrinsic] rotations (by convention given the naming of the arguments),
    /// pitch is with respect to the Y axis _after_ applying the yaw, and roll is with respect to X
    /// after applying both yaw and pitch.
    ///
    /// To determine the direction of rotation (ie, in which direction a positive angle goes), you
    /// can use the [right-hand rule for rotations][rhrot]: curl your fingers and stick your thumb
    /// out in the positive direction of the axis you want to check rotation around (eg, positive Z
    /// for yaw). The direction your fingers curl is the direction of (positive) rotation.
    ///
    /// Be aware that rotational angles have high ambiguities in literature and are easy to use
    /// wrong, especially because different fields tend to use the same term with different
    /// meanings (eg, "Euler angles" mean something else in aerospace than in mathematics).
    ///
    /// See also [`engineering::Orientation::from_tait_bryan_angles`].
    ///
    /// [intrinsic]: https://dominicplein.medium.com/extrinsic-intrinsic-rotation-do-i-multiply-from-right-or-left-357c38c1abfd
    /// [tb]: https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
    /// [aero]: https://en.wikipedia.org/wiki/Aircraft_principal_axes
    /// [enu]: https://en.wikipedia.org/wiki/Axes_conventions#World_reference_frames_for_attitude_description
    /// [rhrot]: https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
    ///
    /// # Safety
    ///
    /// Calling this method asserts that the provided rotational angles represent a correct way to
    /// transform any input from `From` to `To` (and crucially, that no translation is needed). If
    /// this is _not_ the correct transform, then this allows moving values between different
    /// coordinate system types without adjusting the values correctly, leading to a defeat of
    /// their type safety.
    #[doc(alias = "from_nautical_angles")]
    #[doc(alias = "from_cardan_angles")]
    #[doc(alias = "from_ypr")]
    pub unsafe fn from_tait_bryan_angles(
        yaw: impl Into<Angle>,
        pitch: impl Into<Angle>,
        roll: impl Into<Angle>,
    ) -> Self {
        Self {
            // we have intrinstic Tait-Bryan angles, and we want to construct a unit quaternion.
            // Looking at the implementation, nalgebra::UnitQuaternion::from_euler_angles uses the
            // same construction as Wikipedia gives for "Euler angles (in 3-2-1 sequence) to
            // quaternion conversion":
            //
            // <https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_(in_3-2-1_sequence)_to_quaternion_conversion>
            //
            // suggesting that it uses 3-2-1, so z-y-x, so Tait-Bryan not "classic" Euler angles.
            // furthermore, the naming of the arguments uses the words "roll", "pitch", and "yaw",
            // which suggests they represent z, y', and x'' (by convention), which matches the
            // documentation we've written for this function. so, we can simply pass our arguments
            // in unchanged (minding the order).
            //
            // it would be nice if nalgebra's docs were more explicit about this. if they do,
            // probably through one of these issues, we should update reasoning here accordingly to
            // instill more confidence.
            //
            // - https://github.com/dimforge/nalgebra/issues/1294
            // - https://github.com/dimforge/nalgebra/issues/1446
            inner: UnitQuaternion::from_euler_angles(
                roll.into().get::<radian>(),
                pitch.into().get::<radian>(),
                yaw.into().get::<radian>(),
            ),
            from: PhantomData::<From>,
            to: PhantomData::<To>,
        }
    }

    /// Asserts that the rotational transform from `From` to `To` is one that returns the original
    /// point or vector when applied.
    ///
    /// # Safety
    ///
    /// Like [`RigidBodyTransform::identity`], this allows you to claim that the "correct"
    /// rotational transform between the coordinate systems `From` and `To` is the identity
    /// function (and that no translation is needed). If this is _not_ the correct transform, then
    /// this allows moving values between different coordinate system types without adjusting the
    /// values correctly, leading to a defeat of their type safety.
    #[must_use]
    pub unsafe fn identity() -> Self {
        Self {
            inner: UnitQuaternion::identity(),
            from: PhantomData::<From>,
            to: PhantomData::<To>,
        }
    }

    /// Changes the target coordinate system of the rotation to `<NewTo>` with no changes to the
    /// rotational angles.
    ///
    /// This is useful when you have two coordinate systems that you know have exactly equivalent
    /// axes, and you need the types to "work out". This can be the case, for instance, when two
    /// crates have both separately declared a NED-like coordinate system centered on the same
    /// object, and you have a rotation from some coordinate system into one of them, but you want
    /// the end type to be in the other.
    ///
    /// This is not how you _generally_ want to convert between coordinate systems. For that,
    /// you'll want to use [`RigidBodyTransform`].
    ///
    /// This is exactly equivalent to re-constructing the rotation with the same rotational angles
    /// using `<NewTo>` instead of `<To>`, just more concise and legible. That is, it is exactly
    /// equal to:
    ///
    /// ```
    /// use sguaba::{system, math::Rotation};
    /// use uom::si::f64::Angle;
    /// use uom::si::angle::degree;
    /// use approx::assert_relative_eq;
    ///
    /// system!(struct PlaneFrd using NED);
    /// system!(struct PlaneNedFromCrate1 using NED);
    /// system!(struct PlaneNedFromCrate2 using NED);
    ///
    /// // SAFETY
    /// // we're claiming, without any factual basis for doing so,
    /// // that this _is_ the plane's orientation.
    /// let rotation_into_1 = unsafe {
    ///     Rotation::<PlaneFrd, PlaneNedFromCrate1>::from_tait_bryan_angles(
    ///       Angle::new::<degree>(30.),
    ///       Angle::new::<degree>(45.),
    ///       Angle::new::<degree>(90.),
    ///     )
    /// };
    ///
    /// let (yaw, pitch, roll) = rotation_into_1.to_tait_bryan_angles();
    /// assert_relative_eq!(
    ///     unsafe { Rotation::<PlaneFrd, PlaneNedFromCrate2>::from_tait_bryan_angles(yaw, pitch, roll) },
    ///     unsafe { rotation_into_1.is_also_into::<PlaneNedFromCrate2>() }
    /// );
    /// ```
    ///
    /// # Safety
    ///
    /// This asserts that the transform `From` to `To` is the same as the transform from `From` to
    /// `NewTo`. However, if that is _not_ the case, the resulting transform would allow violating
    /// type safety by moving, say, a coordinate into `NewTo` without correctly adjusting its
    /// values.
    #[must_use]
    pub unsafe fn is_also_into<NewTo>(self) -> Rotation<From, NewTo> {
        Rotation {
            inner: self.inner,
            from: self.from,
            to: PhantomData::<NewTo>,
        }
    }

    /// Changes the origin coordinate system of the rotation to `<NewFrom>` with no changes to the
    /// rotational angles.
    ///
    /// This is useful when you have two coordinate systems that you know have exactly equivalent
    /// axes, and you need the types to "work out". This can be the case, for instance, when two
    /// crates have both separately declared a NED-like coordinate system centered on the same
    /// object, and you have a rotation into some coordinate system from one of them, but need to
    /// transform from something typed to be in the other.
    ///
    /// This is not how you _generally_ want to convert between coordinate systems. For that,
    /// you'll want to use [`RigidBodyTransform`].
    ///
    /// This is exactly equivalent to re-constructing the rotation with the same rotational angles
    /// using `<NewFrom>` instead of `<From>`, just more concise and legible. That is, it is exactly
    /// equal to:
    ///
    /// ```
    /// use sguaba::{system, math::Rotation};
    /// use uom::si::f64::Angle;
    /// use uom::si::angle::degree;
    /// use approx::assert_relative_eq;
    ///
    /// system!(struct PlaneFrd using NED);
    /// system!(struct PlaneNedFromCrate1 using NED);
    /// system!(struct PlaneNedFromCrate2 using NED);
    ///
    /// // SAFETY
    /// // we're claiming, without any factual basis for doing so,
    /// // that this _is_ the plane's orientation.
    /// let rotation_into_1 = unsafe {
    ///     Rotation::<PlaneNedFromCrate1, PlaneFrd>::from_tait_bryan_angles(
    ///       Angle::new::<degree>(30.),
    ///       Angle::new::<degree>(45.),
    ///       Angle::new::<degree>(90.),
    ///     )
    /// };
    ///
    /// let (yaw, pitch, roll) = rotation_into_1.to_tait_bryan_angles();
    /// assert_relative_eq!(
    ///     unsafe { Rotation::<PlaneNedFromCrate2, PlaneFrd>::from_tait_bryan_angles(yaw, pitch, roll) },
    ///     unsafe { rotation_into_1.is_also_from::<PlaneNedFromCrate2>() }
    /// );
    /// ```
    ///
    /// # Safety
    ///
    /// This asserts that the transform `From` to `To` is the same as the transform from `NewFrom`
    /// to `To`. However, if that is _not_ the case, the resulting transform would allow violating
    /// type safety by moving, say, a coordinate from `NewFrom` without correctly adjusting its
    /// values.
    #[must_use]
    pub unsafe fn is_also_from<NewFrom>(self) -> Rotation<NewFrom, To> {
        Rotation {
            inner: self.inner,
            from: PhantomData::<NewFrom>,
            to: self.to,
        }
    }

    /// Casts the coordinate system type parameter `From` of the rotation to the equivalent
    /// coordinate system `AlsoFrom`.
    ///
    /// See [`EquivalentTo`] for details on when this is useful (and safe).
    ///
    /// Note that this performs no modifications to the transform itself, as that should be
    /// unnecessary when `EquivalentTo` is implemented.
    #[must_use]
    pub fn cast_type_of_from<AlsoFrom>(self) -> Rotation<AlsoFrom, To>
    where
        From: EquivalentTo<AlsoFrom>,
    {
        Rotation {
            inner: self.inner,
            from: PhantomData::<AlsoFrom>,
            to: self.to,
        }
    }

    /// Casts the coordinate system type parameter `To` of the rotation to the equivalent
    /// coordinate system `AlsoTo`.
    ///
    /// See [`EquivalentTo`] for details on when this is useful (and safe).
    ///
    /// Note that this performs no modifications to the rotation itself, as that should be
    /// unnecessary when `EquivalentTo` is implemented.
    #[must_use]
    pub fn cast_type_of_to<AlsoTo>(self) -> Rotation<From, AlsoTo>
    where
        To: EquivalentTo<AlsoTo>,
    {
        Rotation {
            inner: self.inner,
            from: self.from,
            to: PhantomData::<AlsoTo>,
        }
    }

    /// Returns the equal-but-opposite transform to this one.
    ///
    /// That is, a rotation _from_ the [`CoordinateSystem`] `To` _into_ the coordinate system
    /// `From`.
    #[must_use]
    pub fn inverse(&self) -> Rotation<To, From> {
        Rotation {
            inner: self.inner.inverse(),
            from: PhantomData::<To>,
            to: PhantomData::<From>,
        }
    }

    /// Linearly interpolate between this rotation and another one.
    ///
    /// Conceptually returns `self * (1.0 - t) + rhs * t`, i.e., the linear blend of the two
    /// rotations using the scalar value `t`.
    ///
    /// Note that this function inherently normalizes the underlying rotation such that it remains
    /// a unit quaternion.
    ///
    /// The value for `t` is not restricted to the range [0, 1].
    #[must_use]
    pub fn nlerp(&self, rhs: &Self, t: f64) -> Self {
        Self {
            inner: self.inner.nlerp(&rhs.inner, t),
            from: self.from,
            to: self.to,
        }
    }

    /// Returns the yaw-pitch-roll [Tait-Bryan angles][tb] that describe this rotation.
    ///
    /// See [`Rotation::from_tait_bryan_angles`] for documentation about the exact meaning of yaw,
    /// pitch, and roll here.
    ///
    /// The angles returned are, perhaps counter-intuitively but also in alignment with
    /// [`Rotation::from_tait_bryan_angles`], the rotations that must be performed to go from `To`
    /// _into_ `From`.
    ///
    /// [tb]: https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
    #[must_use]
    pub fn to_tait_bryan_angles(&self) -> (Angle, Angle, Angle) {
        let (roll, pitch, yaw) = self.inner.euler_angles();
        (
            Angle::new::<radian>(yaw),
            Angle::new::<radian>(pitch),
            Angle::new::<radian>(roll),
        )
    }

    /// Returns the [Euler angles] describing this rotation.
    ///
    /// The use of this method is discouraged as Eulers angles are both hard to work with and do
    /// not have a single, canonical definition across domains (eg, [in aerospace]).
    ///
    /// The angles returned are, perhaps counter-intuitively but in alignment with
    /// [`Rotation::from_tait_bryan_angles`], the rotations that must be performed to go
    /// from `To` _into_ `From`.
    ///
    /// [Euler angles]: https://en.wikipedia.org/wiki/Euler_angles
    /// [in aerospace]: https://en.wikipedia.org/wiki/Aircraft_principal_axes
    #[must_use]
    pub fn euler_angles(&self) -> (Angle, Angle, Angle) {
        let (roll, pitch, yaw) = self.inner.euler_angles();
        (
            Angle::new::<radian>(yaw),
            Angle::new::<radian>(pitch),
            Angle::new::<radian>(roll),
        )
    }
}

impl<From, To> Rotation<From, To> {
    /// Transforms an element in [`CoordinateSystem`] `From` into `To`.
    #[doc(alias = "apply")]
    pub fn transform<T>(&self, in_from: T) -> <T as Mul<Self>>::Output
    where
        T: Mul<Self>,
    {
        in_from * *self
    }

    /// Transforms an element in [`CoordinateSystem`] `To` into `From`.
    ///
    /// This is equivalent to (but more efficient than) first inverting the transform with
    /// [`RigidBodyTransform::inverse`] and then calling [`RigidBodyTransform::transform`].
    #[doc(alias = "undo")]
    pub fn inverse_transform<T>(&self, in_to: T) -> <Self as Mul<T>>::Output
    where
        Self: Mul<T>,
    {
        *self * in_to
    }
}

#[cfg(any(test, feature = "approx"))]
impl<From, To> AbsDiffEq<Self> for Rotation<From, To> {
    type Epsilon = <f64 as AbsDiffEq>::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        UnitQuaternion::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.inner.abs_diff_eq(&other.inner, epsilon)
    }
}

#[cfg(any(test, feature = "approx"))]
impl<From, To> RelativeEq for Rotation<From, To> {
    fn default_max_relative() -> Self::Epsilon {
        UnitQuaternion::default_max_relative()
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

impl<From, To> Neg for Rotation<From, To> {
    type Output = Rotation<To, From>;

    fn neg(self) -> Self::Output {
        self.inverse()
    }
}

// Rotation<From, Over> * Rotation<Over, To> -> Rotation<From, To>
//
// self.inner is equivalent to a rotation matrix whose rows are `From` and columns are `Over`.
// rhs.inner is equivalent to a rotation matrix whose rows are `Over` and columns are `To`
// to arrive at a matrix whose rows are `From` and columns are `To` (which is what we store in
// `Rotation<From, To>`), we need:
//
//     (From x Over) * (Over x To)
//
// which is then self.inner * rhs.inner.
impl<From, Over, To> Mul<Rotation<Over, To>> for Rotation<From, Over> {
    type Output = Rotation<From, To>;

    fn mul(self, rhs: Rotation<Over, To>) -> Self::Output {
        Self::Output {
            inner: self.inner * rhs.inner,
            from: self.from,
            to: rhs.to,
        }
    }
}

// Rotation<From, To> * Bearing<To> -> Bearing<From>
impl<From, To> Mul<Bearing<To>> for Rotation<From, To>
where
    From: crate::systems::BearingDefined,
    To: crate::systems::BearingDefined,
{
    type Output = Bearing<From>;

    fn mul(self, rhs: Bearing<To>) -> Self::Output {
        (self * rhs.to_unit_vector())
            .bearing_at_origin()
            .expect("magnitude should still be 1 after rotation")
    }
}

// Bearing<From> * Rotation<From, To> -> Bearing<To>
impl<From, To> Mul<Rotation<From, To>> for Bearing<From>
where
    From: crate::systems::BearingDefined,
    To: crate::systems::BearingDefined,
{
    type Output = Bearing<To>;

    fn mul(self, rhs: Rotation<From, To>) -> Self::Output {
        (self.to_unit_vector() * rhs)
            .bearing_at_origin()
            .expect("magnitude should still be 1 after rotation")
    }
}

/// Defines a [rigid body transform][isometry] between two [`CoordinateSystem`]s.
///
/// A rigid transform may include both translation and [rotation](Rotation).
///
/// There are generally three ways to construct a rigid body transform:
///
/// 1. by using [`engineering::Pose::map_as_zero_in`] when you already have an
///    [`engineering::Pose`];
/// 2. by using [`RigidBodyTransform::ecef_to_ned_at`] when you wish to translate between [`Ecef`]
///    and a [`NedLike`] [`CoordinateSystem`]; or
/// 3. by using [`RigidBodyTransform::new`] to construct transforms from arbitrary translation and
///    rotation.
///
/// Mathematically speaking, this is a type-safe wrapper around an [Euclidian isometry][isometry].
///
/// Transforms can be chained with other transforms (including [`Rotation`]s) using `*` (ie, the
/// [`Mul`] trait) or with [`RigidBodyTransform::and_then`] (they're equivalent). They can also be
/// applied to [`Coordinate`], [`Vector`], and other types either by multiplication (or division
/// for inverse transforms) or by using [`RigidBodyTransform::transform`] (or
/// [`RigidBodyTransform::inverse_transform`]).
///
/// <div class="warning">
///
/// Note that this type implements `Deserialize` despite having `unsafe` constructors -- this is
/// because doing otherwise would be extremely unergonomic. However, when deserializing, the
/// coordinate system of the deserialized value is _not_ checked, so this is a foot-gun to be
/// mindful of.
///
/// </div>
///
/// <div class="warning">
///
/// The order of the operands to `*` matter, and do not match the mathematical convention.
/// Specifically, matrix multiply for transforms traditionally have the transform on the left and
/// the vector to transform on the right. However, doing so here would lead to a type signature of
///
/// ```rust,ignore
/// let _: Coordinate<To> = RigidBodyTransform<From, To> * Coordinate<From>;
/// ```
///
/// Which violates the expectation that a matrix multiply eliminates the "middle" component (ie,
/// (m × n)(n × p) = (m × p)). So, we require that the transform is on the _right_ to go from
/// `From` into `To`, and that the transform is on the _left_ to go from `To` into `From`.
///
/// </div>
///
/// [isometry]: https://en.wikipedia.org/wiki/Rigid_transformation
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
// don't require From/To: Serialize/Deserialize since we skip it anyway
#[cfg_attr(feature = "serde", serde(bound = ""))]
// no need for the "inner": indirection
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct RigidBodyTransform<From, To> {
    /// This is _actually_ the isometry from `To` into `From`, which means we need to take the
    /// _inverse_ transform to go from `From` into `To`. For more details about this, see the docs
    /// on `Rotation.inner`.
    pub(crate) inner: Isometry3,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) from: PhantomData<From>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) to: PhantomData<To>,
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<From, To> Clone for RigidBodyTransform<From, To> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<From, To> Copy for RigidBodyTransform<From, To> {}

impl<To> RigidBodyTransform<Ecef, To>
where
    To: CoordinateSystem<Convention = NedLike>,
{
    /// Constructs the transformation from [`Ecef`] to [`NedLike`] at the given latitude and
    /// longitude.
    ///
    /// The lat/lon is needed because the orientation of North with respect to ECEF depends on
    /// where on the globe you are, and because the transform of something like (0, 0, 0) in
    /// [`NedLike`] into ECEF (and vice-verse) requires knowing the ECEF coordinates of this
    /// [`NedLike`]'s origin.
    ///
    /// See also
    /// <https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates>.
    ///
    /// # Safety
    ///
    /// This function only produces a valid transform from [`Ecef`] to the NED-like `To` if
    /// [`Coordinate::<To>::origin()`](Coordinate::origin) lies at the provided lat/lon. If that is
    /// not the case, the returned `RigidBodyTransform` will allow moving from [`Ecef`] to `To`
    /// without the appropriate transformation of the components of the input, leading to outputs
    /// that are typed to be in `To` but are in fact not.
    #[must_use]
    pub unsafe fn ecef_to_ned_at(position: &Wgs84) -> Self {
        let ecef = Coordinate::<Ecef>::from_wgs84(position);
        let translation = Vector::from(ecef);

        // SAFETY: same safety caveat as us + we will also apply the necessary translation.
        let rotation = unsafe { Rotation::ecef_to_ned_at(position.latitude, position.longitude) };

        // SAFETY: this is indeed a correct transform from ECEF to an NED at `position`.
        unsafe { Self::new(translation, rotation) }
    }
}

impl<From, To> RigidBodyTransform<From, To> {
    /// Constructs a transform directly from a translation and a rotation.
    ///
    /// The translation here should be what must be applied to coordinates and vectors in `To` to
    /// transform them into coordinates and vectors in `From`. In other words, the _inverse_ of
    /// this translation and rotation will be applied in the `transform` methods to go from `From`
    /// to `To`.
    ///
    /// # Safety
    ///
    /// This method is marked as `unsafe` for the same reason [`engineering::Pose::map_as_zero_in`]
    /// is; if you construct a transform incorrectly, it allows moving between different
    /// type-enforced coordinate systems without performing the correct conversions, leading to a
    /// defeat of type safety.
    #[must_use]
    pub unsafe fn new(translation: Vector<From>, rotation: Rotation<From, To>) -> Self {
        Self {
            inner: Isometry3::from_parts(Translation3::from(translation.inner), rotation.inner),
            from: PhantomData::<From>,
            to: PhantomData::<To>,
        }
    }

    /// Chains two transforms to produce a new transform that can transform directly from `From` to
    /// `NewTo`.
    ///
    /// ```rust
    /// use sguaba::{
    ///     system, Coordinate, Vector,
    ///     math::{RigidBodyTransform, Rotation},
    ///     systems::{Ecef, Wgs84},
    /// };
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct PlaneFrd using FRD);
    /// system!(struct PlaneNed using NED);
    ///
    /// // we can construct a transform from ECEF to PlaneNed
    /// // (we can construct the ECEF from a WGS84 lat/lon)
    /// let location = Wgs84::builder()
    ///     .latitude(Angle::new::<degree>(0.)).expect("latitude is in-range")
    ///     .longitude(Angle::new::<degree>(10.))
    ///     .altitude(Length::new::<meter>(0.))
    ///     .build();
    ///
    /// // SAFETY: we're claiming that `location` is the location of `PlaneNed`'s origin.
    /// let ecef_to_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&location) };
    ///
    /// // we can also construct a (rotation) transform from PlaneNed to PlaneFrd
    /// // assuming we know what direction the plane is facing.
    /// // SAFETY: we're claiming that these angles are the orientation of the plane in NED.
    /// let ned_to_frd = unsafe {
    ///     Rotation::<PlaneNed, PlaneFrd>::from_tait_bryan_angles(
    ///         Angle::new::<degree>(0.),
    ///         Angle::new::<degree>(45.),
    ///         Angle::new::<degree>(23.),
    ///     )
    /// };
    ///
    /// // and we can chain the two to give us a single transform that goes
    /// // from ECEF to PlaneFrd in a single computation.
    /// let ecef_to_frd = ecef_to_ned.and_then(ned_to_frd);
    /// ```
    pub fn and_then<NewTo, Transform>(self, rhs: Transform) -> RigidBodyTransform<From, NewTo>
    where
        Self: Mul<Transform, Output = RigidBodyTransform<From, NewTo>>,
    {
        self * rhs
    }

    /// Asserts that the transform from `From` to `To` is one that returns the original point or
    /// vector when applied.
    ///
    /// # Safety
    ///
    /// This allows you to claim that the "correct" transform between the coordinate systems `From`
    /// and `To` is the identity function. If this is _not_ the correct transform, then this allows
    /// moving values between different coordinate system types without adjusting the values
    /// correctly, leading to a defeat of their type safety.
    #[must_use]
    pub unsafe fn identity() -> Self {
        Self::new(Vector::zero(), Rotation::identity())
    }

    /// Casts the coordinate system type parameter `From` of the transform to the equivalent
    /// coordinate system `AlsoFrom`.
    ///
    /// See [`EquivalentTo`] for details on when this is useful (and safe).
    ///
    /// Note that this performs no modifications to the transform itself, as that should be
    /// unnecessary when `EquivalentTo` is implemented.
    #[must_use]
    pub fn cast_type_of_from<AlsoFrom>(self) -> RigidBodyTransform<AlsoFrom, To>
    where
        From: EquivalentTo<AlsoFrom>,
    {
        RigidBodyTransform {
            inner: self.inner,
            from: PhantomData::<AlsoFrom>,
            to: self.to,
        }
    }

    /// Casts the coordinate system type parameter `To` of the transform to the equivalent
    /// coordinate system `AlsoTo`.
    ///
    /// See [`EquivalentTo`] for details on when this is useful (and safe).
    ///
    /// Note that this performs no modifications to the transform itself, as that should be
    /// unnecessary when `EquivalentTo` is implemented.
    #[must_use]
    pub fn cast_type_of_to<AlsoTo>(self) -> RigidBodyTransform<From, AlsoTo>
    where
        To: EquivalentTo<AlsoTo>,
    {
        RigidBodyTransform {
            inner: self.inner,
            from: self.from,
            to: PhantomData::<AlsoTo>,
        }
    }

    /// Returns the equal-but-opposite transform to this one.
    ///
    /// That is, a transform _from_ the [`CoordinateSystem`] `To` _into_ the coordinate system
    /// `From`.
    #[must_use]
    pub fn inverse(&self) -> RigidBodyTransform<To, From> {
        RigidBodyTransform::<To, From> {
            inner: self.inner.inverse(),
            from: PhantomData,
            to: PhantomData,
        }
    }

    /// Linearly interpolate between this transform and another one.
    ///
    /// Conceptually returns `self * (1.0 - t) + rhs * t`, i.e., the linear blend of the two
    /// transforms using the scalar value `t`.
    ///
    /// Internally, this linearly interpolates the translation and rotation separately, and
    /// normalizes the rotation in the process (see [`Rotation::nlerp`]).
    ///
    /// The value for `t` is not restricted to the range [0, 1].
    #[must_use]
    pub fn lerp(&self, rhs: &Self, t: f64) -> Self {
        // SAFETY: if `self` and `rhs` are both correct transforms from `From` to `To`, lerping
        // between them should also be a valid transform.
        unsafe {
            Self::new(
                self.translation().lerp(&rhs.translation(), t),
                self.rotation().nlerp(&rhs.rotation(), t),
            )
        }
    }

    /// Returns the translation of [`Coordinate::origin`] in `To` relative to the
    /// [`Coordinate::origin`] in `From`.
    ///
    /// That is, perhaps counter-intuitively but in alignment with [`RigidBodyTransform::new`], the
    /// translation that must be performed to go from `To` _into_ `From`.
    #[must_use]
    pub fn translation(&self) -> Vector<From> {
        Vector::from_nalgebra_vector(self.inner.translation.vector)
    }

    /// Returns the rotation of the coordinate system `To` with respect to the coordinate system
    /// `From`.
    #[must_use]
    pub fn rotation(&self) -> Rotation<From, To> {
        Rotation {
            inner: self.inner.rotation,
            from: self.from,
            to: self.to,
        }
    }
}

impl<From, To> RigidBodyTransform<From, To> {
    /// Transforms an element in [`CoordinateSystem`] `From` into `To`.
    ///
    /// <div class="warning">
    ///
    /// Note that this transformation behaves differently for vectors than it does for coordinates.
    /// Following the convention in computer vision, a vector defines a displacement _without an
    /// explicit origin_. Thus, when transformed into a different coordinate system the
    /// displacement points in a different direction _but retains its length_. In other words,
    /// vectors are only subjected to the rotation part of the transform.
    ///
    /// Note further that a bearing is transformed exactly the way it would be if it were a unit
    /// vector in the direction of the bearing.
    ///
    /// </div>
    #[doc(alias = "apply")]
    pub fn transform<T>(&self, in_from: T) -> <T as Mul<Self>>::Output
    where
        T: Mul<Self>,
    {
        in_from * *self
    }

    /// Transforms an element in [`CoordinateSystem`] `To` into `From`.
    ///
    /// This is equivalent to (but more efficient than) first inverting the transform with
    /// [`RigidBodyTransform::inverse`] and then calling [`RigidBodyTransform::transform`].
    ///
    /// <div class="warning">
    ///
    /// Note that this transformation behaves differently for vectors than it does for coordinates.
    /// Following the convention in computer vision, a vector defines a displacement _without an
    /// explicit origin_. Thus, when transformed into a different coordinate system the
    /// displacement points in a different direction _but retains its length_. In other words,
    /// vectors are only subjected to the rotation part of the transform.
    ///
    /// Note further that a bearing is transformed exactly the way it would be if it were a unit
    /// vector in the direction of the bearing.
    ///
    /// </div>
    #[doc(alias = "undo")]
    pub fn inverse_transform<T>(&self, in_to: T) -> <Self as Mul<T>>::Output
    where
        Self: Mul<T>,
    {
        *self * in_to
    }
}

impl<From, To> PartialEq<Self> for RigidBodyTransform<From, To> {
    fn eq(&self, other: &Self) -> bool {
        self.inner.eq(&other.inner)
    }
}

impl<From, To> Display for RigidBodyTransform<From, To> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Position: {}, Orientation: {}",
            self.translation(),
            self.rotation()
        )
    }
}

impl<From, To> Neg for RigidBodyTransform<From, To> {
    type Output = RigidBodyTransform<To, From>;

    fn neg(self) -> Self::Output {
        self.inverse()
    }
}

// recall from the docs on `Rotation.inner` that conversions of `From` -> `To` use _inverse_
// transforms, and conversions from `To` -> `From` use regular `transform`.
//
// further recall that we want to have eliminated "dimensions" be adjacent in the input to match
// matrix expectations (ie, (m × n)(n × p) = (m × p)).

// Coordinate<From> * Rotation<From, To> -> Coordinate<To>
impl<From, To> Mul<Rotation<From, To>> for Coordinate<From> {
    type Output = Coordinate<To>;

    fn mul(self, rhs: Rotation<From, To>) -> Self::Output {
        Coordinate::from_nalgebra_point(rhs.inner.inverse_transform_point(&self.point))
    }
}

// Rotation<From, To> * Coordinate<To> -> Coordinate<From>
impl<From, To> Mul<Coordinate<To>> for Rotation<From, To> {
    type Output = Coordinate<From>;

    fn mul(self, rhs: Coordinate<To>) -> Self::Output {
        Coordinate::from_nalgebra_point(self.inner.transform_point(&rhs.point))
    }
}

impl<From, To> Mul<Rotation<From, To>> for Vector<From> {
    type Output = Vector<To>;

    fn mul(self, rhs: Rotation<From, To>) -> Self::Output {
        Vector::from_nalgebra_vector(rhs.inner.inverse_transform_vector(&self.inner))
    }
}

impl<From, To> Mul<Vector<To>> for Rotation<From, To> {
    type Output = Vector<From>;

    fn mul(self, rhs: Vector<To>) -> Self::Output {
        Vector::from_nalgebra_vector(self.inner.transform_vector(&rhs.inner))
    }
}

impl<From, To> Mul<RigidBodyTransform<From, To>> for Coordinate<From> {
    type Output = Coordinate<To>;

    fn mul(self, rhs: RigidBodyTransform<From, To>) -> Self::Output {
        Coordinate::from_nalgebra_point(rhs.inner.inverse_transform_point(&self.point))
    }
}

impl<From, To> Mul<Coordinate<To>> for RigidBodyTransform<From, To> {
    type Output = Coordinate<From>;

    fn mul(self, rhs: Coordinate<To>) -> Self::Output {
        Coordinate::from_nalgebra_point(self.inner.transform_point(&rhs.point))
    }
}

impl<From, To> Mul<RigidBodyTransform<From, To>> for Vector<From> {
    type Output = Vector<To>;

    fn mul(self, rhs: RigidBodyTransform<From, To>) -> Self::Output {
        Vector::from_nalgebra_vector(rhs.inner.inverse_transform_vector(&self.inner))
    }
}

impl<From, To> Mul<Vector<To>> for RigidBodyTransform<From, To> {
    type Output = Vector<From>;

    fn mul(self, rhs: Vector<To>) -> Self::Output {
        Vector::from_nalgebra_vector(self.inner.transform_vector(&rhs.inner))
    }
}

// RigidBodyTransform<From, To> * Bearing<To> -> Bearing<From>
impl<From, To> Mul<Bearing<To>> for RigidBodyTransform<From, To>
where
    From: crate::systems::BearingDefined,
    To: crate::systems::BearingDefined,
{
    type Output = Bearing<From>;

    fn mul(self, rhs: Bearing<To>) -> Self::Output {
        (self * rhs.to_unit_vector())
            .bearing_at_origin()
            .expect("magnitude should still be 1 after rotation")
    }
}

// Bearing<From> * RigidBodyTransform<From, To> -> Bearing<To>
impl<From, To> Mul<RigidBodyTransform<From, To>> for Bearing<From>
where
    From: crate::systems::BearingDefined,
    To: crate::systems::BearingDefined,
{
    type Output = Bearing<To>;

    fn mul(self, rhs: RigidBodyTransform<From, To>) -> Self::Output {
        (self.to_unit_vector() * rhs)
            .bearing_at_origin()
            .expect("magnitude should still be 1 after rotation")
    }
}

// RigidBodyTransform<From, Over> * RigidBodyTransform<Over, To> -> RigidBodyTransform<From, To>
//
// like for Rotation, a RigidBodyTransform is a matrix multiply. the same ordering of the operands
// should hold for RigidBodyTransform as did for Rotation to end up with the columns and rows
// adding up, so we maintain the order used there.
impl<From, Over, To> Mul<RigidBodyTransform<Over, To>> for RigidBodyTransform<From, Over> {
    type Output = RigidBodyTransform<From, To>;

    fn mul(self, rhs: RigidBodyTransform<Over, To>) -> Self::Output {
        Self::Output {
            inner: self.inner * rhs.inner,
            from: PhantomData::<From>,
            to: PhantomData::<To>,
        }
    }
}

// RigidBodyTransform<From, Over> * Rotation<Over, To> -> RigidBodyTransform<From, To>
//
// here, too, the same ordering of the operands to the matrix multiply hold
impl<From, Over, To> Mul<Rotation<Over, To>> for RigidBodyTransform<From, Over> {
    type Output = RigidBodyTransform<From, To>;

    fn mul(self, rhs: Rotation<Over, To>) -> Self::Output {
        Self::Output {
            inner: self.inner * rhs.inner,
            from: PhantomData::<From>,
            to: PhantomData::<To>,
        }
    }
}

#[cfg(any(test, feature = "approx"))]
impl<From, To> AbsDiffEq<Self> for RigidBodyTransform<From, To> {
    type Epsilon = <f64 as AbsDiffEq>::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        Isometry3::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.inner.abs_diff_eq(&other.inner, epsilon)
    }
}

#[cfg(any(test, feature = "approx"))]
impl<From, To> RelativeEq for RigidBodyTransform<From, To> {
    fn default_max_relative() -> Self::Epsilon {
        Isometry3::default_max_relative()
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

#[cfg(test)]
mod tests {
    use crate::builder::bearing::Components;
    use crate::coordinate_systems::{Ecef, Frd, Ned};
    use crate::coordinates::Coordinate;
    use crate::geodedic::Wgs84;
    use crate::math::{RigidBodyTransform, Rotation};
    use crate::util::BoundedAngle;
    use crate::vectors::Vector;
    use crate::{coordinate, Point3};
    use crate::{system, Bearing};
    use crate::{vector, Vector3};
    use approx::assert_abs_diff_eq;
    use approx::assert_relative_eq;
    use rstest::rstest;
    use uom::si::f64::{Angle, Length};
    use uom::si::{angle::degree, length::meter};

    fn m(meters: f64) -> Length {
        Length::new::<meter>(meters)
    }
    fn d(degrees: f64) -> Angle {
        Angle::new::<degree>(degrees)
    }

    system!(struct PlaneFrd using FRD);
    system!(struct PlaneNed using NED);
    system!(struct PlaneBNed using NED);
    system!(struct SensorFrd using FRD);
    system!(struct EmitterFrd using FRD);

    #[rstest]
    // Given as coordinates System A -> Orientation of B in A -> System B
    // Yaw only
    #[case(
        Point3::new(1., 0., 0.),
        (d(0.), d(0.), d(0.)),
        Point3::new(1., 0., 0.)
    )]
    #[case(
        Point3::new(1., 0., 0.),
        (d(90.), d(0.), d(0.)),
        Point3::new(0., -1., 0.)
    )]
    #[case(
        Point3::new(1., 0., 0.),
        (d(180.), d(0.), d(0.)),
        Point3::new(-1., 0., 0.)
    )]
    #[case(
        Point3::new(1., 0., 0.),
        (d(270.), d(0.), d(0.)),
        Point3::new(0., 1., 0.)
    )]
    #[case(
        Point3::new(1., 0., 0.),
        (d(360.), d(0.), d(0.)),
        Point3::new(1., 0., 0.)
    )]
    // Pitch only
    #[case(
        Point3::new(1., 0., 0.),
        (d(0.), d(90.), d(0.)),
        Point3::new(0., 0., 1.)
    )]
    #[case(
        Point3::new(1., 0., 0.),
        (d(0.), d(180.), d(0.)),
        Point3::new(-1., 0., 0.)
    )]
    #[case(
        Point3::new(1., 0., 0.),
        (d(0.), d(270.), d(0.)),
        Point3::new(0., 0., -1.)
    )]
    #[case(
        Point3::new(1., 0., 0.),
        (d(0.), d(360.), d(0.)),
        Point3::new(1., 0., 0.)
    )]
    // Roll only
    #[case(  // No effect on x-axis
        Point3::new(1., 0., 0.),
        (d(0.), d(0.), d(69.)),
        Point3::new(1., 0., 0.)
    )]
    #[case(
        Point3::new(0., 1., 0.),
        (d(0.), d(0.), d(90.)),
        Point3::new(0., 0., -1.)
    )]
    #[case(
        Point3::new(0., 1., 0.),
        (d(0.), d(0.), d(180.)),
        Point3::new(0., -1., 0.)
    )]
    #[case(
        Point3::new(0., 1., 0.),
        (d(0.), d(0.), d(270.)),
        Point3::new(0., 0., 1.)
    )]
    #[case(
        Point3::new(0., 1., 0.),
        (d(0.), d(0.), d(360.)),
        Point3::new(0., 1., 0.)
    )]
    // Rotate pitch and yaw
    // Front axis now points left
    #[case(
        Point3::new(1., 0., 0.),
        (d(90.), d(180.), d(0.)),
        Point3::new(0., -1., 0.)
    )]
    // Right axis now points backwards
    #[case(
        Point3::new(0., 1., 0.),
        (d(90.), d(180.), d(0.)),
        Point3::new(-1., 0., 0.)
    )]
    // Downward axis now points up
    #[case(
        Point3::new(0., 0., 1.),
        (d(90.), d(180.), d(0.)),
        Point3::new(0., 0., -1.)
    )]
    // front-axis is now to the right and 45 degrees up
    #[case(
        Point3::new(0., 10., 0.),
        (d(90.), d(45.), d(0.)),
        Point3::new(7.0710678118654755, 0., 7.071067811865475)
    )]
    // front-axis is now to the right and 45 degrees up
    #[case(
        Point3::new(10., 0., 0.),
        (d(90.), d(45.), d(0.)),
        Point3::new(0., -10., 0.)
    )]
    // Rotate all three axis
    #[case(
        Point3::new(- 0.00028087357950656866, 10.000385238610235, - 2.2283317053339857e-5),
        (d(39.), d(45.), d(55.)),
        Point3::new(4.450, 8.103, - 3.814)
    )]
    /// Test that we can transform a coordinate in FRD system A to the equivalent coordinate in FRD system B, given the orientation of B in A.
    fn ned_and_frd_coordinate_transforms_work(
        #[case] point_in_a: Point3,
        #[case] ypr: (Angle, Angle, Angle),
        #[case] point_in_b: Point3,
    ) {
        let (yaw, pitch, roll) = ypr;
        let frd_orientation =
            unsafe { Rotation::<SensorFrd, EmitterFrd>::from_tait_bryan_angles(yaw, pitch, roll) };

        // Sanity-check that to_tait_bryan_angles does what we expect, but only if we're already
        // giving normalized input (otherwise the unit quaternion will do it for us and the
        // comparison will fail).
        if pitch <= d(90.) {
            let (y, p, r) = frd_orientation.to_tait_bryan_angles();
            for (a, b) in [(y, yaw), (p, pitch), (r, roll)] {
                assert_abs_diff_eq!(&BoundedAngle::new(a), &BoundedAngle::new(b));
            }
        }

        // Orientation transform works between FRD coordinates
        let frd_a = Coordinate::<SensorFrd>::from_nalgebra_point(point_in_a);
        let frd_b = frd_orientation.transform(frd_a);
        let frd_a_again = frd_orientation.inverse_transform(frd_b);
        assert_relative_eq!(
            frd_b,
            Coordinate::<EmitterFrd>::from_nalgebra_point(point_in_b),
        );
        assert_relative_eq!(frd_a_again, frd_a);

        let (yaw_converted, pitch_converted, roll_converted) = frd_orientation.euler_angles();
        let frd_orientation_converted = unsafe {
            Rotation::<SensorFrd, EmitterFrd>::from_tait_bryan_angles(
                yaw_converted,
                pitch_converted,
                roll_converted,
            )
        };
        assert_relative_eq!(frd_orientation, frd_orientation_converted);

        // Orientation works between NED and FRD coordinates
        let ned_orientation =
            unsafe { Rotation::<Ned, Frd>::from_tait_bryan_angles(yaw, pitch, roll) };

        let ned = Coordinate::<Ned>::from_nalgebra_point(point_in_a);
        let frd = ned_orientation.transform(ned);
        let ned_again = ned_orientation.inverse_transform(frd);
        assert_relative_eq!(frd, Coordinate::<Frd>::from_nalgebra_point(point_in_b));
        assert_relative_eq!(ned_again, ned);

        // Use the point in b as translation. Could be arbitrary point instead.
        let translation = Vector3::new(1., 0., 0.);
        // NED pose does the same thing but adds a translation
        let ned_pose = unsafe {
            RigidBodyTransform::<Ned, Frd>::new(
                Vector::from_nalgebra_vector(translation),
                ned_orientation,
            )
        };

        let frd_after_pose = ned_pose.transform(ned);
        assert_relative_eq!(
            frd_after_pose,
            frd - ned_orientation.transform(Vector::<Ned>::from_nalgebra_vector(translation)),
        );
        // Check from FRD to NED
        let ned_again_after_pose = ned_pose.inverse_transform(frd_after_pose);
        assert_relative_eq!(ned, ned_again_after_pose)
    }

    #[test]
    fn orientation_multiplication_works() {
        // Transforms a NED to an FRD coordinate of an object with this yaw, pitch, roll.
        let ned_to_frd = unsafe {
            Rotation::<PlaneNed, PlaneFrd>::from_tait_bryan_angles(d(90.), d(90.), d(0.))
        };

        // Transforms a ECEF coordinate to a NED coordinate for a NED at a particular latitude and longitude.
        let ecef_to_ned = unsafe { Rotation::<Ecef, PlaneNed>::ecef_to_ned_at(d(52.), d(-3.)) };

        // Chains both transformations to transform a ECEF coordinate to a Frd Coordinate via NED.
        let ecef_to_frd = ecef_to_ned * ned_to_frd;

        let forward = coordinate!(f = m(1.), r = m(0.), d = m(0.); in PlaneFrd);
        let right = coordinate!(f = m(0.), r = m(1.), d = m(0.); in PlaneFrd);
        let down = coordinate!(f = m(0.), r = m(0.), d = m(1.); in PlaneFrd);

        // check that the FRD axis are at the right spots in NED.
        let forward_in_ned = ned_to_frd.inverse_transform(forward);
        let right_in_ned = ned_to_frd.inverse_transform(right);
        let down_in_ned = ned_to_frd.inverse_transform(down);

        assert_relative_eq!(
            forward_in_ned,
            -coordinate!(n = m(0.), e = m(0.), d = m(1.))
        );
        assert_relative_eq!(right_in_ned, -coordinate!(n = m(1.), e = m(0.), d = m(0.)));
        assert_relative_eq!(down_in_ned, coordinate!(n = m(0.), e = m(1.), d = m(0.)));

        // Turn the NED to ECEF
        let forward_in_ecef = ecef_to_ned.inverse_transform(forward_in_ned);
        let right_in_ecef = ecef_to_ned.inverse_transform(right_in_ned);
        let down_in_ecef = ecef_to_ned.inverse_transform(down_in_ned);

        // check that the inverse works
        let frd_to_ecef = ecef_to_frd.inverse();

        // Check that the chain did the same.
        assert_relative_eq!(forward_in_ecef, ecef_to_frd.inverse_transform(forward));
        assert_relative_eq!(
            frd_to_ecef.transform(forward),
            ecef_to_frd.inverse_transform(forward)
        );

        assert_relative_eq!(right_in_ecef, ecef_to_frd.inverse_transform(right));
        assert_relative_eq!(
            frd_to_ecef.transform(right),
            ecef_to_frd.inverse_transform(right)
        );

        assert_relative_eq!(down_in_ecef, ecef_to_frd.inverse_transform(down));
        assert_relative_eq!(
            frd_to_ecef.transform(down),
            ecef_to_frd.inverse_transform(down)
        );

        // Now test if we can go to another FRD

        // Pose of a second frd in another ned
        let ned_to_frd_2 = unsafe {
            Rotation::<PlaneBNed, SensorFrd>::from_tait_bryan_angles(d(-45.), d(0.), d(0.))
        };
        // NED of the other FRDs object
        let ecef_to_ned_2 =
            unsafe { Rotation::<Ecef, PlaneBNed>::ecef_to_ned_at(d(54.), d(-3.67)) };

        let ecef_to_frd_2 = ecef_to_ned_2 * ned_to_frd_2;

        // What is the orientation of 2 in 1?
        let frd_1_to_frd_2 = ecef_to_frd.inverse() * ecef_to_frd_2;

        // Manually do the transforms
        let forward_in_ned_2 = ecef_to_ned_2.transform(forward_in_ecef);
        let right_in_ned_2 = ecef_to_ned_2.transform(right_in_ecef);
        let down_in_ned_2 = ecef_to_ned_2.transform(down_in_ecef);

        let forward_in_frd_2 = ned_to_frd_2.transform(forward_in_ned_2);
        let right_in_frd_2 = ned_to_frd_2.transform(right_in_ned_2);
        let down_in_frd_2 = ned_to_frd_2.transform(down_in_ned_2);

        let forward_chained = frd_1_to_frd_2.transform(forward);
        let right_chained = frd_1_to_frd_2.transform(right);
        let down_chained = frd_1_to_frd_2.transform(down);

        // Now check if the chained transform did the same thing.
        assert_relative_eq!(forward_in_frd_2, forward_chained);
        assert_relative_eq!(right_in_frd_2, right_chained);
        assert_relative_eq!(down_in_frd_2, down_chained);
    }

    impl From<&nav_types::ECEF<f64>> for Coordinate<Ecef> {
        fn from(value: &nav_types::ECEF<f64>) -> Self {
            coordinate!(x = m(value.x()), y = m(value.y()), z = m(value.z()))
        }
    }

    #[rstest]
    #[case(d(47.9948211), d(7.8211606), m(1000.))]
    #[case(d(67.112282), d(19.880389), m(0.))]
    #[case(d(84.883074), d(-29.160550), m(2000.))]
    #[case(d(-27.270950), d(143.722880), m(100.))]
    fn orientation_ecef_to_ned_construction(
        #[case] lat: Angle,
        #[case] long: Angle,
        #[case] alt: Length,
    ) {
        // This test uses nav_types to check if the computations match.

        let ecef_to_ned = unsafe { Rotation::<Ecef, PlaneNed>::ecef_to_ned_at(lat, long) };

        let location = nav_types::WGS84::from_degrees_and_meters(
            lat.get::<degree>(),
            long.get::<degree>(),
            alt.get::<meter>(),
        );
        let location_ecef = nav_types::ECEF::from(location);

        let north = nav_types::NED::new(1., 0., 0.);
        let east = nav_types::NED::new(0., 1., 0.);
        let down = nav_types::NED::new(0., 0., 1.);
        let origin = nav_types::NED::new(0., 0., 0.);

        // Get the axis points of the NED local in ECEF using nav_types.
        let ned_origin_in_ecef = location_ecef + origin;
        let ned_origin_in_ecef = Coordinate::<Ecef>::from(&ned_origin_in_ecef);

        let ned_north_in_ecef = location_ecef + north;
        let ned_north_in_ecef = Coordinate::<Ecef>::from(&ned_north_in_ecef);

        let ned_east_in_ecef = location_ecef + east;
        let ned_east_in_ecef = Coordinate::<Ecef>::from(&ned_east_in_ecef);

        let ned_down_in_ecef = location_ecef + down;
        let ned_down_in_ecef = Coordinate::<Ecef>::from(&ned_down_in_ecef);

        // The orientation part does not perform the translation from earth center to NED origin.
        // Nav_types does. This is why we subtract the NED origin for testing the orientation.

        let result_origin = ecef_to_ned.inverse_transform(Coordinate::<PlaneNed>::origin());
        // The origin is not changed because this is only the orientation, not pose.
        //  Ie. there is no translation between ECEF and PlaneNed. The plane is at the center of the earth on this test.
        assert_relative_eq!(Coordinate::<Ecef>::origin(), result_origin);
        assert_relative_eq!(
            ecef_to_ned.transform(Coordinate::<Ecef>::origin()),
            Coordinate::<PlaneNed>::origin()
        );

        let point_on_north =
            Coordinate::<PlaneNed>::origin() + Vector::<PlaneNed>::ned_north_axis();
        let result_north = ecef_to_ned.inverse_transform(point_on_north);
        // nav_types does the NED at the correct position and not the center of the earth. We remove the translation here.
        let expected_north = ned_north_in_ecef - ned_origin_in_ecef;
        assert_relative_eq!(result_north, Coordinate::<Ecef>::origin() + expected_north);

        let point_on_east = Coordinate::<PlaneNed>::origin() + Vector::<PlaneNed>::ned_east_axis();
        let result_east = ecef_to_ned.inverse_transform(point_on_east);
        // nav_types does the NED at the correct position and not the center of the earth. We remove the translation here.
        let expected_east = ned_east_in_ecef - ned_origin_in_ecef;
        assert_relative_eq!(result_east, Coordinate::<Ecef>::origin() + expected_east);

        let point_on_down = Coordinate::<PlaneNed>::origin() + Vector::<PlaneNed>::ned_down_axis();
        let result_down = ecef_to_ned.inverse_transform(point_on_down);
        // nav_types does the NED at the correct position and not the center of the earth. We remove the translation here.
        let expected_down = ned_down_in_ecef - ned_origin_in_ecef;
        assert_relative_eq!(result_down, Coordinate::<Ecef>::origin() + expected_down);

        // Construct this as pose instead of just orientation.
        // This time the translation should be in there as well, so we can compare directly to nav_types output.
        let pose = unsafe {
            RigidBodyTransform::<Ecef, PlaneNed>::ecef_to_ned_at(
                &Wgs84::builder()
                    .latitude(lat)
                    .expect("latitude is in-range")
                    .longitude(long)
                    .altitude(alt)
                    .build(),
            )
        };

        let result_ned_north_in_ecef = pose.inverse_transform(point_on_north);
        assert_relative_eq!(result_ned_north_in_ecef, ned_north_in_ecef);

        let result_ned_east_in_ecef = pose.inverse_transform(point_on_east);
        assert_relative_eq!(result_ned_east_in_ecef, ned_east_in_ecef);

        let result_ned_down_in_ecef = pose.inverse_transform(point_on_down);
        assert_relative_eq!(result_ned_down_in_ecef, ned_down_in_ecef);
    }

    #[test]
    fn pose_serde() {
        let pose = unsafe {
            RigidBodyTransform::new(
                vector!(n = m(50.), e = m(45.), d = m(10.)),
                Rotation::<PlaneNed, PlaneFrd>::from_tait_bryan_angles(d(15.), d(0.), d(1.)),
            )
        };

        let ser = serde_yaml::to_string(&pose).unwrap();

        let de = serde_yaml::from_str::<RigidBodyTransform<PlaneNed, PlaneFrd>>(&ser).unwrap();
        assert_eq!(pose, de);
    }

    #[test]
    fn bearing_rotation() {
        // assume forward is currently pointing east
        let ned_to_frd =
            unsafe { Rotation::<Ned, Frd>::from_tait_bryan_angles(d(90.), d(0.), d(0.)) };

        // a bearing pointing East should have an azimuth of 0° to forward
        // and since the plane has no pitch relative to horizon, elevation shouldn't change
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(90.),
                elevation: d(45.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(0.),
                elevation: d(45.)
            })
            .unwrap()
        );

        // a bearing pointing South should have an azimuth of 90° to forward
        // elevation still shouldn't change
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(180.),
                elevation: d(45.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(90.),
                elevation: d(45.)
            })
            .unwrap()
        );

        // conversely, a bearing pointing Forward should have an azimuth of 90° to North
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(0.),
                    elevation: d(45.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(90.),
                elevation: d(45.)
            })
            .unwrap()
        );

        // and one pointing backwards should have an azimuth of -90° to North
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(180.),
                    elevation: d(45.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(-90.),
                elevation: d(45.)
            })
            .unwrap()
        );

        // if the plane is pitched, elevation _does_ change
        // (we'll keep yaw 0° for now)
        let ned_to_frd =
            unsafe { Rotation::<Ned, Frd>::from_tait_bryan_angles(d(0.), d(45.), d(0.)) };

        // along the horizon should be seen as -45° compared to FR-plane
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(0.),
                elevation: d(0.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(0.),
                elevation: d(-45.)
            })
            .unwrap()
        );

        // 45° to the horizon should be parallel to FR-plane
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(0.),
                elevation: d(45.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(0.),
                elevation: d(0.)
            })
            .unwrap()
        );

        // and vice-versa
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(0.),
                    elevation: d(-45.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(0.),
                elevation: d(0.)
            })
            .unwrap()
        );
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(0.),
                    elevation: d(0.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(0.),
                elevation: d(45.)
            })
            .unwrap()
        );

        // since rotations are intrinsic, things get weird with yaw/azimuth _plus_ elevation
        // in particular, if the plane is pitched up at a yaw, or is pitched but has an observation
        // at a given azimuth, the resulting FRD observations aren't trivial to compute as bearing
        // relative to NED. one that's "easy" for humans to think about is if the yaw is 180, since
        // it just "flips" the azimuth and predictably changes the elevation, so that's what we'll
        // use in these tests. we'll continue with no roll.
        let ned_to_frd = unsafe {
            Rotation::<Ned, Frd>::from_tait_bryan_angles(
                d(180.), // pointed South
                d(45.),  // FRD "up" 0 is NED "up" 45
                d(0.),
            )
        };

        // sanity-check: directly in front should match orientation of plane
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(0.),
                    elevation: d(0.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(180.),
                elevation: d(45.)
            })
            .unwrap()
        );
        // directly behind should be North and "flip" pitch
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(180.),
                    elevation: d(0.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(0.),
                elevation: d(-45.)
            })
            .unwrap()
        );
        // and vice-versa
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(180.),
                elevation: d(45.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(0.),
                elevation: d(0.)
            })
            .unwrap()
        );
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(0.),
                elevation: d(-45.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(180.),
                elevation: d(0.)
            })
            .unwrap()
        );

        // an observation to the left with zero elevation should be East
        // and should _also_ have zero elevation
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(-90.),
                    elevation: d(0.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(90.),
                elevation: d(0.)
            })
            .unwrap()
        );
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(90.),
                elevation: d(0.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(-90.),
                elevation: d(0.)
            })
            .unwrap()
        );

        // an observation directly below us should have our pitch but flipped, and should have an
        // azimuth of _South_ since it's _not_ aligned with NED's Z axis (which would make it get
        // set to 0).
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(0.),
                    elevation: d(-90.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(180.),
                elevation: d(-45.)
            })
            .unwrap()
        );
        // and vice-versa
        // although here we have to be forgiving with the azimuth because we end up with a bearing
        // that points straight down (-90°) in FRD, in which case the azimuth is unpredictable.
        assert_relative_eq!(
            BoundedAngle::new(
                (Bearing::<Ned>::build(Components {
                    azimuth: d(180.),
                    elevation: d(-45.)
                })
                .unwrap()
                    * ned_to_frd)
                    .elevation()
            ),
            BoundedAngle::new(d(-90.))
        );

        // now the same with roll
        // this too makes computing what's expected tricky for the human brain. so, we pick a roll
        // of 180° which essentially just flips elevation _and_ azimuth in this configuration.
        let ned_to_frd = unsafe {
            Rotation::<Ned, Frd>::from_tait_bryan_angles(
                d(180.), // pointed South
                d(45.),  // FRD "up" 0 is NED "up" 45
                d(180.), // upside-down
            )
        };

        // directly in front should still match orientation of plane
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(0.),
                    elevation: d(0.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(180.),
                elevation: d(45.)
            })
            .unwrap()
        );
        // directly behind should still be North and "flip" pitch
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(180.),
                    elevation: d(0.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(0.),
                elevation: d(-45.)
            })
            .unwrap()
        );
        // and vice-versa
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(180.),
                elevation: d(45.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(0.),
                elevation: d(0.)
            })
            .unwrap()
        );
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(0.),
                elevation: d(-45.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(180.),
                elevation: d(0.)
            })
            .unwrap()
        );

        // an observation to the left with zero elevation should now be West
        // and should _also_ have zero elevation
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(-90.),
                    elevation: d(0.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(-90.),
                elevation: d(0.)
            })
            .unwrap()
        );
        assert_relative_eq!(
            Bearing::<Ned>::build(Components {
                azimuth: d(-90.),
                elevation: d(0.)
            })
            .unwrap()
                * ned_to_frd,
            Bearing::<Frd>::build(Components {
                azimuth: d(-90.),
                elevation: d(0.)
            })
            .unwrap()
        );

        // an observation directly below us now _actually_ points "up" by the same amount as our
        // pitch. it also has an azimuth that is flipped (ie, North not South) since the inverse of
        // the pitch crosses the XZ plane.
        assert_relative_eq!(
            ned_to_frd
                * Bearing::<Frd>::build(Components {
                    azimuth: d(0.),
                    elevation: d(-90.)
                })
                .unwrap(),
            Bearing::<Ned>::build(Components {
                azimuth: d(0.),
                elevation: d(45.)
            })
            .unwrap()
        );
        // and vice-versa, though again we have to be forgiving about azimuth
        assert_relative_eq!(
            BoundedAngle::new(
                (Bearing::<Ned>::build(Components {
                    azimuth: d(0.),
                    elevation: d(45.)
                })
                .unwrap()
                    * ned_to_frd)
                    .elevation()
            ),
            BoundedAngle::new(d(-90.))
        );
    }
}
