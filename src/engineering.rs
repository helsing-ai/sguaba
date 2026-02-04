//! Spatial operations expressed in engineering language.
//!
//! This module provides wrappers around the types in [`mod math`](crate::math) that are easier to
//! grok for those with more of an engineering background. The types here have slightly less
//! expressive power than the "raw" mathematical constructs there, but are more straightforward to
//! construct and use for common engineering applications.
//!
//! The main type provided by this module is [`Pose`], which describes an object's position and
//! orientation in a coordinate system (see [`CoordinateSystem`]). Poses can be converted between
//! coordinate systems using [`RigidBodyTransform`] (eg, [`RigidBodyTransform::transform`]).
//!
//! This module also provides the [`Orientation`] type to represents an object's orientation in a
//! coordinate system independent of its position. To represent just an object's position, you can
//! use [`Coordinate`], which is shared between the math and engineering modules.
//!
//! Orientation does not have a globally-unique definition (eg, what's the orientation of a
//! point or of a cube?). However, for the purposes of this module, every object is assumed to have
//! "body axes" that dictate what, eg, "forward" means for that object. By convention, the positive
//! X body axis is always "forward"/the direction of movement. The other two axes vary, but in
//! general the positive Y body axis is either left (commonly in ENU) or right (commonly in
//! [NED](NedLike)), and the positive Z body axis is either up (commonly in association with ENU)
//! or down (commonly in association with [NED](NedLike)). You'll sometimes see these body
//! coordinate systems referred to as Forward-Left-Up (FLU) and Forward-Right-Down
//! ([FRD](FrdLike)). An object's orientation is then the rotation that must be applied to the
//! reference system's axes such that it aligns with the body's axes.
//!
//! One method worth calling out in particular is [`Pose::map_as_zero_in`], which allows you to
//! start a new coordinate system with origin and rotation equal to that of the current pose. This
//! is, for example, how you'd construct a `RigidBodyTransform<_, PlaneFrd>`. To construct a
//! transform to or from a WGS84 location, you need to go via [`Ecef`] and
//! [`RigidBodyTransform::ecef_to_ned_at`].

use crate::coordinates::Coordinate;
use crate::math::{RigidBodyTransform, Rotation};
use crate::systems::EquivalentTo;
use crate::{Point3, Vector};
use core::marker::PhantomData;
use core::ops::Mul;
use uom::si::f64::{Angle, Length};
use uom::ConstZero;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(doc)]
use crate::{
    systems::{Ecef, FrdLike, NedLike},
    Bearing, CoordinateSystem,
};

#[derive(Clone, Copy, Debug)]
pub(crate) struct ObjectCoordinateSystem;

/// Defines the orientation of an object in [`CoordinateSystem`] `In`.
///
/// This includes yaw, pitch, _and_ roll, so not just facing direction (which would miss roll).
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
pub struct Orientation<In> {
    pub(crate) inner: Rotation<In, ObjectCoordinateSystem>,
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<In> Clone for Orientation<In> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<In> Copy for Orientation<In> {}

impl<In> PartialEq<Self> for Orientation<In> {
    fn eq(&self, other: &Self) -> bool {
        self.inner.eq(&other.inner)
    }
}

impl<In> Orientation<In> {
    /// Constructs an orientation from ([intrinsic]) yaw, pitch, and roll [Tait-Bryan angles][tb].
    ///
    /// The meanings of yaw, pitch, and roll tend to align with common use in aerospace
    /// applications (eg, [`FrdLike`] or [`NedLike`]), but are more formally defined as described
    /// below.
    ///
    /// Yaw is rotation about the Z axis of `In`, with an object oriented at 0° yaw facing along
    /// the positive X axis. [`Bearing::azimuth`] tends to have the same semantics as yaw (but not
    /// always; eg, this is [_not_ the case for ENU][enu]).
    ///
    /// Since we are using [intrinsic] rotations (by convention), we now consider further rotations
    /// (ie, pitch and roll) to be with respect to the axes _after_ applying the yaw. So, as an
    /// example, with a yaw of 90° in NED (ie, facing East), a pitch of 45° will tilt the nose of
    /// the plane up by 45° while still pointing the plane East.
    ///
    /// Pitch is rotation about the Y axis, with an object at 0° pitch facing along the positive X
    /// axis (again, this is the X axis with yaw applied). [`Bearing::elevation`] tends to have the
    /// same semantics as pitch (though again, [watch out for ENU][enu]!).
    ///
    /// Roll is rotation about the X axis, with an object with 0° roll having its positive body Z
    /// axis pointing along positive Z. For instance, in NED, after applying a yaw of 90° and a
    /// pitch of 45° (so the plane points directly East with its nose pitched up by 45°), applying
    /// a roll of 20° would result in the plane's nose still pointing in the same direction, but
    /// rotated clockwise about its own axis by 20°. There is no equivalent to roll in [`Bearing`].
    ///
    /// To determine the direction of rotation (ie, in which direction a positive angle goes), you
    /// can use the [right-hand rule for rotations][rhrot]: curl your fingers and stick your thumb
    /// out in the positive direction of the axis you want to check rotation around (eg, positive Z
    /// for yaw). The direction your fingers curl is the direction of (positive) rotation. Note
    /// that a counterclockwise rotation about a Z if positive Z is down is a clockwise rotation
    /// when viewed from negative Z (ie, "up"); this matches the common use of yaw in [`FrdLike`]
    /// where a positive yaw means "to the right", or in [`NedLike`] where a positive yaw means
    /// "eastward".
    ///
    /// Be aware that rotational angles have high ambiguities in literature and are easy to use
    /// wrong, especially because different fields tend to use the same term with different
    /// meanings (eg, "Euler angles" mean something else in aerospace than in mathematics).
    ///
    /// [intrinsic]: https://dominicplein.medium.com/extrinsic-intrinsic-rotation-do-i-multiply-from-right-or-left-357c38c1abfd
    /// [tb]: https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
    /// [aero]: https://en.wikipedia.org/wiki/Aircraft_principal_axes
    /// [enu]: https://en.wikipedia.org/wiki/Axes_conventions#World_reference_frames_for_attitude_description
    /// [rhrot]: https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
    #[doc(alias = "from_nautical_angles")]
    #[doc(alias = "from_cardan_angles")]
    #[doc(alias = "from_ypr")]
    #[deprecated = "Prefer `tait_bryan_builder` to avoid argument-order confusion"]
    pub fn from_tait_bryan_angles(
        yaw: impl Into<Angle>,
        pitch: impl Into<Angle>,
        roll: impl Into<Angle>,
    ) -> Self {
        Self {
            // SAFETY: the object coordinate system is implictly defined, and so if we're told this
            // is the orientation of the object/body axes, then so be it.
            #[allow(deprecated)]
            inner: unsafe { Rotation::from_tait_bryan_angles(yaw, pitch, roll) },
        }
    }

    /// Constructs an orientation that is aligned with the axes of the [`CoordinateSystem`] `In`.
    ///
    /// That is equivalent to calling [`Orientation::from_tait_bryan_angles`] with all
    /// zero values.
    #[must_use]
    pub fn aligned() -> Self {
        Self::tait_bryan_builder()
            .yaw(Angle::ZERO)
            .pitch(Angle::ZERO)
            .roll(Angle::ZERO)
            .build()
    }

    /// Provides a type-safe builder for constructing an orientation from Tait-Bryan angles.
    ///
    /// This builder enforces the correct intrinsic order (yaw → pitch → roll) at compile time
    /// and provides named parameters to prevent argument order confusion.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use sguaba::{system, engineering::Orientation};
    /// use uom::si::{f64::Angle, angle::degree};
    ///
    /// system!(struct PlaneNed using NED);
    ///
    /// let orientation = Orientation::<PlaneNed>::tait_bryan_builder()
    ///     .yaw(Angle::new::<degree>(90.0))
    ///     .pitch(Angle::new::<degree>(45.0))
    ///     .roll(Angle::new::<degree>(5.0))
    ///     .build();
    /// ```
    ///
    /// The following examples should fail to compile because the angles are not provided
    /// in the correct order:
    ///
    /// ```compile_fail
    /// # use sguaba::{system, engineering::Orientation};
    /// # use uom::si::{f64::Angle, angle::degree};
    /// # system!(struct PlaneNed using NED);
    /// // Cannot call roll before pitch - roll() method doesn't exist on NeedsPitch state
    /// let orientation = Orientation::<PlaneNed>::tait_bryan_builder()
    ///     .yaw(Angle::new::<degree>(90.0))
    ///     .roll(Angle::new::<degree>(5.0))
    ///     .pitch(Angle::new::<degree>(45.0))
    ///     .build();
    /// ```
    ///
    /// ```compile_fail
    /// # use sguaba::{system, engineering::Orientation};
    /// # use uom::si::{f64::Angle, angle::degree};
    /// # system!(struct PlaneNed using NED);
    /// // Cannot skip yaw and start with pitch - pitch() method doesn't exist on NeedsYaw state
    /// let orientation = Orientation::<PlaneNed>::tait_bryan_builder()
    ///     .pitch(Angle::new::<degree>(45.0))
    ///     .yaw(Angle::new::<degree>(90.0))
    ///     .roll(Angle::new::<degree>(5.0))
    ///     .build();
    /// ```
    pub fn tait_bryan_builder() -> crate::math::tait_bryan_builder::TaitBryanBuilder<
        crate::math::tait_bryan_builder::NeedsYaw,
        Orientation<In>,
    > {
        crate::math::tait_bryan_builder::TaitBryanBuilder::new()
    }
}

impl<In> Default for Orientation<In> {
    fn default() -> Self {
        Self::aligned()
    }
}

impl<In> Orientation<In> {
    /// Constructs a rotation into [`CoordinateSystem`] `To` such that `self` has an orientation of
    /// zero in `To` (ie, [`Orientation::aligned`]).
    ///
    /// Less informally, if this rotation is applied to this `Orientation<In>`, it will yield
    /// [`Orientation::aligned`] as `Orientation<To>`.
    ///
    /// Conversely, if the inverse of this rotation is applied to [`Orientation::aligned`] for
    /// `Orientation<To>`, it will yield this `Orientation<In>`.
    ///
    /// Or, alternatively phrased, this yields a transformation that
    ///
    /// 1. takes a coordinate, vector, or orientation observed by the object with this orientation
    ///    in the coordinate system `In`; and
    /// 2. returns that coordinate or vector as if it were observed in a coordinate system (`To`)
    ///    where this `Orientation<In>` is [`Orientation::aligned`].
    ///
    /// Or, if you prefer a more mathematical description: this defines the pose of the whole
    /// coordinate system `To` _in_ `In`.
    ///
    /// # Safety
    ///
    /// <div class="warning">
    ///
    /// This method allows you to end up with erroneous transforms if you're not careful. See
    /// [`Pose::map_as_zero_in`] for more details.
    ///
    /// Specifically in the case of `Orientation`, you are also asserting that _only_ rotation (ie,
    /// no translation) is needed to convert from `In` to `To`.
    ///
    /// </div>
    ///
    /// # Examples
    ///
    /// ```rust
    /// use approx::assert_relative_eq;
    /// use sguaba::{system, Bearing, Coordinate, engineering::Orientation};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct PlaneNed using NED);
    /// system!(struct PlaneFrd using FRD);
    ///
    /// // plane orientation in NED is yaw 90° (east), pitch 45° (climbing), roll 5° (tilted right)
    /// let orientation = Orientation::<PlaneNed>::from_tait_bryan_angles(
    ///     Angle::new::<degree>(90.),
    ///     Angle::new::<degree>(45.),
    ///     Angle::new::<degree>(5.),
    /// );
    ///
    /// // plane observes something below it to its right (45° azimuth, -20° elevation),
    /// // at a range of 100m.
    /// let observation = Coordinate::<PlaneFrd>::from_bearing(
    ///     Bearing::builder()
    ///       .azimuth(Angle::new::<degree>(45.))
    ///       .elevation(Angle::new::<degree>(-20.)).expect("elevation is in-range")
    ///       .build(),
    ///     Length::new::<meter>(100.)
    /// );
    ///
    /// // to get the absolute bearing of the observation (ie, with respect to North and horizon),
    /// // we can use map_as_zero_in to obtain a transformation from PlaneNed to PlaneFrd, and
    /// // then invert it to go from PlaneFrd to PlaneNed:
    /// let plane_frd_to_ned = unsafe { orientation.map_as_zero_in::<PlaneFrd>() }.inverse();
    ///
    /// // applying that transform gives us the translated coordinate
    /// let observation_in_ned = plane_frd_to_ned.transform(observation);
    /// ```
    #[doc(alias = "as_transform_to")]
    #[doc(alias = "defines_pose_of")]
    #[must_use]
    pub unsafe fn map_as_zero_in<To>(self) -> Rotation<In, To> {
        Rotation {
            inner: self.inner.inner,
            from: self.inner.from,
            to: PhantomData::<To>,
        }
    }

    /// Casts the coordinate system type parameter of the orientation to the equivalent coordinate
    /// system `NewIn`.
    ///
    /// See [`EquivalentTo`] for details on when this is useful (and safe).
    ///
    /// Note that this performs no transform on the orientation's components, as that should be
    /// unnecessary when `EquivalentTo` is implemented.
    ///
    /// ```
    /// use sguaba::{system, systems::EquivalentTo, Bearing, engineering::Orientation};
    /// use uom::si::{f64::Angle, angle::degree};
    ///
    /// system!(struct PlaneNedFromCrate1 using NED);
    /// system!(struct PlaneNedFromCrate2 using NED);
    ///
    /// // SAFETY: these are truly the same thing just defined in different places
    /// unsafe impl EquivalentTo<PlaneNedFromCrate1> for PlaneNedFromCrate2 {}
    /// unsafe impl EquivalentTo<PlaneNedFromCrate2> for PlaneNedFromCrate1 {}
    ///
    /// let orientation_in_1 = Orientation::<PlaneNedFromCrate1>::from_tait_bryan_angles(
    ///     Angle::new::<degree>(90.),
    ///     Angle::new::<degree>(45.),
    ///     Angle::new::<degree>(5.),
    /// );
    ///
    /// assert_eq!(
    ///     Orientation::<PlaneNedFromCrate2>::from_tait_bryan_angles(
    ///         Angle::new::<degree>(90.),
    ///         Angle::new::<degree>(45.),
    ///         Angle::new::<degree>(5.),
    ///     ),
    ///     orientation_in_1.cast::<PlaneNedFromCrate2>()
    /// );
    /// ```
    #[must_use]
    pub fn cast<NewIn>(self) -> Orientation<NewIn>
    where
        In: EquivalentTo<NewIn>,
    {
        Orientation {
            inner: self.inner.cast_type_of_from::<NewIn>(),
        }
    }

    /// Returns the yaw-pitch-roll [Tait-Bryan angles][tb] that describe this orientation.
    ///
    /// See [`Orientation::from_tait_bryan_angles`] for documentation about the exact meaning of
    /// yaw, pitch, and roll here.
    ///
    /// [tb]: https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
    #[must_use]
    pub fn to_tait_bryan_angles(&self) -> (Angle, Angle, Angle) {
        self.inner.to_tait_bryan_angles()
    }

    /// Linearly interpolate between this orientation and another one.
    ///
    /// Conceptually returns `self * (1.0 - t) + rhs * t`, i.e., the linear blend of the two
    /// rotations using the scalar value `t`.
    ///
    /// The value for `t` is not restricted to the range [0, 1].
    #[must_use]
    pub fn nlerp(&self, rhs: &Self, t: f64) -> Self {
        Self {
            inner: self.inner.nlerp(&rhs.inner, t),
        }
    }
}

impl<From, To> Mul<Rotation<From, To>> for Orientation<From> {
    type Output = Orientation<To>;

    fn mul(self, rhs: Rotation<From, To>) -> Self::Output {
        // trivially, this is:
        //
        if false {
            let _ = rhs.inverse() * self;
            // which also typechecks!
        }
        //
        // but we'd like to avoid the inverse intermediate
        //
        // if we inline .inverse(), this is really
        //
        //     Rotation::<To, From> {
        //         inner: rhs.inverse(),
        //         from: PhantomData,
        //         to: PhantomData,
        //     } * self
        //
        // which if we inline the Mul below (which ^ uses) is really
        //
        //     Pose {
        //         inner: Rotation {
        //             inner: rhs.inverse() * self.inner,
        //             from: PhantomData,
        //             to: PhantomData,
        //         },
        //     }
        //
        // which if we inline the Mul of Rotation and Rotation is really
        //
        //     Pose {
        //         inner: Rotation {
        //             inner: rhs.inverse().inner * self.inner.inner,
        //             from: PhantomData,
        //             to: PhantomData,
        //         },
        //     }
        //
        // so that's an inverse of an nalgebra::UnitQuaternion followed by a multiply of
        // two UnitQuaternions. unfortunately, unlike for isometries, there isn't an inv_mul on
        // UnitQuaternion (why not?), so for now we'll do it the naive way:
        rhs.inverse() * self
    }
}

impl<From, To> Mul<Orientation<To>> for Rotation<From, To> {
    type Output = Orientation<From>;

    fn mul(self, rhs: Orientation<To>) -> Self::Output {
        Orientation {
            inner: self * rhs.inner,
        }
    }
}

impl<From, To> Mul<RigidBodyTransform<From, To>> for Orientation<From> {
    type Output = Pose<To>;

    fn mul(self, rhs: RigidBodyTransform<From, To>) -> Self::Output {
        // trivially, this is:
        //
        if false {
            let _ = rhs.inverse() * self;
            // which also typechecks!
        }
        //
        // but we'd like to avoid the inverse intermediate
        //
        // if we inline .inverse(), this is really
        //
        //     RigidBodyTransform::<To, From> {
        //         inner: rhs.inverse(),
        //         from: PhantomData,
        //         to: PhantomData,
        //     } * self
        //
        // which if we inline the Mul below (which ^ uses) is really
        //
        //     Pose {
        //         inner: RigidBodyTransform {
        //             inner: rhs.inverse() * self.inner,
        //             from: PhantomData,
        //             to: PhantomData,
        //         },
        //     }
        //
        // which if we inline the Mul of RigidBodyTransform and Rotation is really
        //
        //     Pose {
        //         inner: RigidBodyTransform {
        //             inner: rhs.inverse().inner * self.inner.inner,
        //             from: PhantomData,
        //             to: PhantomData,
        //         },
        //     }
        //
        // so that's an inverse of an nalgebra::Isometry3 followed by a multiply of
        // an Isometry3 and a UnitQuaternion. which in nalgebra is defined as
        // retaining the translation and multiplying the rotation (with the isometry
        // on the right):
        //
        //     Pose {
        //         inner: RigidBodyTransform {
        //             inner: Isometry3::from_parts(
        //                 rhs.inverse().inner.translation,
        //                 rhs.inverse().inner.rotation * self.inner.inner,
        //             ),
        //             from: rhs.to,
        //             to: self.inner.to,
        //         },
        //     }
        //
        // unfortunately, this isn't trivial to compute directly without the
        // inverse() intermediate the way it is for two `Isometry3` (as is the
        // case when multiplying a `Pose`), so we're stuck with the trivial impl
        // with the intermediate until someone smarter comes along who can directly
        // compute the above.
        rhs.inverse() * self
    }
}

impl<From, To> Mul<Orientation<To>> for RigidBodyTransform<From, To> {
    type Output = Pose<From>;

    fn mul(self, rhs: Orientation<To>) -> Self::Output {
        Pose {
            inner: self * rhs.inner,
        }
    }
}

/// Defines the pose (ie, position and orientation) of an object in the [`CoordinateSystem`] `In`.
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
pub struct Pose<In> {
    pub(crate) inner: RigidBodyTransform<In, ObjectCoordinateSystem>,
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<In> Clone for Pose<In> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<In> Copy for Pose<In> {}

impl<In> PartialEq<Self> for Pose<In> {
    fn eq(&self, other: &Self) -> bool {
        self.inner.eq(&other.inner)
    }
}

impl<In> Pose<In> {
    /// Constructs a pose from a position and orientation.
    #[must_use]
    pub fn new(position: Coordinate<In>, orientation: Orientation<In>) -> Self {
        Self {
            // SAFETY: the object coordinate system is implictly defined, and so if we're told this
            // is the position and orientation of the object/body axes, then so be it.
            inner: unsafe {
                RigidBodyTransform::new(Vector::from(position), orientation.map_as_zero_in())
            },
        }
    }

    /// Constructs a transform into [`CoordinateSystem`] `To` such that `self` is "zero" in `To`.
    ///
    /// Less informally,
    ///
    /// - if this transform is applied to the [`Coordinate`] of this pose, it will yield
    ///   [`Coordinate::origin`], and
    /// - if this transform is applied to the [`Orientation`] of this pose, it will yield
    ///   [`Orientation::aligned`].
    ///
    /// Conversely,
    ///
    /// - if the inverse transform is applied to [`Coordinate::origin`] for `Coordinate<To>`,
    ///   it will yield this pose's [`Coordinate`].
    /// - if the inverse transform is applied to an object with [`Orientation::aligned`] for
    ///   `Orientation<To>`, it will yield this pose's [`Orientation`].
    ///
    /// Or, alternatively phrased, this yields a transformation that
    ///
    /// 1. takes a coordinate, vector, or orientation observed by the object with this pose
    ///    in the coordinate system `In`; and
    /// 2. returns that coordinate or vector as if it were observed in a coordinate system (`To`)
    ///    where this pose's orientation is [`Orientation::aligned`] and this pose's position is
    ///    [`Coordinate::origin`].
    ///
    /// Or, if you prefer a more mathematical description: this defines the pose of the whole
    /// coordinate system `To` _in_ `In`.
    ///
    /// This is most commonly used to derive a transformation into a "body" coordinate system
    /// (like [`FrdLike`]) by calling `pose.map_as_zero_in::<BodySystem>()` on the pose of that
    /// body in some other coordinate system (`In`). The body's position in `In` is
    /// straightforwardly the origin in the body coordinate system. Meanwhile, the intuition for
    /// orientation is that this is what it _means_ for a an object to be "facing" a particular
    /// direction in `In` (ie, to have an `Orientation<In>`); that the object's axes (ie, body
    /// coordinate system axes) "point" in that direction in `In`. Which in turn means that the
    /// object's orientation in `In` "maps as zero" in the body system.
    ///
    /// # Safety
    ///
    /// <div class="warning">
    ///
    /// This method is the primary way in which you can end up with erroneous transforms in this
    /// crate. There are no safeguards against claiming that, say, an object's NED pose is zero in
    /// ECEF. Ultimately, this is a necessary shortcoming -- there is no inherent connection
    /// between, say, FRD and NED coordinate systems, so there must be a declaration _somewhere_
    /// about how they map to each other. That's this method.
    ///
    /// It is marked as `unsafe` because creating a transform erroneously will allow converting
    /// between type-safe wrappers _without_ doing the correct corrections to their embedded data,
    /// thus defeating the type safety elsewhere.
    ///
    /// </div>
    ///
    /// # Examples
    ///
    /// ```rust
    /// use approx::assert_relative_eq;
    /// use sguaba::{system, Bearing, Coordinate, engineering::{Orientation, Pose}};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct PlaneNed using NED);
    /// system!(struct PlaneFrd using FRD);
    ///
    /// // plane orientation in NED is yaw 90° (east), pitch 45° (climbing), roll 5° (tilted right)
    /// let orientation = Orientation::<PlaneNed>::from_tait_bryan_angles(
    ///     Angle::new::<degree>(90.),
    ///     Angle::new::<degree>(45.),
    ///     Angle::new::<degree>(5.),
    /// );
    ///
    /// // plane position in _plane_ NED has the plane at the origin
    /// let pose = Pose::<PlaneNed>::new(
    ///     Coordinate::<PlaneNed>::origin(),
    ///     orientation,
    /// );
    ///
    /// // plane observes something below it to its right (45° azimuth, -20° elevation),
    /// // at a range of 100m.
    /// let observation = Coordinate::<PlaneFrd>::from_bearing(
    ///     Bearing::builder()
    ///       .azimuth(Angle::new::<degree>(45.))
    ///       .elevation(Angle::new::<degree>(-20.)).expect("elevation is in-range")
    ///       .build(),
    ///     Length::new::<meter>(100.)
    /// );
    ///
    /// // to get the location of the observation with respect to NED, we can use map_as_zero_in
    /// // to obtain a transformation from PlaneNed to PlaneFrd, and then invert it to go from
    /// // PlaneFrd to PlaneNed:
    /// let plane_frd_to_ned = unsafe { pose.map_as_zero_in::<PlaneFrd>() }.inverse();
    ///
    /// // applying that transform gives us the translated coordinate
    /// let observation_in_ned = plane_frd_to_ned.transform(observation);
    /// ```
    ///
    /// Note that in the example above, to convert `observation` into absolute Earth-bound
    /// coordinates (eg, ECEF or WGS84), we'd also need the plane's absolute location so that we
    /// can construct a a transform from ECEF to `PlaneNed`. For that, see
    /// [`RigidBodyTransform::ecef_to_ned_at`].
    #[doc(alias = "as_transform_to")]
    #[doc(alias = "defines_pose_of")]
    #[doc(alias = "is_position_and_facing_direction_of")]
    #[must_use]
    pub unsafe fn map_as_zero_in<To>(self) -> RigidBodyTransform<In, To> {
        RigidBodyTransform {
            inner: self.inner.inner,
            from: self.inner.from,
            to: PhantomData::<To>,
        }
    }

    /// Casts the coordinate system type parameter of the pose to the equivalent coordinate
    /// system `NewIn`.
    ///
    /// See [`EquivalentTo`] for details on when this is useful (and safe).
    ///
    /// Note that this performs no transform on the pose's constituent parts, as that should be
    /// unnecessary when `EquivalentTo` is implemented.
    ///
    /// ```
    /// use sguaba::{coordinate, system, systems::EquivalentTo, Bearing, Coordinate, engineering::{Orientation, Pose}};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct PlaneNedFromCrate1 using NED);
    /// system!(struct PlaneNedFromCrate2 using NED);
    ///
    /// // SAFETY: these are truly the same thing just defined in different places
    /// unsafe impl EquivalentTo<PlaneNedFromCrate1> for PlaneNedFromCrate2 {}
    /// unsafe impl EquivalentTo<PlaneNedFromCrate2> for PlaneNedFromCrate1 {}
    ///
    /// let position_in_1 = coordinate! {
    ///     n = Length::new::<meter>(1.),
    ///     e = Length::new::<meter>(2.),
    ///     d = Length::new::<meter>(3.);
    ///     in PlaneNedFromCrate1
    /// };
    /// let orientation_in_1 = Orientation::<PlaneNedFromCrate1>::from_tait_bryan_angles(
    ///     Angle::new::<degree>(90.),
    ///     Angle::new::<degree>(45.),
    ///     Angle::new::<degree>(5.),
    /// );
    /// let pose_in_1 = Pose::<PlaneNedFromCrate1>::new(
    ///     position_in_1,
    ///     orientation_in_1,
    /// );
    ///
    /// assert_eq!(
    ///     Pose::<PlaneNedFromCrate2>::new(
    ///         coordinate! {
    ///             n = Length::new::<meter>(1.),
    ///             e = Length::new::<meter>(2.),
    ///             d = Length::new::<meter>(3.),
    ///         },
    ///         Orientation::<PlaneNedFromCrate2>::from_tait_bryan_angles(
    ///             Angle::new::<degree>(90.),
    ///             Angle::new::<degree>(45.),
    ///             Angle::new::<degree>(5.),
    ///         ),
    ///     ),
    ///     pose_in_1.cast::<PlaneNedFromCrate2>()
    /// );
    /// ```
    #[must_use]
    pub fn cast<NewIn>(self) -> Pose<NewIn>
    where
        In: EquivalentTo<NewIn>,
    {
        Pose {
            inner: self.inner.cast_type_of_from::<NewIn>(),
        }
    }

    /// Linearly interpolate between this pose and another one.
    ///
    /// Conceptually returns `self * (1.0 - t) + rhs * t`, i.e., the linear blend of the two
    /// poses using the scalar value `t`.
    ///
    /// Internally, this linearly interpolates the position and orientation separately.
    ///
    /// The value for `t` is not restricted to the range [0, 1].
    #[must_use]
    pub fn lerp(&self, rhs: &Self, t: f64) -> Self {
        Self {
            inner: self.inner.lerp(&rhs.inner, t),
        }
    }
}

impl<In> Default for Pose<In> {
    fn default() -> Self {
        Self::new(Coordinate::default(), Orientation::default())
    }
}

impl<In> Pose<In> {
    /// Returns the position of the object with this pose.
    #[must_use]
    pub fn position(&self) -> Coordinate<In> {
        Coordinate::from_nalgebra_point(Point3::from(self.inner.translation().inner))
    }

    /// Returns the orientation of the object with this pose.
    #[must_use]
    pub fn orientation(&self) -> Orientation<In> {
        Orientation {
            inner: self.inner.rotation(),
        }
    }

    /// Returns the distance from the origin to the object with this pose.
    #[must_use]
    pub fn distance_from_origin(&self) -> Length {
        self.inner.translation().magnitude()
    }
}

impl<From, To> Mul<RigidBodyTransform<From, To>> for Pose<From> {
    type Output = Pose<To>;

    fn mul(self, rhs: RigidBodyTransform<From, To>) -> Self::Output {
        // trivially, this is:
        //
        if false {
            let _ = rhs.inverse() * self;
            // which also typechecks!
        }
        //
        // but we'd like to avoid the inverse intermediate
        //
        // if we inline .inverse(), this is really
        //
        //     RigidBodyTransform::<To, From> {
        //         inner: rhs.inverse(),
        //         from: PhantomData,
        //         to: PhantomData,
        //     } * self
        //
        // which if we inline the Mul below (which ^ uses) is really
        //
        //     Pose {
        //         inner: RigidBodyTransform {
        //             inner: rhs.inverse() * self.inner,
        //             from: PhantomData,
        //             to: PhantomData,
        //         },
        //     }
        //
        // which if we inline the Mul of RigidBodyTransforms is really
        //
        //     Pose {
        //         inner: RigidBodyTransform {
        //             inner: rhs.inverse().inner * self.inner.inner,
        //             from: PhantomData,
        //             to: PhantomData,
        //         },
        //     }
        //
        // so an inverse and multiply of nalgebra::Isometry3, which
        // the nalgebra docs tell us we can do more efficiently
        // (ie, without an inverse) with inv_mul:
        Pose {
            inner: RigidBodyTransform {
                inner: rhs.inner.inv_mul(&self.inner.inner),
                from: rhs.to,
                to: self.inner.to,
            },
        }
    }
}

impl<From, To> Mul<Pose<To>> for RigidBodyTransform<From, To> {
    type Output = Pose<From>;

    fn mul(self, rhs: Pose<To>) -> Self::Output {
        Pose {
            inner: self * rhs.inner,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::coordinate_systems::{Ecef, Ned};
    use crate::coordinates::Coordinate;
    use crate::directions::{Bearing, Components as BearingComponents};
    use crate::engineering::{Orientation, Pose};
    use crate::float_math::FloatMath;
    use crate::geodetic::{Components as Wgs84Components, Wgs84};
    use crate::math::{RigidBodyTransform, Rotation};
    use crate::util::BoundedAngle;
    use crate::vectors::Vector;
    use crate::{coordinate, Point3};
    use approx::assert_relative_eq;
    use approx::{assert_abs_diff_eq, AbsDiffEq};
    use rstest::rstest;
    use std::println;
    use uom::si::f64::{Angle, Length};
    use uom::si::{
        angle::{degree, radian},
        length::meter,
    };

    fn m(meters: f64) -> Length {
        Length::new::<meter>(meters)
    }
    fn r(radians: f64) -> Angle {
        Angle::new::<radian>(radians)
    }
    fn d(degrees: f64) -> Angle {
        Angle::new::<degree>(degrees)
    }

    system!(struct PlaneFrd using FRD);
    system!(struct PlaneNed using NED);
    system!(struct PlaneBNed using NED);
    system!(struct SensorFrd using FRD);

    #[test]
    fn usecase_1_where_is_the_object_in_world() {
        // Objective:
        // Platform observes point in azimuth, elevation, range of its body coordinate system (FRD).
        // Where is it in WGS84?
        let observation = Coordinate::<PlaneFrd>::from_bearing(
            Bearing::build(BearingComponents {
                azimuth: d(20.),
                elevation: d(10.),
            })
            .expect("elevation is in-range"),
            m(400.),
        );

        // The pilot also knows where the plane is located given by a GPS device.
        let wgs84 = Wgs84::build(Wgs84Components {
            latitude: d(12.),
            longitude: d(30.),
            altitude: m(1000.),
        })
        .expect("latitude is in-range");

        // The API allows two compatible paths.
        // One for thinking about transformations between coordinate systems.
        // One for thinking about objects in a coordinate system. Their pose or orientation.

        // The pilot can read the instrument of the planes orientation relative to the local NED.
        // The instruments give the orientation as yaw, pitch, roll
        // The pilot knows that these are the tait-bryan angles we expect to get.
        // Here the plane is pitched 45 degrees upwards.
        let orientation_in_ned = Orientation::<PlaneNed>::tait_bryan_builder()
            .yaw(d(0.))
            .pitch(d(45.))
            .roll(d(0.))
            .build();

        // And the pilot knows from pilot school, that the pose NED of the plane is defined by the WGS84 coordinate.
        // Also, the pilot knows that ECEF is a cartesian representation of WGS84.
        let ecef_to_plane_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&wgs84) };

        // Option 1: Transformation in world.

        // Now the pilot also needs to account for the planes orientation in NED.
        // This is done by creating a rotation transform
        let plane_ned_to_plane_frd = unsafe { orientation_in_ned.map_as_zero_in::<PlaneFrd>() };

        // Chaining them together results in the transformation from ECEF to FRD
        let world_to_plane_frd = ecef_to_plane_ned.and_then(plane_ned_to_plane_frd);

        // Now the pilot can convert the observed point to the world
        let observation_in_world_eng1 = world_to_plane_frd.inverse_transform(observation);

        println!("{:?}", observation_in_world_eng1.to_wgs84());

        // Option 2: Pose in world.

        // Alternatively the pilot could create the pose of the plane in world as
        let pose_in_world = ecef_to_plane_ned.inverse_transform(orientation_in_ned);
        // And then create the transformation from it
        let transformation_world_to_frd_eng2 =
            unsafe { pose_in_world.map_as_zero_in::<PlaneFrd>() };

        let observation_in_world_eng2 =
            transformation_world_to_frd_eng2.inverse_transform(observation);

        assert_relative_eq!(observation_in_world_eng1, observation_in_world_eng2);

        // Option 3: Transformations all the way.

        // If the pilot studied math and just wants to use transformations
        let ecef_to_ned = unsafe { RigidBodyTransform::<Ecef, PlaneNed>::ecef_to_ned_at(&wgs84) };
        let ned_to_frd = unsafe {
            Rotation::tait_bryan_builder()
                .yaw(d(0.))
                .pitch(d(45.))
                .roll(d(0.))
                .build()
        };
        // Think of matrix multiplication
        let ecef_to_frd = ecef_to_ned * ned_to_frd;
        let observation_in_world_math = ecef_to_frd.inverse_transform(observation);

        assert_relative_eq!(observation_in_world_math, observation_in_world_eng1);
    }

    #[test]
    fn usecase_2_4() {
        // There are two airplanes with known poses in the world.

        // 1. How far are they apart?
        // 2. What is the pose of one airplane with respect to the other?
        // 3. If pilot of plane A looks towards plane B, in which direction does the pilot have to look?

        let position_plane_a = Wgs84::build(Wgs84Components {
            latitude: d(12.),
            longitude: d(30.2),
            altitude: m(1000.),
        })
        .expect("latitude is in-range");
        let position_plane_b = Wgs84::build(Wgs84Components {
            latitude: d(12.),
            longitude: d(30.),
            altitude: m(1000.),
        })
        .expect("latitude is in-range");

        // Pilot A reads plane A instruments.
        let orientation_plane_a_in_ned = Orientation::<PlaneNed>::tait_bryan_builder()
            .yaw(d(0.))
            .pitch(d(45.))
            .roll(d(0.))
            .build();

        // Pilot B reads plane B instruments.
        let orientation_plane_b_in_ned = Orientation::tait_bryan_builder()
            .yaw(d(20.))
            .pitch(d(12.))
            .roll(d(0.))
            .build();

        // Now both pilots can get the pose of their plane in the world
        let ecef_to_plane_a_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&position_plane_a) };
        let pose_plane_a_in_world =
            ecef_to_plane_a_ned.inverse_transform(orientation_plane_a_in_ned);

        let ecef_to_plane_b_ned =
            unsafe { RigidBodyTransform::<Ecef, PlaneBNed>::ecef_to_ned_at(&position_plane_b) };
        let pose_plane_b_in_world =
            ecef_to_plane_b_ned.inverse_transform(orientation_plane_b_in_ned);

        // Question 1: How far are they apart?
        // There are multiple options to answer this.
        // 1. Compute the approx. distance using WGS84 on the surface of the earth.
        let _surface_distance = position_plane_b.haversine_distance_on_surface(&position_plane_a);
        // 2. Compute the free space distance using ECEF
        let free_space_distance = (Coordinate::from_wgs84(&position_plane_b)
            - Coordinate::from_wgs84(&position_plane_a))
        .magnitude();
        // 3. Use the positions in the poses.
        let free_space_distance_from_pose =
            (pose_plane_b_in_world.position() - pose_plane_a_in_world.position()).magnitude();
        assert_eq!(free_space_distance, free_space_distance_from_pose);

        // Question 2: What is the pose of one airplane with respect to the other?
        let plane_a_frd_to_ecef =
            unsafe { pose_plane_a_in_world.map_as_zero_in::<PlaneFrd>() }.inverse();
        let pose_plane_b_for_plane_a = plane_a_frd_to_ecef.inverse_transform(pose_plane_b_in_world);
        // If the pilot wants the transformation from Plane A FRD to plane B FRD, there it can be constructed directly without this pose.

        // Question 3: If pilot of plane A looks towards plane B, in which direction does the pilot have to look?
        let direction_to_plane_b_for_plane_a = pose_plane_b_for_plane_a
            .position()
            .bearing_from_origin()
            .expect("azimuth is well-defined");
        println!(
            "Direction towards plane B as observed by plane A {direction_to_plane_b_for_plane_a}"
        );
    }

    #[rstest]
    #[case(
        Point3::new(10., 0., 0.),
        (d(0.), d(0.), d(0.)),
        Bearing::build(BearingComponents { azimuth: d(0.), elevation: d(0.) }).unwrap()
    )]
    #[case(
        Point3::new(0., 10., 0.),
        (d(0.), d(0.), d(0.)),
        Bearing::build(BearingComponents { azimuth: d(90.), elevation: d(0.) }).unwrap()
    )]
    #[case(
        Point3::new(-10., 0., 0.),
        (d(0.), d(0.), d(0.)),
        Bearing::build(BearingComponents { azimuth: d(-180.), elevation: d(0.) }).unwrap()
    )]
    #[case(
        Point3::new(0., -10., 0.),
        (d(0.), d(0.), d(0.)),
        Bearing::build(BearingComponents { azimuth: d(270.), elevation: d(0.) }).unwrap()
    )]
    #[case(
        Point3::new(0., 0., 10.),
        (d(0.), d(0.), d(0.)),
        Bearing::build(BearingComponents { azimuth: d(0.), elevation: d(-90.) }).unwrap()
    )]
    #[case(
        Point3::new(0., -10., 10.),
        (d(0.), d(0.), d(0.)),
        Bearing::build(BearingComponents { azimuth: d(270.), elevation: d(-45.) }).unwrap()
    )]
    #[case(
        Point3::new(10., 10., -10.),
        (d(0.), d(0.), d(0.)),
        Bearing::build(BearingComponents { azimuth: d(45.), elevation: r(FloatMath::asin(10. / FloatMath::sqrt(FloatMath::powi(10_f64, 2) * 3.))) }).unwrap()
    )]
    fn pose_direction_towards(
        #[case] position: Point3,
        #[case] ypr: (Angle, Angle, Angle),
        #[case] expected: Bearing<Ned>,
    ) {
        let (yaw, pitch, roll) = ypr;
        let pose = Pose::<Ned>::new(
            Coordinate::from_nalgebra_point(position),
            Orientation::tait_bryan_builder()
                .yaw(yaw)
                .pitch(pitch)
                .roll(roll)
                .build(),
        );

        // also double-check the sanity of to_tait_bryan_angles
        if pitch <= d(90.) {
            let (y, p, r) = pose.orientation().to_tait_bryan_angles();
            for (a, b) in [(y, yaw), (p, pitch), (r, roll)] {
                assert_abs_diff_eq!(&BoundedAngle::new(a), &BoundedAngle::new(b),);
            }
        }

        let direction = pose.position().bearing_from_origin().unwrap();

        assert_abs_diff_eq!(direction, expected);
    }

    #[test]
    fn orientation_inverse_works() {
        let ned_to_frd = unsafe {
            Rotation::<PlaneNed, PlaneFrd>::tait_bryan_builder()
                .yaw(d(45.))
                .pitch(d(85.))
                .roll(d(150.))
                .build()
        };
        let frd_to_ned = ned_to_frd.inverse();

        let frd = coordinate!(f = m(42.), r = m(69.), d = m(99.); in PlaneFrd);
        assert_relative_eq!(ned_to_frd.inverse_transform(frd), frd_to_ned.transform(frd));

        let ned = coordinate!(n = m(42.), e = m(69.), d = m(99.); in PlaneNed);

        assert_relative_eq!(ned_to_frd.transform(ned), frd_to_ned.inverse_transform(ned));
    }

    #[test]
    fn pose_multiplication_works() {
        // Transforms a NED to an FRD coordinate of an object with this yaw, pitch, roll.
        let ned_to_frd = unsafe {
            RigidBodyTransform::<PlaneNed, PlaneFrd>::new(
                Vector::<PlaneNed>::zero(),
                Rotation::tait_bryan_builder()
                    .yaw(d(90.))
                    .pitch(d(90.))
                    .roll(d(0.))
                    .build(),
            )
        };

        let wgs84 = Wgs84::build(Wgs84Components {
            latitude: d(52.),
            longitude: d(-3.),
            altitude: m(1000.),
        })
        .expect("latitude is in-range");

        // Define the pose of a NED at a specific location. This is where altitude comes in.
        let ecef_to_ned = unsafe { RigidBodyTransform::<Ecef, PlaneNed>::ecef_to_ned_at(&wgs84) };

        // Chains both transformations to transform a ECEF coordinate to a Frd Coordinate via NED.
        let ecef_to_frd = ecef_to_ned * ned_to_frd;

        let forward = coordinate!(f = m(1.), r = m(0.), d = m(0.); in PlaneFrd);
        let right = coordinate!(f = m(0.), r = m(1.), d = m(0.); in PlaneFrd);
        let down = coordinate!(f = m(0.), r = m(0.), d = m(1.); in PlaneFrd);

        // check that the FRD axis are at the right spots in NED.
        let forward_in_ned = ned_to_frd.inverse_transform(forward);
        let right_in_ned = ned_to_frd.inverse_transform(right);
        let down_in_ned = ned_to_frd.inverse_transform(down);
        // The origin is the same for FRD to NED so these are just rotated.
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

        // Check that the chain did the same as the manual steps.
        assert_relative_eq!(forward_in_ecef, ecef_to_frd.inverse_transform(forward));
        assert_relative_eq!(
            frd_to_ecef.transform(forward),
            ecef_to_frd.inverse_transform(forward),
        );

        assert_relative_eq!(right_in_ecef, ecef_to_frd.inverse_transform(right));
        assert_relative_eq!(
            frd_to_ecef.transform(right),
            ecef_to_frd.inverse_transform(right),
        );

        assert_relative_eq!(down_in_ecef, ecef_to_frd.inverse_transform(down));
        assert_relative_eq!(
            frd_to_ecef.transform(down),
            ecef_to_frd.inverse_transform(down),
        );

        // ECEF of the origin of the FRD should be at the emitter position.
        assert_relative_eq!(
            Coordinate::<Ecef>::from_wgs84(&wgs84),
            ecef_to_frd.inverse_transform(Coordinate::<PlaneFrd>::origin()),
        );

        // The distance between two points should stay the same after transformation.
        let point_a_frd = coordinate!(f = m(20.), r = m(-45.), d = m(10.); in PlaneFrd);
        let point_b_frd = coordinate!(f = m(100.), r = m(-200.), d = m(50.); in PlaneFrd);

        assert_relative_eq!(
            (point_b_frd - point_a_frd).magnitude().get::<meter>(),
            (ecef_to_frd.inverse_transform(point_b_frd)
                - ecef_to_frd.inverse_transform(point_a_frd))
            .magnitude()
            .get::<meter>(),
            epsilon = Coordinate::<Ecef>::default_epsilon().get::<meter>(),
        );

        // Now test if we can go to another FRD

        // Pose of a second frd in another ned
        let ned_to_frd_2 = unsafe {
            RigidBodyTransform::<PlaneBNed, SensorFrd>::new(
                Vector::<PlaneBNed>::zero(),
                Rotation::tait_bryan_builder()
                    .yaw(d(-45.))
                    .pitch(d(0.))
                    .roll(d(0.))
                    .build(),
            )
        };

        // NED of the other FRDs object
        let ecef_to_ned_2 = unsafe {
            RigidBodyTransform::<Ecef, PlaneBNed>::ecef_to_ned_at(
                &Wgs84::build(Wgs84Components {
                    latitude: d(54.),
                    longitude: d(-3.67),
                    altitude: m(0.),
                })
                .expect("latitude is in-range"),
            )
        };

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
}
