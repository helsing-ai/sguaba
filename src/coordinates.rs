use crate::coordinate_systems::{CoordinateSystem, FrdLike, NedLike, RightHandedXyzLike};
use crate::directions::Bearing;
use crate::math::RigidBodyTransform;
use crate::systems::EquivalentTo;
use crate::vectors::Vector;
use crate::{engineering, Point3};
use std::fmt;
use std::fmt::{Display, Formatter};
use std::marker::PhantomData;
use std::ops::{Add, AddAssign, Neg, Sub, SubAssign};
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;
use uom::si::quantities::Ratio;
use uom::typenum::P2;
use uom::ConstZero;

#[cfg(any(test, feature = "approx"))]
use approx::{AbsDiffEq, RelativeEq};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(doc)]
use crate::{engineering::Pose, systems::BearingDefined};

/// Defines a point (ie, position) in the coordinate system specified by `In`.
///
/// You can construct one using [cartesian](Coordinate::from_cartesian) or
/// [spherical](Coordinate::from_spherical) coordinates, or using [bearing +
/// range](Coordinate::from_bearing).
///
/// Depending on the convention of the coordinate system (eg, [`NedLike`], [`FrdLike`], or
/// [`RightHandedXyzLike`]), you'll have different appropriately-named accessors for the
/// coordinate's cartesian components like [`Coordinate::ned_north`] or [`Coordinate::frd_front`].
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
// no need for the "point": indirection
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct Coordinate<In> {
    /// X, Y, Z in meters
    pub(crate) point: Point3,
    #[cfg_attr(feature = "serde", serde(skip))]
    system: PhantomData<In>,
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<In> Clone for Coordinate<In> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<In> Copy for Coordinate<In> {}

/// Components for constructing a [`Coordinate`].
///
/// This struct provides named fields to avoid confusion about argument order
/// when constructing a coordinate.
///
/// # Example
///
/// ```
/// use sguaba::{Coordinate, coordinate_systems::Ned};
/// use sguaba::coordinates::CoordinateComponents;
/// use uom::si::length::meter;
/// use uom::si::f64::Length;
///
/// let components = CoordinateComponents {
///     x: Length::new::<meter>(100.0),
///     y: Length::new::<meter>(50.0),
///     z: Length::new::<meter>(-10.0),
/// };
/// let coord = Coordinate::<Ned>::from_components(components);
/// ```
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CoordinateComponents {
    /// The x component of the coordinate.
    pub x: Length,
    /// The y component of the coordinate.
    pub y: Length,
    /// The z component of the coordinate.
    pub z: Length,
}

/// Builder for constructing a [`Coordinate`] with named parameters.
///
/// This builder uses the type-state pattern to ensure x, y, and z
/// are all set before the coordinate can be built.
///
/// # Example
///
/// ```
/// use sguaba::{Coordinate, coordinate_systems::Ned};
/// use uom::si::length::meter;
/// use uom::si::f64::Length;
///
/// let coord = Coordinate::<Ned>::builder()
///     .x(Length::new::<meter>(100.0))
///     .y(Length::new::<meter>(50.0))
///     .z(Length::new::<meter>(-10.0))
///     .build();
/// ```
#[derive(Debug)]
pub struct CoordinateBuilder<In, XSet = (), YSet = (), ZSet = ()> {
    x: Option<Length>,
    y: Option<Length>,
    z: Option<Length>,
    _phantom: PhantomData<(In, XSet, YSet, ZSet)>,
}

/// Marker type indicating that x has been set in the builder.
#[derive(Debug)]
pub struct XSet;

/// Marker type indicating that y has been set in the builder.
#[derive(Debug)]
pub struct YSet;

/// Marker type indicating that z has been set in the builder.
#[derive(Debug)]
pub struct ZSet;

impl<In> CoordinateBuilder<In, (), (), ()> {
    fn new() -> Self {
        Self {
            x: None,
            y: None,
            z: None,
            _phantom: PhantomData,
        }
    }
}

impl<In, Y, Z> CoordinateBuilder<In, (), Y, Z> {
    /// Sets the x component for the coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Coordinate, coordinate_systems::Ned};
    /// use uom::si::length::meter;
    /// use uom::si::f64::Length;
    ///
    /// let builder = Coordinate::<Ned>::builder()
    ///     .x(Length::new::<meter>(100.0));
    /// ```
    pub fn x(mut self, x: impl Into<Length>) -> CoordinateBuilder<In, XSet, Y, Z> {
        self.x = Some(x.into());
        CoordinateBuilder {
            x: self.x,
            y: self.y,
            z: self.z,
            _phantom: PhantomData,
        }
    }
}

impl<In, X, Z> CoordinateBuilder<In, X, (), Z> {
    /// Sets the y component for the coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Coordinate, coordinate_systems::Ned};
    /// use uom::si::length::meter;
    /// use uom::si::f64::Length;
    ///
    /// let builder = Coordinate::<Ned>::builder()
    ///     .y(Length::new::<meter>(50.0));
    /// ```
    pub fn y(mut self, y: impl Into<Length>) -> CoordinateBuilder<In, X, YSet, Z> {
        self.y = Some(y.into());
        CoordinateBuilder {
            x: self.x,
            y: self.y,
            z: self.z,
            _phantom: PhantomData,
        }
    }
}

impl<In, X, Y> CoordinateBuilder<In, X, Y, ()> {
    /// Sets the z component for the coordinate.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Coordinate, coordinate_systems::Ned};
    /// use uom::si::length::meter;
    /// use uom::si::f64::Length;
    ///
    /// let builder = Coordinate::<Ned>::builder()
    ///     .z(Length::new::<meter>(-10.0));
    /// ```
    pub fn z(mut self, z: impl Into<Length>) -> CoordinateBuilder<In, X, Y, ZSet> {
        self.z = Some(z.into());
        CoordinateBuilder {
            x: self.x,
            y: self.y,
            z: self.z,
            _phantom: PhantomData,
        }
    }
}

impl<In> CoordinateBuilder<In, XSet, YSet, ZSet> {
    /// Builds the [`Coordinate`] from the configured values.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Coordinate, coordinate_systems::Ned};
    /// use uom::si::length::meter;
    /// use uom::si::f64::Length;
    ///
    /// let coord = Coordinate::<Ned>::builder()
    ///     .x(Length::new::<meter>(100.0))
    ///     .y(Length::new::<meter>(50.0))
    ///     .z(Length::new::<meter>(-10.0))
    ///     .build();
    ///
    /// assert_eq!(coord.ned_north().get::<meter>(), 100.0);
    /// assert_eq!(coord.ned_east().get::<meter>(), 50.0);
    /// assert_eq!(coord.ned_down().get::<meter>(), -10.0);
    /// ```
    #[must_use]
    pub fn build(self) -> Coordinate<In> {
        Coordinate::from_nalgebra_point(Point3::new(
            self.x.unwrap().get::<meter>(),
            self.y.unwrap().get::<meter>(),
            self.z.unwrap().get::<meter>(),
        ))
    }
}

impl<In> Coordinate<In> {
    pub(crate) fn from_nalgebra_point(p: Point3) -> Self {
        Self {
            point: p,
            system: PhantomData,
        }
    }

    /// Constructs a coordinate at the given (x, y, z) cartesian point in the [`CoordinateSystem`]
    /// `In`.
    ///
    /// The meaning of `x`, `y`, and `z` is dictated by the [`CoordinateSystem::Convention`] of
    /// `In`. For example, in [`NedLike`], `x` is North, `y` is East, and `z` is "down" (ie,
    /// orthogonal to the earth's surface).
    ///
    /// **Note**: This constructor is deprecated. Use [`Coordinate::builder`] or
    /// [`Coordinate::from_components`] instead to avoid confusion about argument order.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Coordinate, coordinate_systems::Ned};
    /// use uom::si::length::meter;
    /// use uom::si::f64::Length;
    ///
    /// #[allow(deprecated)]
    /// let coord = Coordinate::<Ned>::from_cartesian(
    ///     Length::new::<meter>(100.0),  // North
    ///     Length::new::<meter>(50.0),   // East
    ///     Length::new::<meter>(-10.0)   // Down
    /// );
    /// ```
    #[deprecated(
        since = "0.10.0",
        note = "Use `Coordinate::builder()` or `Coordinate::from_components()` instead to avoid confusion about argument order"
    )]
    pub fn from_cartesian(
        x: impl Into<Length>,
        y: impl Into<Length>,
        z: impl Into<Length>,
    ) -> Self {
        Self::from_nalgebra_point(Point3::new(
            x.into().get::<meter>(),
            y.into().get::<meter>(),
            z.into().get::<meter>(),
        ))
    }

    /// Creates a new builder for constructing a [`Coordinate`].
    ///
    /// This builder ensures that x, y, and z are explicitly set
    /// before the coordinate can be built, preventing confusion about argument order.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Coordinate, coordinate_systems::Ned};
    /// use uom::si::length::meter;
    /// use uom::si::f64::Length;
    ///
    /// let coord = Coordinate::<Ned>::builder()
    ///     .x(Length::new::<meter>(100.0))
    ///     .y(Length::new::<meter>(50.0))
    ///     .z(Length::new::<meter>(-10.0))
    ///     .build();
    /// ```
    pub fn builder() -> CoordinateBuilder<In, (), (), ()> {
        CoordinateBuilder::new()
    }

    /// Constructs a coordinate from a [`CoordinateComponents`] struct.
    ///
    /// This constructor ensures that x, y, and z are explicitly named,
    /// preventing confusion about argument order.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Coordinate, coordinate_systems::Ned};
    /// use sguaba::coordinates::CoordinateComponents;
    /// use uom::si::length::meter;
    /// use uom::si::f64::Length;
    ///
    /// let coord = Coordinate::<Ned>::from_components(CoordinateComponents {
    ///     x: Length::new::<meter>(100.0),
    ///     y: Length::new::<meter>(50.0),
    ///     z: Length::new::<meter>(-10.0),
    /// });
    ///
    /// assert_eq!(coord.ned_north().get::<meter>(), 100.0);
    /// assert_eq!(coord.ned_east().get::<meter>(), 50.0);
    /// assert_eq!(coord.ned_down().get::<meter>(), -10.0);
    /// ```
    #[must_use]
    pub fn from_components(components: CoordinateComponents) -> Self {
        #[allow(deprecated)]
        Self::from_cartesian(components.x, components.y, components.z)
    }

    /// Constructs a coordinate at the given (r, θ, φ) spherical point in the [`CoordinateSystem`]
    /// `In`.
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
    /// [`Coordinate::from_bearing`].
    ///
    /// </div>
    ///
    /// # Examples
    ///
    /// ```rust
    /// use approx::assert_relative_eq;
    /// use sguaba::{system, Coordinate};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct Ned using NED);
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// #[allow(deprecated)]
    /// {
    /// assert_relative_eq!(
    ///     Coordinate::<Ned>::from_spherical(unit, Angle::new::<degree>(0.), Angle::new::<degree>(0.)),
    ///     Coordinate::<Ned>::from_cartesian(zero, zero, unit),
    /// );
    /// assert_relative_eq!(
    ///     Coordinate::<Ned>::from_spherical(unit, Angle::new::<degree>(90.), Angle::new::<degree>(0.)),
    ///     Coordinate::<Ned>::from_cartesian(unit, zero, zero),
    /// );
    /// assert_relative_eq!(
    ///     Coordinate::<Ned>::from_spherical(unit, Angle::new::<degree>(90.), Angle::new::<degree>(90.)),
    ///     Coordinate::<Ned>::from_cartesian(zero, unit, zero),
    /// );
    /// }
    /// ```
    ///
    /// [sph]: https://en.wikipedia.org/wiki/Spherical_coordinate_system
    pub fn from_spherical(
        radius: impl Into<Length>,
        polar: impl Into<Angle>,
        azimuth: impl Into<Angle>,
    ) -> Self {
        let direction = Vector::from_spherical(radius, polar.into(), azimuth.into());
        Self::origin() + direction
    }

    /// Constructs a coordinate at the given azimuth, elevation, and range in the
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
    /// use sguaba::{system, Bearing, Coordinate};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct Ned using NED);
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// #[allow(deprecated)]
    /// {
    /// assert_relative_eq!(
    ///     Coordinate::<Ned>::from_bearing(Bearing::<Ned>::builder()
    ///       .azimuth(Angle::new::<degree>(0.))
    ///       .elevation(Angle::new::<degree>(0.))
    ///       .build(), unit),
    ///     Coordinate::<Ned>::from_cartesian(unit, zero, zero),
    /// );
    /// assert_relative_eq!(
    ///     Coordinate::<Ned>::from_bearing(Bearing::<Ned>::builder()
    ///       .azimuth(Angle::new::<degree>(90.))
    ///       .elevation(Angle::new::<degree>(0.))
    ///       .build(), unit),
    ///     Coordinate::<Ned>::from_cartesian(zero, unit, zero),
    /// );
    /// assert_relative_eq!(
    ///     Coordinate::<Ned>::from_bearing(Bearing::<Ned>::builder()
    ///       .azimuth(Angle::new::<degree>(90.))
    ///       .elevation(Angle::new::<degree>(90.))
    ///       .build(), unit),
    ///     Coordinate::<Ned>::from_cartesian(zero, zero, -unit),
    /// );
    /// }
    /// ```
    ///
    /// [bearing]: https://en.wikipedia.org/wiki/Bearing_%28navigation%29
    /// [azel]: https://en.wikipedia.org/wiki/Horizontal_coordinate_system
    pub fn from_bearing(bearing: Bearing<In>, range: impl Into<Length>) -> Self
    where
        In: crate::systems::BearingDefined,
    {
        let direction = Vector::from_bearing(bearing, range);
        Self::origin() + direction
    }

    /// Constructs a coordinate at the origin of the coordinate system `In`.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use sguaba::{system, Coordinate};
    /// use uom::si::f64::{Length};
    /// use uom::si::length::meter;
    ///
    /// system!(struct Ned using NED);
    ///
    /// let zero = Length::new::<meter>(0.);
    /// #[allow(deprecated)]
    /// assert_eq!(
    ///     Coordinate::<Ned>::origin(),
    ///     Coordinate::<Ned>::from_cartesian(zero, zero, zero),
    /// );
    /// ```
    #[must_use]
    pub fn origin() -> Self {
        Self {
            point: Point3::origin(),
            system: PhantomData,
        }
    }

    /// Constructs a translation into [`CoordinateSystem`] `To` such that `self` is "zero" in `To`.
    ///
    /// Less informally, if this translation is applied to this `Coordinate<In>`, it will yield
    /// [`Coordinate::origin`] in `Coordination<To>`.
    ///
    /// Conversely, if the inverse of this translation is applied to [`Coordinate::origin`] in
    /// `Orientation<To>`, it will yield this `Coordinate<In>`.
    ///
    /// Or, alternatively phrased, this yields a translation transformation that
    ///
    /// 1. takes a coordinate, vector, or orientation observed by the object with this position
    ///    in the coordinate system `In`; and
    /// 2. returns that coordinate or vector as if it were observed in a coordinate system (`To`)
    ///    where this `Coordinate<In>` is [`Coordinate::origin`].
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
    /// Specifically in the case of `Coordinate`, you are also asserting that _only_ translation
    /// (ie, no rotation) is needed to convert from `In` to `To`.
    ///
    /// </div>
    ///
    ///
    /// # Examples
    ///
    /// ```rust
    /// use approx::assert_relative_eq;
    /// use sguaba::{system, Bearing, Coordinate, engineering::Orientation, systems::Ecef};
    /// use uom::si::f64::Length;
    /// use uom::si::length::meter;
    ///
    /// system!(struct PlaneCenteredEcef using right-handed XYZ);
    ///
    /// // plane position in ECEF
    /// #[allow(deprecated)]
    /// let position = Coordinate::<Ecef>::from_cartesian(
    ///     Length::new::<meter>(30_000.),
    ///     Length::new::<meter>(25_000.),
    ///     Length::new::<meter>(19_000.),
    /// );
    ///
    ///
    /// // plane observes something at a particular ECEF coordinate
    /// #[allow(deprecated)]
    /// let observation = Coordinate::<PlaneCenteredEcef>::from_cartesian(
    ///     Length::new::<meter>(1_000.),
    ///     Length::new::<meter>(6_000.),
    ///     Length::new::<meter>(0.),
    /// );
    ///
    /// // if we now want this vector to be relative to the plane (ie, a direction
    /// // of arrival vector), we need to map from one space to the other. so, we
    /// // declare that the plane's position in ECEF is zero (ie, origin) in
    /// // `PlaneCenteredEcef`:
    /// let ecef_to_plane_centered_ecef = unsafe { position.map_as_zero_in::<PlaneCenteredEcef>() };
    ///
    /// // this transformation lets us move from ECEF to plane-centered ECEF
    /// // and vice-versa:
    /// let observation_in_ecef = ecef_to_plane_centered_ecef.inverse_transform(observation);
    /// ```
    #[doc(alias = "as_transform_to")]
    #[doc(alias = "defines_pose_of")]
    #[must_use]
    pub unsafe fn map_as_zero_in<To>(self) -> RigidBodyTransform<In, To> {
        unsafe {
            engineering::Pose::new(self, engineering::Orientation::aligned()).map_as_zero_in()
        }
    }

    /// Casts the coordinate system type parameter of the coordinate to the equivalent coordinate
    /// system `NewIn`.
    ///
    /// See [`EquivalentTo`] for details on when this is useful (and safe).
    ///
    /// Note that this performs no transform on the coordinate's components, as that should be
    /// unnecessary when `EquivalentTo` is implemented.
    ///
    /// ```
    /// use sguaba::{system, systems::EquivalentTo, Coordinate};
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
    /// #[allow(deprecated)]
    /// let coordinate_in_1 = Coordinate::<PlaneNedFromCrate1>::from_cartesian(unit, zero, unit);
    ///
    /// #[allow(deprecated)]
    /// assert_eq!(
    ///     Coordinate::<PlaneNedFromCrate2>::from_cartesian(
    ///       coordinate_in_1.ned_north(),
    ///       coordinate_in_1.ned_east(),
    ///       coordinate_in_1.ned_down(),
    ///     ),
    ///     coordinate_in_1.cast::<PlaneNedFromCrate2>()
    /// );
    /// ```
    #[must_use]
    pub fn cast<NewIn>(self) -> Coordinate<NewIn>
    where
        In: EquivalentTo<NewIn>,
    {
        Coordinate {
            point: self.point,
            system: PhantomData::<NewIn>,
        }
    }

    /// Linearly interpolate between this coordinate and another coordinate.
    ///
    /// Specifically, returns `self * (1.0 - t) + rhs * t`, i.e., the linear blend of the
    /// two coordinates using the scalar value `t`.
    ///
    /// The value for `t` is not restricted to the range [0, 1].
    #[must_use]
    pub fn lerp(&self, rhs: &Self, t: f64) -> Self {
        Self {
            point: self.point.lerp(&rhs.point, t),
            system: self.system,
        }
    }
}

impl<In> Default for Coordinate<In> {
    fn default() -> Self {
        Self::origin()
    }
}

macro_rules! accessors {
    {
        $convention:ident
        using $x:ident, $y:ident, $z:ident
        // TODO: https://github.com/rust-lang/rust/issues/124225
        + $x_ax:ident, $y_ax:ident, $z_ax:ident
    } => {
        impl<In> Coordinate<In> where In: CoordinateSystem<Convention = $convention> {
            #[must_use]
            pub fn $x(&self) -> Length { Length::new::<meter>(self.point.x) }
            #[must_use]
            pub fn $y(&self) -> Length { Length::new::<meter>(self.point.y) }
            #[must_use]
            pub fn $z(&self) -> Length { Length::new::<meter>(self.point.z) }

            #[must_use]
            pub fn $x_ax() -> Vector<In> { Vector::<In>::$x_ax() }
            #[must_use]
            pub fn $y_ax() -> Vector<In> { Vector::<In>::$y_ax() }
            #[must_use]
            pub fn $z_ax() -> Vector<In> { Vector::<In>::$z_ax() }
        }
    };
}

accessors!(RightHandedXyzLike using x, y, z + x_axis, y_axis, z_axis);
// NOTE(jon): it's sad that we need the ned_ and frd_ prefixes here, but it's because we need to
// disambigued FRD down and NED down, which is in turn necessary because Rust doesn't know that a
// coordinate system cannot implement `NedLike` _and_ `FrdLike`. Or rather, it _does_ know because
// a given type can only implement a non-generic trait once, but sadly it fails to realize that
// today (see https://github.com/rust-lang/rfcs/pull/1672).
accessors!(NedLike using ned_north, ned_east, ned_down + ned_north_axis, ned_east_axis, ned_down_axis);
accessors!(FrdLike using frd_front, frd_right, frd_down + frd_front_axis, frd_right_axis, frd_down_axis);

impl<In> Coordinate<In> {
    /// Returns the cartesian components of this coordinate in XYZ order.
    ///
    /// To turn this into a simple (ie, unitless) `[f64; 3]`, use [`array::map`] combined with
    /// `.get::<meter>()`.
    #[doc(alias = "components")]
    #[must_use]
    pub fn to_cartesian(&self) -> [Length; 3] {
        [
            Length::new::<meter>(self.point.x),
            Length::new::<meter>(self.point.y),
            Length::new::<meter>(self.point.z),
        ]
    }

    /// Computes the distance of this point from the coordinate system's origin.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use approx::assert_relative_eq;
    /// use sguaba::{Coordinate, Vector, systems::Ecef};
    /// use uom::si::f64::{Length};
    /// use uom::si:: length::meter;
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// #[allow(deprecated)]
    /// let p = Coordinate::<Ecef>::from_cartesian(unit, unit, zero);
    /// assert_eq!(
    ///     p.distance_from_origin(),
    ///     (p - Coordinate::<Ecef>::origin()).magnitude(),
    /// );
    /// ```
    #[doc(alias = "norm")]
    #[must_use]
    pub fn distance_from_origin(&self) -> Length {
        Length::new::<uom::si::length::meter>(self.point.coords.norm())
    }

    /// Computes the distance between this point and the given point.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use approx::assert_relative_eq;
    /// use sguaba::{Coordinate, Vector, systems::Ecef};
    /// use uom::si::f64::{Length};
    /// use uom::si:: length::meter;
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// #[allow(deprecated)]
    /// let p = Coordinate::<Ecef>::from_cartesian(unit, unit, zero);
    /// assert_eq!(
    ///     p.distance_from(&Coordinate::<Ecef>::origin()),
    ///     p.distance_from_origin(),
    /// );
    /// assert_eq!(
    ///     p.distance_from(&p),
    ///     zero,
    /// );
    /// ```
    #[must_use]
    pub fn distance_from(&self, other: &Coordinate<In>) -> Length {
        (*other - *self).magnitude()
    }

    fn spherical_coordinates(&self) -> Option<(Angle, Angle)> {
        let r = self.distance_from_origin();

        if r == Length::ZERO {
            return None;
        }

        let polar = Ratio::from(Length::new::<meter>(self.point.z) / r).acos();
        let xy = (Length::new::<meter>(self.point.x).powi(P2::new())
            + Length::new::<meter>(self.point.y).powi(P2::new()))
        .sqrt();
        if xy == Length::ZERO {
            // as promised by bearing_from_origin
            return Some((polar, Angle::ZERO));
        }
        let mut azimuth = Ratio::from(Length::new::<meter>(self.point.x) / xy).acos();
        azimuth.value = azimuth.value.copysign(self.point.y);

        Some((polar, azimuth))
    }

    /// Calculates the bearing towards the point from [`Coordinate::origin`].
    ///
    /// Returns `None` for [`Coordinate::origin`] as the azimuth to it is ill-defined.
    ///
    /// Returns an azimuth of zero for coordinates along the Z axis.
    #[must_use]
    pub fn bearing_from_origin(&self) -> Option<Bearing<In>>
    where
        In: crate::systems::BearingDefined,
    {
        // first, we compute the spherical coordinates,
        // and then we can convert them into bearing.
        let (polar, azimuth) = self.spherical_coordinates()?;
        Some(
            In::spherical_to_bearing(polar, azimuth)
                .expect("polar is computed with acos, which is always 0°-180°"),
        )
    }
}

impl<In> PartialEq<Self> for Coordinate<In> {
    fn eq(&self, other: &Self) -> bool {
        self.point.eq(&other.point)
    }
}

#[cfg(any(test, feature = "approx"))]
impl<In> AbsDiffEq<Self> for Coordinate<In> {
    type Epsilon = Length;

    fn default_epsilon() -> Self::Epsilon {
        // NOTE(jon): this value is in Meters, and realistically we're fine with .1m precision
        Length::new::<meter>(0.1)
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        // NOTE(jon): this measures whether the absolute difference in any _one_ coordinate is off
        // by more than epsilon, not whether the magnitude of the vector between the coordinates is
        // below epsilon.
        self.point.abs_diff_eq(&other.point, epsilon.get::<meter>())
    }
}

#[cfg(any(test, feature = "approx"))]
impl<In> RelativeEq for Coordinate<In> {
    fn default_max_relative() -> Self::Epsilon {
        Length::new::<meter>(Point3::default_max_relative())
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        self.point.relative_eq(
            &other.point,
            epsilon.get::<meter>(),
            max_relative.get::<meter>(),
        )
    }
}

impl<In> Display for Coordinate<In> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.point)
    }
}

impl<In> Neg for Coordinate<In> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            point: -self.point,
            system: self.system,
        }
    }
}

impl<In> Sub<Self> for Coordinate<In> {
    type Output = Vector<In>;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector::from_nalgebra_vector(self.point - rhs.point)
    }
}

impl<In> Add<Vector<In>> for Coordinate<In> {
    type Output = Coordinate<In>;

    fn add(self, rhs: Vector<In>) -> Self::Output {
        Coordinate {
            point: self.point + rhs.inner,
            system: self.system,
        }
    }
}

impl<In> AddAssign<Vector<In>> for Coordinate<In> {
    fn add_assign(&mut self, rhs: Vector<In>) {
        self.point += rhs.inner;
    }
}

impl<In> Sub<Vector<In>> for Coordinate<In> {
    type Output = Coordinate<In>;

    fn sub(self, rhs: Vector<In>) -> Self::Output {
        Coordinate {
            point: self.point - rhs.inner,
            system: self.system,
        }
    }
}

impl<In> SubAssign<Vector<In>> for Coordinate<In> {
    fn sub_assign(&mut self, rhs: Vector<In>) {
        self.point -= rhs.inner;
    }
}

#[cfg(test)]
mod tests {
    use super::{CoordinateComponents, Length};
    use crate::coordinate_systems::{Ecef, Frd, Ned};
    use crate::coordinates::Coordinate;
    use crate::Point3;
    use approx::assert_relative_eq;
    use rstest::rstest;
    use uom::si::length::meter;

    fn m(meters: f64) -> Length {
        Length::new::<uom::si::length::meter>(meters)
    }

    #[rstest]
    #[case(Point3::new(500., 0., 0.), m(500.))]
    #[case(Point3::new(0., 300., 0.), m(300.))]
    #[case(Point3::new(0., 0., 200.), m(200.))]
    #[case(Point3::new(-500., 0., 0.), m(500.))]
    #[case(Point3::new(0., -300., 0.), m(300.))]
    #[case(Point3::new(0., 0., -200.), m(200.))]
    fn frd_coordinate_distance_to_origin(#[case] point_in_a: Point3, #[case] expected: Length) {
        let coordinate = Coordinate::<Frd>::from_nalgebra_point(point_in_a);
        assert_eq!(coordinate.distance_from_origin(), expected);
    }

    #[test]
    #[allow(deprecated)]
    fn neg_works() {
        let frd = Coordinate::<Frd>::from_cartesian(m(10.), m(-5.), m(3.5));
        let ned = Coordinate::<Ned>::from_cartesian(m(10.), m(-5.), m(3.5));
        let ecef = Coordinate::<Ecef>::from_cartesian(m(10.), m(-5.), m(3.5));

        assert_relative_eq!(
            -frd,
            Coordinate::<Frd>::from_cartesian(m(-10.), m(5.), m(-3.5))
        );

        assert_relative_eq!(
            -ned,
            Coordinate::<Ned>::from_cartesian(m(-10.), m(5.), m(-3.5))
        );

        assert_relative_eq!(
            -ecef,
            Coordinate::<Ecef>::from_cartesian(m(-10.), m(5.), m(-3.5))
        );
    }

    #[test]
    fn test_coordinate_builder() {
        // Test successful construction
        let coord = Coordinate::<Ned>::builder()
            .x(m(100.))
            .y(m(50.))
            .z(m(-10.))
            .build();
        assert_eq!(coord.ned_north().get::<meter>(), 100.);
        assert_eq!(coord.ned_east().get::<meter>(), 50.);
        assert_eq!(coord.ned_down().get::<meter>(), -10.);

        // Test order independence
        let coord2 = Coordinate::<Ned>::builder()
            .z(m(-10.))
            .x(m(100.))
            .y(m(50.))
            .build();
        assert_relative_eq!(coord, coord2);
    }

    #[test]
    fn test_coordinate_from_components() {
        // Test successful construction
        let components = CoordinateComponents {
            x: m(100.),
            y: m(50.),
            z: m(-10.),
        };
        let coord = Coordinate::<Ned>::from_components(components);
        assert_eq!(coord.ned_north().get::<meter>(), 100.);
        assert_eq!(coord.ned_east().get::<meter>(), 50.);
        assert_eq!(coord.ned_down().get::<meter>(), -10.);
    }

    #[test]
    fn test_coordinate_constructors_equivalence() {
        let x = m(100.);
        let y = m(50.);
        let z = m(-10.);

        // All three constructors should produce the same result
        #[allow(deprecated)]
        let coord1 = Coordinate::<Ned>::from_cartesian(x, y, z);

        let coord2 = Coordinate::<Ned>::builder().x(x).y(y).z(z).build();

        let coord3 = Coordinate::<Ned>::from_components(CoordinateComponents { x, y, z });

        assert_relative_eq!(coord1, coord2);
        assert_relative_eq!(coord2, coord3);
    }
}
