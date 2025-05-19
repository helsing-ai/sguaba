use crate::directions::Bearing;
use crate::Vector3;
use crate::{
    systems::{EquivalentTo, FrdLike, NedLike, RightHandedXyzLike},
    Coordinate, CoordinateSystem,
};
use std::fmt::{Display, Formatter};
use std::marker::PhantomData;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};
use std::{fmt, iter::Sum};
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

#[cfg(any(test, feature = "approx"))]
use {
    crate::Point3,
    approx::{AbsDiffEq, RelativeEq},
};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(doc)]
use crate::{math::RigidBodyTransform, systems::BearingDefined};

/// Defines a vector (ie, direction with magnitude) in the coordinate system specified by `In`.
///
/// You can construct one using [cartesian](Vector::from_cartesian) or
/// [spherical](Vector::from_spherical) components, or using [bearing +
/// range](Vector::from_bearing).
///
/// Depending on the convention of the coordinate system (eg, [`NedLike`], [`FrdLike`], or
/// [`RightHandedXyzLike`]), you'll have different appropriately-named accessors for the vector's
/// cartesian components like [`Vector::ned_north`] or [`Vector::frd_front`].
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
// don't require In: Serialize/Deserialize since we skip it anyway
#[cfg_attr(feature = "serde", serde(bound = ""))]
// no need for the "inner": indirection
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct Vector<In> {
    /// X, Y, Z in meters
    pub(crate) inner: Vector3,
    #[cfg_attr(feature = "serde", serde(skip))]
    system: PhantomData<In>,
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<In> Clone for Vector<In> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<In> Copy for Vector<In> {}

impl<In> Vector<In> {
    pub(crate) fn from_nalgebra_vector(value: Vector3) -> Self {
        Self {
            inner: value,
            system: PhantomData,
        }
    }

    /// Constructs a vector with the given (x, y, z) cartesian components in the
    /// [`CoordinateSystem`] `In`.
    ///
    /// The meaning of `x`, `y`, and `z` is dictated by the "convention" of `In`. For example, in
    /// [`NedLike`], `x` is North, `y` is East, and `z` is "down" (ie, in the direction of
    /// gravity).
    pub fn from_cartesian(
        x: impl Into<Length>,
        y: impl Into<Length>,
        z: impl Into<Length>,
    ) -> Self {
        Self::from_nalgebra_vector(Vector3::new(
            x.into().get::<meter>(),
            y.into().get::<meter>(),
            z.into().get::<meter>(),
        ))
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
    /// use njord::{system, Vector};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct Ned using NED);
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_spherical(unit, Angle::new::<degree>(0.), Angle::new::<degree>(0.)),
    ///     Vector::<Ned>::from_cartesian(zero, zero, unit),
    /// );
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_spherical(unit, Angle::new::<degree>(90.), Angle::new::<degree>(0.)),
    ///     Vector::<Ned>::from_cartesian(unit, zero, zero),
    /// );
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_spherical(unit, Angle::new::<degree>(90.), Angle::new::<degree>(90.)),
    ///     Vector::<Ned>::from_cartesian(zero, unit, zero),
    /// );
    /// ```
    ///
    /// [sph]: https://en.wikipedia.org/wiki/Spherical_coordinate_system
    pub fn from_spherical(
        radius: impl Into<Length>,
        polar: impl Into<Angle>,
        azimuth: impl Into<Angle>,
    ) -> Self {
        let radius = radius.into();
        let azimuth = azimuth.into();
        let polar = polar.into();

        let x = radius * polar.sin() * azimuth.cos();
        let y = radius * polar.sin() * azimuth.sin();
        let z = radius * polar.cos();

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
    /// use njord::{system, Bearing, Vector};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct Ned using NED);
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_bearing(Bearing::new(
    ///       Angle::new::<degree>(0.),
    ///       Angle::new::<degree>(0.),
    ///     ).expect("elevation is in-range"), unit),
    ///     Vector::<Ned>::from_cartesian(unit, zero, zero),
    /// );
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_bearing(Bearing::new(
    ///       Angle::new::<degree>(90.),
    ///       Angle::new::<degree>(0.),
    ///     ).expect("elevation is in-range"), unit),
    ///     Vector::<Ned>::from_cartesian(zero, unit, zero),
    /// );
    /// assert_relative_eq!(
    ///     Vector::<Ned>::from_bearing(Bearing::new(
    ///       Angle::new::<degree>(90.),
    ///       Angle::new::<degree>(90.),
    ///     ).expect("elevation is in-range"), unit),
    ///     Vector::<Ned>::from_cartesian(zero, zero, -unit),
    /// );
    /// ```
    ///
    /// [bearing]: https://en.wikipedia.org/wiki/Bearing_%28navigation%29
    /// [azel]: https://en.wikipedia.org/wiki/Horizontal_coordinate_system
    pub fn from_bearing(bearing: Bearing<In>, range: impl Into<Length>) -> Self
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
    /// using `<NewIn>` instead of `<In>`, just more concise and legible. That is, it is exactly
    /// equal to:
    ///
    /// ```
    /// use njord::{system, Bearing, Vector};
    /// use uom::si::f64::{Angle, Length};
    /// use uom::si::{angle::degree, length::meter};
    ///
    /// system!(struct PlaneNedFromCrate1 using NED);
    /// system!(struct PlaneNedFromCrate2 using NED);
    ///
    /// let zero = Length::new::<meter>(0.);
    /// let unit = Length::new::<meter>(1.);
    /// let vector_in_1 = Vector::<PlaneNedFromCrate1>::from_cartesian(unit, zero, unit);
    ///
    /// assert_eq!(
    ///     Vector::<PlaneNedFromCrate2>::from_cartesian(
    ///       vector_in_1.ned_north(),
    ///       vector_in_1.ned_east(),
    ///       vector_in_1.ned_down(),
    ///     ),
    ///     vector_in_1.with_same_components_in::<PlaneNedFromCrate2>()
    /// );
    /// ```
    #[must_use]
    pub fn with_same_components_in<NewIn>(self) -> Vector<NewIn> {
        Vector {
            inner: self.inner,
            system: PhantomData::<NewIn>,
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
    /// use njord::{system, systems::EquivalentTo, Vector};
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
    /// let vector_in_1 = Vector::<PlaneNedFromCrate1>::from_cartesian(unit, zero, unit);
    ///
    /// assert_eq!(
    ///     Vector::<PlaneNedFromCrate2>::from_cartesian(
    ///       vector_in_1.ned_north(),
    ///       vector_in_1.ned_east(),
    ///       vector_in_1.ned_down(),
    ///     ),
    ///     vector_in_1.cast::<PlaneNedFromCrate2>()
    /// );
    /// ```
    #[must_use]
    pub fn cast<NewIn>(self) -> Vector<NewIn>
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
        let from_origin = Coordinate::origin() + *self;
        from_origin.bearing_from_origin()
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
            system: PhantomData,
        }
    }
}

impl<In> From<Coordinate<In>> for Vector<In> {
    fn from(value: Coordinate<In>) -> Self {
        Self::from_nalgebra_vector(value.point.coords)
    }
}

impl<In> Default for Vector<In> {
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
        impl<In> Vector<In> where In: CoordinateSystem<Convention = $convention> {
            #[must_use]
            pub fn $x(&self) -> Length { Length::new::<meter>(self.inner.x) }
            #[must_use]
            pub fn $y(&self) -> Length { Length::new::<meter>(self.inner.y) }
            #[must_use]
            pub fn $z(&self) -> Length { Length::new::<meter>(self.inner.z) }

            #[must_use]
            pub fn $x_ax() -> Vector<In> {
                Self {
                    inner: *Vector3::x_axis(),
                    system: core::marker::PhantomData,
                }
            }
            #[must_use]
            pub fn $y_ax() -> Vector<In> {
                Self {
                    inner: *Vector3::y_axis(),
                    system: core::marker::PhantomData,
                }
            }
            #[must_use]
            pub fn $z_ax() -> Vector<In> {
                Self {
                    inner: *Vector3::z_axis(),
                    system: core::marker::PhantomData,
                }
            }
        }
    };
}

accessors!(RightHandedXyzLike using x, y, z + x_axis, y_axis, z_axis);
accessors!(NedLike using ned_north, ned_east, ned_down + ned_north_axis, ned_east_axis, ned_down_axis);
accessors!(FrdLike using frd_front, frd_right, frd_down + frd_front_axis, frd_right_axis, frd_down_axis);

impl<In> Vector<In> {
    /// Returns the cartesian components of this vector in XYZ order.
    ///
    /// To turn this into a simple (ie, unitless) `[f64; 3]`, use [`array::map`] combined with
    /// `.get::<meter>()`.
    #[doc(alias = "components")]
    #[must_use]
    pub fn to_cartesian(&self) -> [Length; 3] {
        [
            Length::new::<meter>(self.inner.x),
            Length::new::<meter>(self.inner.y),
            Length::new::<meter>(self.inner.z),
        ]
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

impl<In> Neg for Vector<In> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            inner: -self.inner,
            system: self.system,
        }
    }
}

impl<In> Add<Vector<In>> for Vector<In> {
    type Output = Vector<In>;

    fn add(self, rhs: Vector<In>) -> Self::Output {
        Vector {
            inner: self.inner + rhs.inner,
            system: self.system,
        }
    }
}

impl<In> AddAssign<Vector<In>> for Vector<In> {
    fn add_assign(&mut self, rhs: Vector<In>) {
        self.inner += rhs.inner;
    }
}

impl<In> Sub<Vector<In>> for Vector<In> {
    type Output = Vector<In>;

    fn sub(self, rhs: Vector<In>) -> Self::Output {
        Vector {
            inner: self.inner - rhs.inner,
            system: self.system,
        }
    }
}

impl<In> SubAssign<Vector<In>> for Vector<In> {
    fn sub_assign(&mut self, rhs: Vector<In>) {
        self.inner -= rhs.inner;
    }
}

impl<In> Sum for Vector<In> {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::zero(), |sum, v| sum + v)
    }
}

impl<In> Mul<f64> for Vector<In> {
    type Output = Self;

    fn mul(self, scalar: f64) -> Self::Output {
        Self {
            inner: self.inner * scalar,
            system: self.system,
        }
    }
}

impl<In> Div<f64> for Vector<In> {
    type Output = Self;

    fn div(self, scalar: f64) -> Self::Output {
        Self {
            inner: self.inner / scalar,
            system: self.system,
        }
    }
}

impl<In> Div<Length> for Vector<In> {
    type Output = Vector<In>;

    fn div(self, rhs: Length) -> Self::Output {
        self / rhs.get::<meter>()
    }
}

impl<In> Mul<Length> for Vector<In> {
    type Output = Vector<In>;

    fn mul(self, rhs: Length) -> Self::Output {
        self * rhs.get::<meter>()
    }
}

impl<In> PartialEq<Self> for Vector<In> {
    fn eq(&self, other: &Self) -> bool {
        self.inner.eq(&other.inner)
    }
}

impl<In> Display for Vector<In> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.inner)
    }
}

#[cfg(any(test, feature = "approx"))]
impl<In> AbsDiffEq<Self> for Vector<In> {
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
impl<In> RelativeEq for Vector<In> {
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

#[cfg(test)]
mod tests {}
