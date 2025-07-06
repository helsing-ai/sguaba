use crate::util::BoundedAngle;
use crate::Vector;
use std::fmt::{Display, Formatter};
use std::marker::PhantomData;
use uom::si::f64::{Angle, Length};
use uom::si::{angle::degree, length::meter};

#[cfg(any(feature = "approx", test))]
use approx::{AbsDiffEq, RelativeEq};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(doc)]
use crate::{
    systems::{BearingDefined, FrdLike, NedLike},
    CoordinateSystem,
};
use uom::ConstZero;

/// A direction (conceptually represented as a unit vector) in the [`CoordinateSystem`] `In`.
///
/// A `Bearing` has an azimuth and elevation, whose meanings differ for different kinds of
/// coordinate systems (although there [are conventions][bearing]). The meaning of azimuth and
/// elevation for a given coordinate system is defined by its implementation of [`BearingDefined`].
/// For example, a bearing azimuth in an FRD system is relative to the forward direction of the
/// system, while in both NED _and_ ENU (somewhat counter-intuitively) it is relative to North. In
/// general, azimuth _tends to_ align with yaw and elevation with pitch (as defined by Tait-Bryan
/// angles).
///
/// See also [horizontal coordinate systems][azel].
///
/// To construct a bearing, you can either use [a builder] via [`Bearing::builder`] or provide a
/// [`Components`] to [`Bearing::build`]. The following are equivalent:
///
/// ```rust
/// use sguaba::{Bearing, system};
/// use uom::si::f64::Angle;
/// use uom::si::angle::degree;
///
/// system!(struct PlaneFrd using FRD);
///
/// Bearing::<PlaneFrd>::builder()
///     // clockwise from forward
///     .azimuth(Angle::new::<degree>(20.))
///     // upwards from straight-ahead
///     .elevation(Angle::new::<degree>(10.))
///     .expect("elevation is in [-90º, 90º]")
///     .build();
/// ```
///
/// ```rust
/// use sguaba::{Bearing, system, builder::bearing::Components};
/// use uom::si::f64::Angle;
/// use uom::si::angle::degree;
///
/// system!(struct PlaneFrd using FRD);
///
/// Bearing::<PlaneFrd>::build(Components {
///   // clockwise from forward
///   azimuth: Angle::new::<degree>(20.),
///   // upwards from straight-ahead
///   elevation: Angle::new::<degree>(10.),
/// }).expect("elevation is in [-90º, 90º]");
/// ```
///
/// [bearing]: https://en.wikipedia.org/wiki/Bearing_%28navigation%29
/// [azel]: https://en.wikipedia.org/wiki/Horizontal_coordinate_system
/// [a builder]: https://rust-unofficial.github.io/patterns/patterns/creational/builder.html
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
// don't require In: Serialize/Deserialize since we skip it anyway
#[cfg_attr(feature = "serde", serde(bound = ""))]
pub struct Bearing<In> {
    azimuth: Angle,
    elevation: Angle,

    #[cfg_attr(feature = "serde", serde(skip))]
    system: PhantomData<In>,
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<In> Clone for Bearing<In> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<In> Copy for Bearing<In> {}

impl<In> Bearing<In> {
    /// Constructs a bearing towards the given azimuth and elevation in the [`CoordinateSystem`]
    /// `In`.
    ///
    /// The elevation must be in [-90°,90°] % 360°. If it is not, this function returns `None`.
    #[must_use]
    pub fn build(Components { azimuth, elevation }: Components) -> Option<Self> {
        Some(
            Self::builder()
                .azimuth(azimuth)
                .elevation(elevation)?
                .build(),
        )
    }

    /// Provides a constructor for a bearing in the [`CoordinateSystem`] `In`.
    pub fn builder() -> Builder<In, MissingAzimuth, MissingElevation> {
        Builder {
            under_construction: Self {
                azimuth: Angle::ZERO,
                elevation: Angle::ZERO,
                system: PhantomData,
            },
            has: (PhantomData, PhantomData),
        }
    }

    /// Constructs a bearing towards the given azimuth and elevation in the [`CoordinateSystem`]
    /// `In`.
    ///
    /// Prefer [`Bearing::build`] or [`Bearing::builder`] to avoid risk of argument order
    /// confusion. This function will be removed in a future version of Sguaba in favor of those.
    ///
    /// The elevation must be in [-90°,90°] % 360°. If it is not, this function returns `None`.
    #[must_use]
    #[deprecated = "prefer `Bearing::build` or `Bearing::builder` to avoid risk of argument order confusion"]
    pub fn new(azimuth: impl Into<Angle>, elevation: impl Into<Angle>) -> Option<Self> {
        Self::build(Components {
            azimuth: azimuth.into(),
            elevation: elevation.into(),
        })
    }

    /// Returns the azimuthal angle of this bearing.
    ///
    /// The definition of the azimuthal angle depends on the implementation of [`BearingDefined`]
    /// for `In`.
    #[must_use]
    pub fn azimuth(&self) -> Angle {
        self.azimuth
    }

    /// Returns the elevation angle of this bearing.
    ///
    /// The definition of the azimuthal angle depends on the implementation of [`BearingDefined`]
    /// for `In`.
    ///
    /// The returned value is always in [-90°, 90°].
    #[must_use]
    pub fn elevation(&self) -> Angle {
        self.elevation
    }

    /// Returns a unit vector pointing away from origin at this bearing.
    #[must_use]
    pub fn to_unit_vector(&self) -> Vector<In>
    where
        In: crate::systems::BearingDefined,
    {
        Vector::from_bearing(*self, Length::new::<meter>(1.))
    }

    /// Constructs a bearing in coordinate system `In` with azimuth and elevation set to 0.
    ///
    /// The definition of the azimuthal angle depends on the implementation of [`BearingDefined`]
    /// for `In`.
    #[must_use]
    pub fn zero() -> Self {
        Self {
            azimuth: Angle::ZERO,
            elevation: Angle::ZERO,
            system: PhantomData,
        }
    }
}

impl<In> Default for Bearing<In> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<In> Display for Bearing<In> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "bearing {:?}° at elevation {:?}°",
            self.azimuth().get::<degree>(),
            self.elevation().get::<degree>(),
        )
    }
}

impl<In> PartialEq<Self> for Bearing<In> {
    fn eq(&self, other: &Self) -> bool {
        self.elevation.eq(&other.elevation) && self.azimuth.eq(&other.azimuth)
    }
}

#[cfg(any(feature = "approx", test))]
impl<In> AbsDiffEq<Self> for Bearing<In> {
    type Epsilon = <f64 as AbsDiffEq>::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        BoundedAngle::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        BoundedAngle::abs_diff_eq(
            &BoundedAngle::new(self.azimuth()),
            &BoundedAngle::new(other.azimuth()),
            epsilon,
        ) && BoundedAngle::abs_diff_eq(
            &BoundedAngle::new(self.elevation()),
            &BoundedAngle::new(other.elevation()),
            epsilon,
        )
    }
}

#[cfg(any(feature = "approx", test))]
impl<In> RelativeEq for Bearing<In> {
    fn default_max_relative() -> Self::Epsilon {
        BoundedAngle::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        BoundedAngle::relative_eq(
            &BoundedAngle::new(self.azimuth()),
            &BoundedAngle::new(other.azimuth()),
            epsilon,
            max_relative,
        ) && BoundedAngle::relative_eq(
            &BoundedAngle::new(self.elevation()),
            &BoundedAngle::new(other.elevation()),
            epsilon,
            max_relative,
        )
    }
}

/// Argument type for [`Bearing::build`].
#[derive(Debug, Default)]
#[must_use]
pub struct Components {
    /// The azimuthal angle of the proposed [`Bearing`].
    pub azimuth: Angle,

    /// The elevation angle of the proposed [`Bearing`].
    ///
    /// The elevation must be in [-90°,90°] % 360° to form a valid [`Bearing`].
    pub elevation: Angle,
}

/// Used to indicate that a partially-constructed [`Bearing`] is missing the azimuthal component.
pub struct MissingAzimuth;
/// Used to indicate that a partially-constructed [`Bearing`] has the azimuthal component set.
pub struct HasAzimuth;
/// Used to indicate that a partially-constructed [`Bearing`] is missing the elevation component.
pub struct MissingElevation;
/// Used to indicate that a partially-constructed [`Bearing`] has the elevation component set.
pub struct HasElevation;

/// [Builder] for a [`Bearing`].
///
/// Construct one through [`Bearing::builder`], and finalize with [`Builder::build`].
///
/// [Builder]: https://rust-unofficial.github.io/patterns/patterns/creational/builder.html
#[derive(Debug)]
#[must_use]
pub struct Builder<In, Azimuth, Elevation> {
    under_construction: Bearing<In>,
    has: (PhantomData<Azimuth>, PhantomData<Elevation>),
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<In, H1, H2> Clone for Builder<In, H1, H2> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<In, H1, H2> Copy for Builder<In, H1, H2> {}

impl<In, H1, H2> Builder<In, H1, H2> {
    /// Sets the azimuthal angle of the [`Bearing`]-to-be.
    pub fn azimuth(mut self, angle: impl Into<Angle>) -> Builder<In, HasAzimuth, H2> {
        self.under_construction.azimuth = angle.into();
        Builder {
            under_construction: self.under_construction,
            has: (PhantomData::<HasAzimuth>, self.has.1),
        }
    }

    /// Sets the elevation angle of the [`Bearing`]-to-be.
    ///
    /// The elevation must be in [-90°,90°] % 360°. If it is not, this function returns `None`.
    pub fn elevation(mut self, angle: impl Into<Angle>) -> Option<Builder<In, H1, HasElevation>> {
        let elevation = angle.into();
        let elevation_signed = BoundedAngle::new(elevation).to_signed_range();
        if !(-std::f64::consts::FRAC_PI_2..=std::f64::consts::FRAC_PI_2).contains(&elevation_signed)
        {
            None
        } else {
            self.under_construction.elevation = elevation;
            Some(Builder {
                under_construction: self.under_construction,
                has: (self.has.0, PhantomData::<HasElevation>),
            })
        }
    }
}

impl<In> Builder<In, HasAzimuth, HasElevation> {
    #[must_use]
    pub fn build(self) -> Bearing<In> {
        self.under_construction
    }
}

#[cfg(test)]
mod tests {
    use crate::coordinate_systems::Frd;
    use crate::coordinates::Coordinate;
    use crate::directions::{Bearing, Components};
    use crate::util::BoundedAngle;
    use crate::{coordinate, vector};
    use approx::{assert_abs_diff_eq, assert_relative_eq};
    use quickcheck::quickcheck;
    use rstest::rstest;
    use uom::si::f64::{Angle, Length};
    use uom::si::{
        angle::{degree, radian},
        length::meter,
    };

    fn m(meters: f64) -> Length {
        Length::new::<meter>(meters)
    }
    fn d(degrees: f64) -> Angle {
        Angle::new::<degree>(degrees)
    }

    // recall that in Frd (which we're using here), positive Z is _down_
    #[rstest]
    #[case(0.0, 90.0, [0.0, 0.0, -1.0])]
    #[case(90.0, 90.0, [0.0, 0.0, -1.0])]
    #[case(180.0, 90.0, [0.0, 0.0, -1.0])]
    #[case(270.0, 90.0, [0.0, 0.0, -1.0])]
    #[case(0.0, 0.0, [1.0, 0.0, 0.0])]
    #[case(90.0, 0.0, [0.0, 1.0, 0.0])]
    #[case(180.0, 0.0, [-1.0, 0.0, 0.0])]
    #[case(270.0, 0.0, [0.0, -1.0, 0.0])]
    #[case(0.0, -90.0, [0.0, 0.0, 1.0])]
    #[case(90.0, -90.0, [0.0, 0.0, 1.0])]
    #[case(180.0, -90.0, [0.0, 0.0, 1.0])]
    #[case(270.0, -90.0, [0.0, 0.0, 1.0])]
    fn to_unit_vector(#[case] azimuth: f64, #[case] elevation: f64, #[case] expected: [f64; 3]) {
        assert_relative_eq!(
            Bearing::<Frd>::build(Components {
                azimuth: d(azimuth),
                elevation: d(elevation)
            })
            .unwrap()
            .to_unit_vector(),
            vector!(f = m(expected[0]), r = m(expected[1]), d = m(expected[2]))
        );
    }

    impl<In> quickcheck::Arbitrary for Bearing<In>
    where
        In: 'static,
    {
        fn arbitrary(g: &mut quickcheck::Gen) -> Self {
            // quickcheck will give us awkward f64 values -- we ignore those
            let azimuth = loop {
                match f64::arbitrary(g) {
                    0. => break 0.,
                    f if f.is_normal() => break f,
                    _ => {}
                }
            };
            let elevation = loop {
                match f64::arbitrary(g) {
                    0. => break 0.,
                    f if f.is_normal() => break f,
                    _ => {}
                }
            };
            Self {
                elevation: uom::si::f64::Angle::new::<uom::si::angle::radian>(
                    elevation.rem_euclid(std::f64::consts::PI) - std::f64::consts::FRAC_PI_2,
                ),
                azimuth: uom::si::f64::Angle::new::<uom::si::angle::radian>(
                    azimuth.rem_euclid(std::f64::consts::TAU),
                ),
                system: std::marker::PhantomData,
            }
        }

        fn shrink(&self) -> Box<dyn Iterator<Item = Self>> {
            let Self {
                azimuth,
                elevation,
                system: phantom_data,
            } = *self;
            if azimuth.get::<uom::si::angle::radian>() == 0. {
                Box::new(
                    elevation
                        .get::<uom::si::angle::radian>()
                        .shrink()
                        .map(move |el| Self {
                            elevation: uom::si::f64::Angle::new::<uom::si::angle::radian>(el),
                            azimuth,
                            system: phantom_data,
                        }),
                )
            } else {
                Box::new(
                    azimuth
                        .get::<uom::si::angle::radian>()
                        .shrink()
                        .map(move |az| Self {
                            elevation,
                            azimuth: uom::si::f64::Angle::new::<uom::si::angle::radian>(az),
                            system: phantom_data,
                        }),
                )
            }
        }
    }

    quickcheck! {
        fn bearing_vector_roundtrip(bearing: Bearing<Frd>) -> () {
            // azimuth won't be preserved if the bearing is along the Z axis
            let mut bearing = bearing;
            if approx::relative_eq!(bearing.elevation().get::<radian>(), std::f64::consts::FRAC_PI_2)
                || approx::relative_eq!(bearing.elevation().get::<radian>(), -std::f64::consts::FRAC_PI_2) {
                bearing.azimuth = uom::si::f64::Angle::new::<uom::si::angle::radian>(0.);
            }

            assert_relative_eq!(
                bearing,
                bearing
                  .to_unit_vector()
                  .bearing_at_origin()
                  .expect("it was a bearing, so it can be again")
            );
        }
    }

    /// Asserts that the angles are within the correct ranges
    fn _assert_correct_angles(point: (i16, i16, i16), direction: &Bearing<Frd>) {
        let positive_map = (point.0 >= 0, point.1 >= 0, point.2 >= 0);

        // Consider the z-axis pointing down. Then points with positive z-value are "below" the xy-plane.
        // These are the 8 sections spanned by the coordinate system.

        let error_checker = |(az_start, az_end, el_start, el_end): (f64, f64, f64, f64)| {
            if !(direction.elevation() >= d(el_start) && direction.elevation() <= d(el_end)) {
                if BoundedAngle::new(d(el_start) - direction.elevation()).to_signed_range()
                    < f64::EPSILON
                {
                    // boundary conditions that are also acceptable
                } else {
                    panic!(
                        "{point:?} elevation is not in [{el_start:?}, {el_end:?}] (was {:?})",
                        direction.elevation().get::<degree>()
                    );
                }
            }
            if !(direction.azimuth() >= d(az_start) && direction.azimuth() <= d(az_end)) {
                if BoundedAngle::new(d(az_start) - direction.azimuth()).to_signed_range()
                    < f64::EPSILON
                {
                    // boundary conditions that are also acceptable
                } else {
                    panic!(
                        "{point:?} azimuth is not in [{az_start:?}, {az_end:?}] (was {:?})",
                        direction.azimuth().get::<degree>()
                    );
                }
            }
        };

        let (azimuth_start, azimuth_stop, elevation_start, elevation_stop) = match positive_map {
            // x, y, z
            (true, true, true) => {
                // Point is below the xy-plane and to the top-right
                // Azimuth between 0 and 90
                // Elevation between 270 and 360  [-90, 0]
                (0., 90., -90., 0.)
            }
            (true, true, false) => {
                // Point is above the xy-plane and to the top-right
                // Azimuth between 0 and 90
                // Elevation between 0 and 90
                (0., 90., 0., 90.)
            }
            (true, false, true) => {
                // Point is below the xy-plane and to the top-left
                // Azimuth between 270 and 360
                // Elevation between 270 and 360 [-90, 0]
                (-90., 0., -90., 0.)
            }
            (true, false, false) => {
                // Point is above the xy-plane and to the top-left
                // Azimuth between 270 and 360
                // Elevation between 0 and 90
                (-90., 0., 0., 90.)
            }
            (false, true, true) => {
                // Point is below the xy-plane and to the bottom-right
                // Azimuth between 90 and 180
                // Elevation between 270 and 360 [-90, 0]
                (90., 180., -90., 0.)
            }
            (false, true, false) => {
                // Point is above the xy-plane and to the bottom-right
                // Azimuth between 90 and 180
                // Elevation between 0 and 90
                (90., 180., 0., 90.)
            }
            (false, false, true) => {
                // Point is below the xy-plane and to the bottom-left
                // Azimuth between 180 and 270
                // Elevation between 270 and 360  [-90, 0]
                (-180., -90., -90., 0.)
            }
            (false, false, false) => {
                // Point is above the xy-plane and to the bottom-left
                // Azimuth between 180 and 270
                // Elevation between 0 and 90
                (-180., -90., 0., 90.)
            }
        };

        error_checker((azimuth_start, azimuth_stop, elevation_start, elevation_stop));
    }

    // Test the cartesian -> spherical -> cartesian conversion around the origin in a [-100, 100) grid.
    quickcheck! {
        fn azimuth_elevation_range_conversion_works(x: i16, y: i16, z: i16) -> () {
            let frd_coordinate = coordinate! {
                f = m(x as f64),
                r = m(y as f64),
                d = m(z as f64);
                in Frd
            };
            let frd_direction = frd_coordinate.bearing_from_origin();

            let Some(frd_direction) = frd_direction else {
                assert!(
                    x == 0 && y == 0,
                    "only zero vectors or Z-aligned vectors have no bearing"
                );
                return;
            };

            let range = frd_coordinate.distance_from_origin();

            let frd_again = Coordinate::<Frd>::from_bearing(frd_direction, range);

            assert_relative_eq!(frd_coordinate, frd_again);

            assert_abs_diff_eq!(
                frd_direction,
                frd_again
                    .bearing_from_origin()
                    .expect("if it could be a bearing once, it can again")
            );

            _assert_correct_angles((x, y, z), &frd_direction);
        }
    }
}
