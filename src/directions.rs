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

/// A direction (conceptually represented as a unit vector) in the [`CoordinateSystem`] `In`.
///
/// A `Bearing` has an azimuth and elevation, whose meanings differ for different kinds of
/// coordinate systems (although there [are conventions][bearing]). The meaning of azimuth and
/// elevation for a given coordinate system is defined by its implementation of [`BearingDefined`].
/// In general though, azimuth tends to align with yaw and elevation with pitch (as defined by
/// Tait-Bryan angles).
///
/// See also [horizontal coordinate systems][azel].
///
/// [bearing]: https://en.wikipedia.org/wiki/Bearing_%28navigation%29
/// [azel]: https://en.wikipedia.org/wiki/Horizontal_coordinate_system
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
// don't require In: Serialize/Deserialize since we skip it anyway
#[cfg_attr(feature = "serde", serde(bound = ""))]
pub struct Bearing<In> {
    azimuth: Angle,
    elevation: Angle,

    #[cfg_attr(feature = "serde", serde(skip))]
    phantom_data: PhantomData<In>,
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<In> Clone for Bearing<In> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<In> Copy for Bearing<In> {}

/// Components for constructing a [`Bearing`].
///
/// This struct provides named fields to avoid confusion about argument order
/// when constructing a bearing.
///
/// # Example
///
/// ```
/// use sguaba::{Bearing, coordinate_systems::Ned};
/// use sguaba::directions::BearingComponents;
/// use uom::si::angle::degree;
/// use uom::si::f64::Angle;
///
/// let components = BearingComponents {
///     azimuth: Angle::new::<degree>(45.0),
///     elevation: Angle::new::<degree>(30.0),
/// };
/// let bearing = Bearing::<Ned>::from_components(components)
///     .expect("elevation is within valid range");
/// ```
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BearingComponents {
    /// The azimuthal angle.
    pub azimuth: Angle,
    /// The elevation angle. Must be in the range [-90°, 90°].
    pub elevation: Angle,
}

/// Builder for constructing a [`Bearing`] with named parameters.
///
/// This builder uses the type-state pattern to ensure both azimuth and elevation
/// are set before the bearing can be built.
///
/// # Example
///
/// ```
/// use sguaba::{Bearing, coordinate_systems::Ned};
/// use uom::si::angle::degree;
/// use uom::si::f64::Angle;
///
/// let bearing = Bearing::<Ned>::builder()
///     .azimuth(Angle::new::<degree>(45.0))
///     .elevation(Angle::new::<degree>(30.0))
///     .build()
///     .expect("elevation is within valid range");
/// ```
#[derive(Debug)]
pub struct BearingBuilder<In, AzimuthSet = (), ElevationSet = ()> {
    azimuth: Option<Angle>,
    elevation: Option<Angle>,
    _phantom: PhantomData<(In, AzimuthSet, ElevationSet)>,
}

/// Marker type indicating that azimuth has been set in the builder.
#[derive(Debug)]
pub struct AzimuthSet;

/// Marker type indicating that elevation has been set in the builder.
#[derive(Debug)]
pub struct ElevationSet;

impl<In> BearingBuilder<In, (), ()> {
    fn new() -> Self {
        Self {
            azimuth: None,
            elevation: None,
            _phantom: PhantomData,
        }
    }
}

impl<In, E> BearingBuilder<In, (), E> {
    /// Sets the azimuth angle for the bearing.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Bearing, coordinate_systems::Ned};
    /// use uom::si::angle::degree;
    /// use uom::si::f64::Angle;
    ///
    /// let builder = Bearing::<Ned>::builder()
    ///     .azimuth(Angle::new::<degree>(45.0));
    /// ```
    pub fn azimuth(mut self, azimuth: impl Into<Angle>) -> BearingBuilder<In, AzimuthSet, E> {
        self.azimuth = Some(azimuth.into());
        BearingBuilder {
            azimuth: self.azimuth,
            elevation: self.elevation,
            _phantom: PhantomData,
        }
    }
}

impl<In, A> BearingBuilder<In, A, ()> {
    /// Sets the elevation angle for the bearing.
    ///
    /// The elevation must be in the range [-90°, 90°].
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Bearing, coordinate_systems::Ned};
    /// use uom::si::angle::degree;
    /// use uom::si::f64::Angle;
    ///
    /// let builder = Bearing::<Ned>::builder()
    ///     .elevation(Angle::new::<degree>(30.0));
    /// ```
    pub fn elevation(mut self, elevation: impl Into<Angle>) -> BearingBuilder<In, A, ElevationSet> {
        self.elevation = Some(elevation.into());
        BearingBuilder {
            azimuth: self.azimuth,
            elevation: self.elevation,
            _phantom: PhantomData,
        }
    }
}

impl<In> BearingBuilder<In, AzimuthSet, ElevationSet> {
    /// Builds the [`Bearing`] from the configured values.
    ///
    /// Returns `None` if the elevation is not in the range [-90°, 90°].
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Bearing, coordinate_systems::Ned};
    /// use uom::si::angle::degree;
    /// use uom::si::f64::Angle;
    ///
    /// let bearing = Bearing::<Ned>::builder()
    ///     .azimuth(Angle::new::<degree>(45.0))
    ///     .elevation(Angle::new::<degree>(30.0))
    ///     .build()
    ///     .expect("elevation is within valid range");
    ///
    /// assert_eq!(bearing.azimuth().get::<degree>(), 45.0);
    /// assert_eq!(bearing.elevation().get::<degree>(), 30.0);
    /// ```
    #[must_use]
    pub fn build(self) -> Option<Bearing<In>> {
        Bearing::new(self.azimuth.unwrap(), self.elevation.unwrap())
    }
}

impl<In> Bearing<In> {
    /// Constructs a bearing towards the given azimuth and elevation in the [`CoordinateSystem`]
    /// `In`.
    ///
    /// The elevation must be in [-90°,90°] % 360°. If it is not, this function returns `None`.
    ///
    /// **Note**: This constructor is deprecated. Use [`Bearing::builder`] or
    /// [`Bearing::from_components`] instead to avoid confusion about argument order.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Bearing, coordinate_systems::Ned};
    /// use uom::si::angle::degree;
    /// use uom::si::f64::Angle;
    ///
    /// let bearing = Bearing::<Ned>::new(
    ///     Angle::new::<degree>(45.0),  // azimuth
    ///     Angle::new::<degree>(30.0)   // elevation
    /// ).expect("elevation is within valid range");
    /// ```
    #[must_use]
    #[deprecated(
        since = "0.10.0",
        note = "Use `Bearing::builder()` or `Bearing::from_components()` instead to avoid confusion about argument order"
    )]
    pub fn new(azimuth: impl Into<Angle>, elevation: impl Into<Angle>) -> Option<Self> {
        let elevation = elevation.into();
        let elevation_signed = BoundedAngle::new(elevation).to_signed_range();
        if !(-std::f64::consts::FRAC_PI_2..=std::f64::consts::FRAC_PI_2).contains(&elevation_signed)
        {
            None
        } else {
            Some(Self {
                azimuth: azimuth.into(),
                elevation,
                phantom_data: PhantomData,
            })
        }
    }

    /// Creates a new builder for constructing a [`Bearing`].
    ///
    /// This builder ensures that both azimuth and elevation are explicitly set
    /// before the bearing can be built, preventing confusion about argument order.
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Bearing, coordinate_systems::Ned};
    /// use uom::si::angle::degree;
    /// use uom::si::f64::Angle;
    ///
    /// let bearing = Bearing::<Ned>::builder()
    ///     .azimuth(Angle::new::<degree>(45.0))
    ///     .elevation(Angle::new::<degree>(30.0))
    ///     .build()
    ///     .expect("elevation is within valid range");
    /// ```
    pub fn builder() -> BearingBuilder<In, (), ()> {
        BearingBuilder::new()
    }

    /// Constructs a bearing from a [`BearingComponents`] struct.
    ///
    /// This constructor ensures that azimuth and elevation are explicitly named,
    /// preventing confusion about argument order.
    ///
    /// Returns `None` if the elevation is not in the range [-90°, 90°].
    ///
    /// # Example
    ///
    /// ```
    /// use sguaba::{Bearing, coordinate_systems::Ned};
    /// use sguaba::directions::BearingComponents;
    /// use uom::si::angle::degree;
    /// use uom::si::f64::Angle;
    ///
    /// let bearing = Bearing::<Ned>::from_components(BearingComponents {
    ///     azimuth: Angle::new::<degree>(45.0),
    ///     elevation: Angle::new::<degree>(30.0),
    /// }).expect("elevation is within valid range");
    ///
    /// assert_eq!(bearing.azimuth().get::<degree>(), 45.0);
    /// assert_eq!(bearing.elevation().get::<degree>(), 30.0);
    /// ```
    #[must_use]
    pub fn from_components(components: BearingComponents) -> Option<Self> {
        #[allow(deprecated)]
        Self::new(components.azimuth, components.elevation)
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

#[cfg(test)]
mod tests {
    use crate::coordinate_systems::Frd;
    use crate::coordinates::Coordinate;
    use crate::directions::{Bearing, BearingComponents};
    use crate::util::BoundedAngle;
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
        use crate::Vector;

        #[allow(deprecated)]
        let bearing = Bearing::<Frd>::new(d(azimuth), d(elevation)).unwrap();
        assert_relative_eq!(
            bearing.to_unit_vector(),
            Vector::<Frd>::from_cartesian(m(expected[0]), m(expected[1]), m(expected[2]),)
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
                phantom_data: std::marker::PhantomData,
            }
        }

        fn shrink(&self) -> Box<dyn Iterator<Item = Self>> {
            let Self {
                azimuth,
                elevation,
                phantom_data,
            } = *self;
            if azimuth.get::<uom::si::angle::radian>() == 0. {
                Box::new(
                    elevation
                        .get::<uom::si::angle::radian>()
                        .shrink()
                        .map(move |el| Self {
                            elevation: uom::si::f64::Angle::new::<uom::si::angle::radian>(el),
                            azimuth,
                            phantom_data,
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
                            phantom_data,
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
            let frd_coordinate = Coordinate::<Frd>::from_cartesian(
                m(x as f64),
                m(y as f64),
                m(z as f64),
            );
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

    #[test]
    fn test_bearing_builder() {
        // Test successful construction
        let bearing = Bearing::<Frd>::builder()
            .azimuth(d(45.))
            .elevation(d(30.))
            .build()
            .unwrap();
        assert_relative_eq!(bearing.azimuth().get::<degree>(), 45.);
        assert_relative_eq!(bearing.elevation().get::<degree>(), 30.);

        // Test elevation validation
        let invalid = Bearing::<Frd>::builder()
            .azimuth(d(45.))
            .elevation(d(100.)) // Invalid elevation
            .build();
        assert!(invalid.is_none());

        // Test order independence
        let bearing2 = Bearing::<Frd>::builder()
            .elevation(d(30.))
            .azimuth(d(45.))
            .build()
            .unwrap();
        assert_eq!(bearing.azimuth(), bearing2.azimuth());
        assert_eq!(bearing.elevation(), bearing2.elevation());
    }

    #[test]
    fn test_bearing_from_components() {
        // Test successful construction
        let components = BearingComponents {
            azimuth: d(45.),
            elevation: d(30.),
        };
        let bearing = Bearing::<Frd>::from_components(components).unwrap();
        assert_relative_eq!(bearing.azimuth().get::<degree>(), 45.);
        assert_relative_eq!(bearing.elevation().get::<degree>(), 30.);

        // Test elevation validation
        let invalid_components = BearingComponents {
            azimuth: d(45.),
            elevation: d(100.), // Invalid elevation
        };
        let invalid = Bearing::<Frd>::from_components(invalid_components);
        assert!(invalid.is_none());
    }

    #[test]
    fn test_bearing_constructors_equivalence() {
        let azimuth = d(45.);
        let elevation = d(30.);

        // All three constructors should produce the same result
        #[allow(deprecated)]
        let bearing1 = Bearing::<Frd>::new(azimuth, elevation).unwrap();

        let bearing2 = Bearing::<Frd>::builder()
            .azimuth(azimuth)
            .elevation(elevation)
            .build()
            .unwrap();

        let bearing3 =
            Bearing::<Frd>::from_components(BearingComponents { azimuth, elevation }).unwrap();

        assert_eq!(bearing1.azimuth(), bearing2.azimuth());
        assert_eq!(bearing1.elevation(), bearing2.elevation());
        assert_eq!(bearing2.azimuth(), bearing3.azimuth());
        assert_eq!(bearing2.elevation(), bearing3.elevation());
    }
}
