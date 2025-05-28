use crate::{systems::Ecef, util::BoundedAngle, Coordinate, Point3};
use std::fmt;
use std::fmt::Display;
use uom::si::f64::{Angle, Length};
use uom::si::{
    angle::{degree, radian},
    length::meter,
};

#[cfg(any(test, feature = "approx"))]
use approx::{AbsDiffEq, RelativeEq};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use std::marker::PhantomData;
use uom::ConstZero;

// Parameters required for WGS84 ellipsoid
// https://nsgreg.nga.mil/doc/view?i=4085 table 3.1
#[doc(alias = "equatorial radius")]
#[doc(alias = "a")]
pub(crate) const SEMI_MAJOR_AXIS: f64 = 6_378_137.0;
#[doc(alias = "1/f")]
const FLATTENING_FACTOR: f64 = 298.257_223_563;
#[doc(alias = "f")]
const FLATTENING: f64 = 1.0 / FLATTENING_FACTOR;
#[doc(alias = "polar radius")]
#[doc(alias = "b")]
// b/a = 1 - f
// b = a * (1 - f)
//   = a - af
const SEMI_MINOR_AXIS: f64 = SEMI_MAJOR_AXIS * (1.0 - FLATTENING);
#[doc(alias = "e^2")]
// e^2 = 1 - b^2/a^2
//     = 1 - (a - af)^2 / a^2
//     = 1 - (a^2 - 2 * a * af + a^2 f^2) / a^2
//     = 1 - (1 - 2 * f + f^2)
//     = 1 - 1 + 2 * f - f^2
//     = 2 * f - f^2
const ECCENTRICITY_SQ: f64 = 2.0 * FLATTENING - FLATTENING * FLATTENING;

/// Representing an Earth-bound location using the [World Geodedic System
/// '84](https://en.wikipedia.org/wiki/World_Geodetic_System#WGS_84).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Wgs84 {
    // NOTE(jon): note that uom does not guarantee how these angles are normalized -- they might
    // be [-180,180) or [0,360), or something else altogether. we do not normalize them ourselves
    // because callers will generally not care (they're more likely to feed the value into some
    // other formula that also doesn't care.
    pub(crate) latitude: uom::si::f64::Angle,
    pub(crate) longitude: uom::si::f64::Angle,
    altitude: uom::si::f64::Length,
}

impl Wgs84 {
    /// Constructs a world location from latitude, longitude, and altitude.
    ///
    /// The latitude must be in [-90°,90°] % 360°. If it is not, this function returns `None`.
    ///
    /// The altitude is measured as distance above the WGS84 datum reference ellipsoid.
    #[must_use]
    pub fn build(
        Components {
            latitude,
            longitude,
            altitude,
        }: Components,
    ) -> Option<Self> {
        Some(
            Self::builder()
                .latitude(latitude)?
                .longitude(longitude)
                .altitude(altitude)
                .build(),
        )
    }

    /// Provides a constructor for a [`Wgs84`] coordinate.
    pub fn builder() -> Builder<MissingLatitude, MissingLongitude, MissingAltitude> {
        Builder {
            under_construction: Wgs84 {
                latitude: Angle::ZERO,
                longitude: Angle::ZERO,
                altitude: Length::ZERO,
            },
            has: (PhantomData, PhantomData, PhantomData),
        }
    }

    /// Constructs a world location from latitude, longitude, and altitude.
    ///
    /// Prefer [`Wgs84::build`] or [`Wgs84::builder`] to avoid risk of argument order confusion.
    /// This function will be removed in a future version of Sguaba in favor of those.
    ///
    /// The latitude must be in [-90°,90°] % 360°. If it is not, this function returns `None`.
    ///
    /// The altitude is measured as distance above the WGS84 datum reference ellipsoid.
    #[must_use]
    #[deprecated = "prefer `Wgs84::build` or `Wgs84::builder` to avoid risk of argument order confusion"]
    pub fn new(
        latitude: impl Into<Angle>,
        longitude: impl Into<Angle>,
        altitude: impl Into<Length>,
    ) -> Option<Self> {
        Self::build(Components {
            latitude: latitude.into(),
            longitude: longitude.into(),
            altitude: altitude.into(),
        })
    }

    /// Computes the [great-circle distance] between the two locations on the surface of
    /// the earth.
    ///
    /// Note that this is an approximation as the earth is not a perfect sphere.
    ///
    /// The current implementation computes this [using the archaversine] (inverse haversine).
    ///
    /// [great-circle distance]: https://en.wikipedia.org/wiki/Great-circle_distance
    /// [using the archaversine]: https://en.wikipedia.org/wiki/Haversine_formula#Formulation
    #[doc(alias = "great_circle_distance")]
    #[must_use]
    pub fn haversine_distance_on_surface(&self, other: &Wgs84) -> Length {
        let haversine = central_angle_by_inverse_haversine(
            self.latitude,
            other.latitude,
            self.longitude,
            other.longitude,
        );

        haversine * Length::new::<meter>(SEMI_MAJOR_AXIS)
    }

    /// Returns the number of degrees longitude north of the equator ("northing").
    ///
    /// The returned value is always in [-90, 90).
    #[must_use]
    pub fn latitude(&self) -> Angle {
        Angle::new::<radian>(BoundedAngle::new(self.latitude).to_signed_range())
    }

    /// Returns the number of degrees longitude east of the [IERS Reference Meridian] near
    /// Greenwich ("easting").
    ///
    /// [IERS Reference Meridian]: https://en.wikipedia.org/wiki/IERS_Reference_Meridian
    #[must_use]
    pub fn longitude(&self) -> Angle {
        Angle::new::<radian>(BoundedAngle::new(self.longitude).to_signed_range())
    }

    /// Returns the distance in meters beyond the WGS84 vertical datum, ie the WGS84 ellipsoid.
    ///
    /// Note that the WGS84 ellipsoid is an approximation and does not perfectly align with ground
    /// level. Thus, while this is similar to altitude above sea/ground level, it is not equal to
    /// either of those measures.
    #[must_use]
    pub fn altitude(&self) -> Length {
        self.altitude
    }
}

impl Display for &Wgs84 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let lat = self.latitude();
        let lat_is_positive = lat.is_sign_positive();
        let lat = lat.abs().get::<degree>();
        let lon = self.longitude();
        let lon_is_positive = lon.is_sign_positive();
        let lon = lon.abs().get::<degree>();
        let alt = self.altitude.get::<uom::si::length::meter>();
        match (lat_is_positive, lon_is_positive) {
            (true, true) => write!(f, "{lat}°N, {lon}°E, {alt}m"),
            (true, false) => write!(f, "{lat}°N, {lon}°W, {alt}m"),
            (false, true) => write!(f, "{lat}°S, {lon}°E, {alt}m"),
            (false, false) => write!(f, "{lat}°S, {lon}°W, {alt}m"),
        }
    }
}

impl Coordinate<Ecef> {
    /// Converts latitude, longitude, and altitude to the Earth-Centered, Earth-Fixed coordinate
    /// system.
    ///
    /// See:
    /// <https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates>
    #[must_use]
    pub fn from_wgs84(wgs84: &Wgs84) -> Self {
        // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
        let height_h = wgs84.altitude.get::<meter>();
        let lon_lambda = wgs84.longitude.get::<radian>();
        let lat_phi = wgs84.latitude.get::<radian>();

        // https://en.wikipedia.org/wiki/Earth_radius#Prime_vertical
        let cot_2_phi = 1. / lat_phi.tan().powi(2);
        let n_phi = SEMI_MAJOR_AXIS / (1. - ECCENTRICITY_SQ / (1. + cot_2_phi)).sqrt();

        let x = (n_phi + height_h) * lat_phi.cos() * lon_lambda.cos();
        let y = (n_phi + height_h) * lat_phi.cos() * lon_lambda.sin();
        let z = ((1. - ECCENTRICITY_SQ) * n_phi + height_h) * lat_phi.sin();

        Self::from_nalgebra_point(Point3::new(x, y, z))
    }

    /// Converts an Earth-Centered, Earth-Fixed coordinate into latitude, longitude, and altitude.
    ///
    /// Note that this conversion is not trivial and needs to be approximated. This implementation
    /// currentaly uses [Ferrari's solution][ferrari], but this may change in the future.
    ///
    /// [ferrari]: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#The_application_of_Ferrari's_solution
    #[must_use]
    pub fn to_wgs84(&self) -> Wgs84 {
        let lon = self.point.y.atan2(self.point.x);

        // interestingly, there is no single way to convert from ECEF to WGS84.
        // however, there are a number of algorithms, some closed-form (non-iterative) and some
        // iterative/approximating. while it's tempting to use a closed-form version that doesn't
        // need any looping, like the original one in
        // <https://link.springer.com/article/10.1007/BF02520228>, in practice those algorithms
        // appear to have worse boundary conditions _and_ be slower, as shown in
        // <https://link.springer.com/article/10.1007/s001900050271>.
        //
        // there _is_ an iterative algorithm outlined on Wikipedia -- the Newton-Raphson method --
        // but interestingly after implementing it as presented I observed that it produces
        // slightly-wrong latitudes and very-wrong altitudes for a number of inputs. I then
        // replicating the Fukushima implementation (second paper above), but it produces
        // similarly-wrong results for points with a negative latitude. it also requires some
        // boundary post-processing to ensure it produces latitude in [-90,90] and altitudes near
        // the correct "side" of the earth (it will sometimes produces altitudes that are
        // `-ellipsoid_diameter-h`).
        //
        // I eventually found the more recent paper
        //
        //   An iterative algorithm to compute geodetic coordinates
        //   Chanfang Shu a,b,n, Fei Li
        //   https://www.sciencedirect.com/science/article/pii/S0098300410001238
        //
        // which seemed promising both computationally and accuracy-wise. and indeed, implementing
        // it gives us very fast convergence to correct values:
        let a = SEMI_MAJOR_AXIS;
        let b = SEMI_MINOR_AXIS;
        let a2 = a.powi(2);
        let b2 = b.powi(2);
        let ab = a * b;
        let z2 = self.point.z.powi(2);
        let x2y2 = self.point.x.powi(2) + self.point.y.powi(2);
        let r2 = x2y2;
        let r = x2y2.sqrt();
        let bigr2 = x2y2 + z2;

        let k0 = (((a2 * z2 + b2 * r2).sqrt() - ab) * bigr2) / (a2 * z2 + b2 * r2);
        let mut k = k0;
        loop {
            let p = a + b * k;
            let q = b + a * k;
            let f_k = 2. * (b * p * q.powi(2) + a * p.powi(2) * q - a * r2 * q - b * z2 * p);
            // NOTE(jon): dk here is the delta to the angle of the tangent _of the earth's
            // surface_, so it will get very small very quickly.
            let dk = -1. / f_k;

            if !dk.is_normal() || dk.abs() < f64::EPSILON {
                // don't propagate NaNs and stop if there's no further refinement
                break;
            }

            k += dk;
        }

        let p = a + b * k;
        let q = b + a * k;
        let lat = ((a * p * self.point.z) / (b * q * r)).atan();
        let altitude = k * ((b2 * r2 / p.powi(2)) + (a2 * z2 / q.powi(2))).sqrt();

        let wgs84 = Wgs84::builder()
            .latitude(Angle::new::<radian>(lat))
            .expect("produces lat in [-pi/2,pi/2]")
            .longitude(Angle::new::<radian>(lon))
            .altitude(Length::new::<meter>(altitude))
            .build();

        #[cfg(all(debug_assertions, any(test, feature = "approx")))]
        {
            // double check our math in tests
            let back_to_ecef = Self::from_wgs84(&wgs84);
            approx::assert_relative_eq!(self, &back_to_ecef, epsilon = Wgs84::default_epsilon());
        }

        wgs84
    }
}

impl From<Coordinate<Ecef>> for Wgs84 {
    fn from(ecef: Coordinate<Ecef>) -> Self {
        ecef.to_wgs84()
    }
}

impl From<Wgs84> for Coordinate<Ecef> {
    fn from(wgs84: Wgs84) -> Self {
        Self::from_wgs84(&wgs84)
    }
}

/// Computes the central angle between the given lat/lon points.
///
/// To turn this angle into [great-circle distance], multiply this value by the radius of the
/// sphere (ie, of the earth).
///
/// The current implementation computes this [using the archaversine] (inverse haversine).
///
/// [great-circle distance]: https://en.wikipedia.org/wiki/Great-circle_distance
/// [using the archaversine]: https://en.wikipedia.org/wiki/Haversine_formula#Formulation
pub(crate) fn central_angle_by_inverse_haversine(
    lat_a: Angle,
    lat_b: Angle,
    lon_a: Angle,
    lon_b: Angle,
) -> Angle {
    let lat_a = lat_a.get::<radian>(); // φ1
    let lat_b = lat_b.get::<radian>(); // φ2
    let lon_a = lon_a.get::<radian>(); // λ1
    let lon_b = lon_b.get::<radian>(); // λ2
    let delta_lat = lat_b - lat_a;
    let delta_lon = lon_b - lon_a;

    let inner = 1. - delta_lat.cos() + lat_a.cos() * lat_b.cos() * (1. - delta_lon.cos());
    uom::si::f64::Angle::new::<uom::si::angle::radian>(2. * (inner / 2.).sqrt().asin())
}

#[cfg(any(test, feature = "approx"))]
impl AbsDiffEq<Self> for Wgs84 {
    type Epsilon = Length;

    fn default_epsilon() -> Self::Epsilon {
        // NOTE(jon): this value is in Meters, and realistically we're fine with just-sub-meter
        // precision. we kind of have to be since the conversion from ECEF to lat/lon is inherently
        // lossy (it needs the tangent to the curvature of the earth, which challenges f64's
        // epsilon).
        Length::new::<meter>(0.75)
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.haversine_distance_on_surface(other) < epsilon
            && self.altitude.get::<uom::si::length::meter>().abs_diff_eq(
                &other.altitude.get::<uom::si::length::meter>(),
                epsilon.get::<meter>(),
            )
    }
}

#[cfg(any(test, feature = "approx"))]
impl RelativeEq for Wgs84 {
    fn default_max_relative() -> Self::Epsilon {
        Length::new::<meter>(f64::default_max_relative())
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        self.haversine_distance_on_surface(other)
            .get::<meter>()
            .abs_diff_eq(&0., epsilon.get::<meter>())
            && self.altitude.get::<meter>().relative_eq(
                &other.altitude.get::<meter>(),
                epsilon.get::<meter>(),
                max_relative.get::<meter>(),
            )
    }
}

/// Argument type for [`Wgs84::build`].
#[derive(Debug, Default)]
#[must_use]
pub struct Components {
    /// The latitude angle of the proposed [`Wgs84`] coordinate.
    ///
    /// The latitude must be in [-90°,90°] % 360°. If it is not, this function returns `None`.
    pub latitude: Angle,

    /// The longitude angle of the proposed [`Wgs84`] coordinate.
    pub longitude: Angle,

    /// The altitude of the proposed [`Wgs84`] coordinate.
    ///
    /// The altitude is measured as distance above the WGS84 datum reference ellipsoid.
    pub altitude: Length,
}

/// Used to indicate that a partially-constructed [`Wgs84`] is missing the latitude component.
pub struct MissingLatitude;
/// Used to indicate that a partially-constructed [`Wgs84`] has the latitude component set.
pub struct HasLatitude;
/// Used to indicate that a partially-constructed [`Wgs84`] is missing the longitude component.
pub struct MissingLongitude;
/// Used to indicate that a partially-constructed [`Wgs84`] has the longitude component set.
pub struct HasLongitude;
/// Used to indicate that a partially-constructed [`Wgs84`] is missing the altitude component.
pub struct MissingAltitude;
/// Used to indicate that a partially-constructed [`Wgs84`] has the altitude component set.
pub struct HasAltitude;

/// [Builder] for a [`Wgs84`] coordinate.
///
/// Construct one through [`Wgs84::builder`], and finalize with [`Builder::build`].
///
/// [Builder]: https://rust-unofficial.github.io/patterns/patterns/creational/builder.html
#[derive(Debug)]
#[must_use]
pub struct Builder<Latitude, Longitude, Altitude> {
    under_construction: Wgs84,
    has: (
        PhantomData<Latitude>,
        PhantomData<Longitude>,
        PhantomData<Altitude>,
    ),
}

// manual impls of Clone and Copy to avoid requiring In: Copy + Clone
impl<L1, L2, A> Clone for Builder<L1, L2, A> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<L1, L2, A> Copy for Builder<L1, L2, A> {}

impl<L1, L2, A> Builder<L1, L2, A> {
    /// Sets the latitudal angle of the [`Wgs84`]-to-be.
    ///
    /// The latitude must be in [-90°,90°] % 360°. If it is not, this function returns `None`.
    pub fn latitude(mut self, latitude: impl Into<Angle>) -> Option<Builder<HasLatitude, L2, A>> {
        let latitude = latitude.into();
        let latitude_in_signed_radians = BoundedAngle::new(latitude).to_signed_range();
        if !(-std::f64::consts::FRAC_PI_2..=std::f64::consts::FRAC_PI_2)
            .contains(&latitude_in_signed_radians)
        {
            None
        } else {
            self.under_construction.latitude = latitude;
            Some(Builder {
                under_construction: self.under_construction,
                has: (PhantomData::<HasLatitude>, self.has.1, self.has.2),
            })
        }
    }

    /// Sets the longitudal angle of the [`Wgs84`]-to-be.
    pub fn longitude(mut self, longitude: impl Into<Angle>) -> Builder<L1, HasLongitude, A> {
        self.under_construction.longitude = longitude.into();
        Builder {
            under_construction: self.under_construction,
            has: (self.has.0, PhantomData::<HasLongitude>, self.has.2),
        }
    }

    /// Sets the altitude of the [`Wgs84`]-to-be.
    ///
    /// The altitude is measured as distance above the WGS84 datum reference ellipsoid.
    pub fn altitude(mut self, altitude: impl Into<Length>) -> Builder<L1, L2, HasAltitude> {
        self.under_construction.altitude = altitude.into();
        Builder {
            under_construction: self.under_construction,
            has: (self.has.0, self.has.1, PhantomData::<HasAltitude>),
        }
    }
}

impl Builder<HasLatitude, HasLongitude, HasAltitude> {
    #[must_use]
    pub fn build(self) -> Wgs84 {
        self.under_construction
    }
}

#[cfg(test)]
mod tests {
    use super::Wgs84;
    use crate::coordinate;
    use crate::coordinate_systems::Ecef;
    use crate::coordinates::Coordinate;
    use crate::geodedic::Components;
    use crate::util::BoundedAngle;
    use approx::{assert_relative_eq, AbsDiffEq};
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

    impl quickcheck::Arbitrary for Wgs84 {
        fn arbitrary(g: &mut quickcheck::Gen) -> Self {
            // quickcheck will give us awkward f64 values -- we ignore those
            let latitude = loop {
                match f64::arbitrary(g) {
                    0. => break 0.,
                    f if f.is_normal() => break f,
                    _ => {}
                }
            };
            let longitude = loop {
                match f64::arbitrary(g) {
                    0. => break 0.,
                    f if f.is_normal() => break f,
                    _ => {}
                }
            };
            let altitude = loop {
                match f64::arbitrary(g) {
                    0. => break 0.,
                    f if f.is_normal() => break f,
                    _ => {}
                }
            };
            Self {
                latitude: Angle::new::<radian>(
                    latitude.rem_euclid(std::f64::consts::PI) - std::f64::consts::FRAC_PI_2,
                ),
                longitude: Angle::new::<radian>(longitude.rem_euclid(std::f64::consts::TAU)),
                altitude: Length::new::<meter>(altitude.rem_euclid(50000.) - 10000.),
            }
        }

        fn shrink(&self) -> Box<dyn Iterator<Item = Self>> {
            let Self {
                latitude,
                longitude,
                altitude,
            } = *self;
            if altitude.get::<meter>() == 0. {
                if longitude.get::<radian>() == 0. {
                    Box::new(latitude.get::<radian>().shrink().map(move |lat| Self {
                        latitude: Angle::new::<radian>(lat),
                        longitude,
                        altitude,
                    }))
                } else {
                    Box::new(
                        longitude
                            .get::<uom::si::angle::radian>()
                            .shrink()
                            .map(move |lon| Self {
                                latitude,
                                longitude: Angle::new::<radian>(lon),
                                altitude,
                            }),
                    )
                }
            } else {
                Box::new(altitude.get::<meter>().shrink().map(move |alt| Self {
                    latitude,
                    longitude,
                    altitude: Length::new::<meter>(alt),
                }))
            }
        }
    }

    #[rstest]
    #[case(d(90.9948211), d(7.8211606), m(1000.))]
    #[case(d(190.112282), d(19.880389), m(0.))]
    fn wgs_fails_with_bad_lat(
        #[case] latitude: Angle,
        #[case] longitude: Angle,
        #[case] altitude: Length,
    ) {
        assert_eq!(
            Wgs84::build(Components {
                latitude,
                longitude,
                altitude
            }),
            None,
            "WGS84 position with lat in (90,-90) should be bad"
        );
    }

    #[test]
    fn wgs_display() {
        for (lat, lon, alt) in [
            (0., 0., 0.),
            // Mt. Fuji
            (35.3619, 138.7280, 2294.0),
            (-35.3619, 138.7280, 2294.0),
            (35.3619, -138.7280, 2294.0),
            (-35.3619, -138.7280, 2294.0),
        ] {
            insta::assert_snapshot!(Wgs84::build(Components {
                latitude: d(lat),
                longitude: d(lon),
                altitude: m(alt)
            })
            .unwrap());
        }
    }

    fn try_wgs_ecef_roundtrip(wgs84: Wgs84) {
        let ecef = Coordinate::<Ecef>::from_wgs84(&wgs84);

        let lat = wgs84.latitude;
        let lon = wgs84.longitude;
        let alt = wgs84.altitude;

        // normalize lat/lon so nav_types doesn't get sad
        let lat = BoundedAngle::new(lat).to_signed_range().to_degrees();
        let lon = BoundedAngle::new(lon).to_signed_range().to_degrees();

        let location = nav_types::WGS84::from_degrees_and_meters(lat, lon, alt.get::<meter>());
        let location_ecef = nav_types::ECEF::from(location);

        let expected_ecef = Coordinate::<Ecef>::from(&location_ecef);

        // we use the WGS84 epsilon even for ECEF here since we have to allow for the loss of
        // precision when going from ECEF to lat/lon.
        assert_relative_eq!(ecef, expected_ecef, epsilon = Wgs84::default_epsilon());

        let wgs_84_result = Wgs84::from(ecef);
        assert_relative_eq!(wgs_84_result, wgs84);

        // also double-check that rotations of 360° are fine
        for rot in [-720., -360., 360., 720.] {
            let wgs84_rot = Wgs84::build(Components {
                latitude: d(lat + rot),
                longitude: d(lon),
                altitude: alt,
            })
            .unwrap();
            let ecef_rot = Coordinate::<Ecef>::from_wgs84(&wgs84_rot);
            assert_relative_eq!(ecef, ecef_rot, epsilon = Wgs84::default_epsilon());

            let wgs84_rot = Wgs84::build(Components {
                latitude: d(lat),
                longitude: d(lon + rot),
                altitude: alt,
            })
            .unwrap();
            let ecef_rot = Coordinate::<Ecef>::from_wgs84(&wgs84_rot);
            assert_relative_eq!(ecef, ecef_rot, epsilon = Wgs84::default_epsilon());
        }
    }

    quickcheck! {
        fn wgs_ecef_roundtrip(wgs84: Wgs84) -> () {
            try_wgs_ecef_roundtrip(wgs84);
        }
    }

    // also, stress test known problematic things
    #[rstest]
    #[case(d(0.), d(0.), m(1000.))]
    #[case(d(90.), d(0.), m(1000.))]
    #[case(d(-90.), d(0.), m(1000.))]
    #[case(d(90.), d(90.), m(1000.))]
    #[case(d(90.), d(180.), m(1000.))]
    #[case(d(90.), d(-90.), m(1000.))]
    #[case(d(-90.), d(90.), m(1000.))]
    #[case(d(-90.), d(180.), m(1000.))]
    #[case(d(-90.), d(-90.), m(1000.))]
    #[case(d(89.999999), d(0.), m(1000.))]
    #[case(d(-89.999999), d(0.), m(1000.))]
    #[case(d(89.999999), d(180.), m(1000.))]
    #[case(d(-89.999999), d(180.), m(1000.))]
    #[case(d(89.999999), d(-179.99999), m(1000.))]
    #[case(d(-89.999999), d(-179.99999), m(1000.))]
    fn hard_wgs_to_ecef(#[case] lat: Angle, #[case] long: Angle, #[case] alt: Length) {
        try_wgs_ecef_roundtrip(
            Wgs84::build(Components {
                latitude: lat,
                longitude: long,
                altitude: alt,
            })
            .expect("lat in [-90,90]"),
        );
    }

    #[test]
    fn known_wgs_to_ecef() {
        for (wgs, ecef) in [
            ((0., 0., 0.), (6378137., 0., 0.)),
            (
                // Mt. Fuji
                (35.3619, 138.7280, 2294.0),
                (-3915138.118709466, 3436144.354064903, 3672011.028417511),
            ),
            (
                (-27.270950, 19.880389, 3000.),
                (5337604.33, 1930119.71, -2906308.35),
            ),
        ] {
            let (lat, lon, alt) = wgs;
            let (x, y, z) = ecef;
            let wgs84 = Wgs84::build(Components {
                latitude: d(lat),
                longitude: d(lon),
                altitude: m(alt),
            })
            .unwrap();
            let ecef = Coordinate::<Ecef>::from_wgs84(&wgs84);
            assert_relative_eq!(ecef, coordinate!(x = m(x), y = m(y), z = m(z)),);
        }
    }
}
