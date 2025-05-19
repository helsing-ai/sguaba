use uom::si::angle::radian;
use uom::si::f64::Angle;

#[cfg(any(test, feature = "approx"))]
use approx::{AbsDiffEq, RelativeEq};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct BoundedAngle {
    angle: Angle,
}

impl BoundedAngle {
    pub(crate) fn new(angle: impl Into<Angle>) -> Self {
        Self {
            // NOTE(jon): even though we put the value into bounds here, uom may choose to store
            // the value differently-normalized, so we must normalize on output as well.
            angle: Angle::new::<radian>(Self::into_bounds(angle.into())),
        }
    }

    /// Returns the angle in [0°, 360°) in radians.
    pub(crate) fn get_bounded(self) -> f64 {
        Self::into_bounds(self.angle)
    }

    /// Check if the angle is in [start, stop]
    /// Based on <https://math.stackexchange.com/a/2276916>
    #[cfg(test)]
    pub(crate) fn is_in_range(&self, start: &BoundedAngle, stop: &BoundedAngle) -> bool {
        let is_between_angles = (self.angle <= stop.angle) && (start.angle <= self.angle);

        // Easy case: start is smaller than stop, then the angle needs to be between them.
        //  If start == stop the angle needs to be in there as well.
        if start.angle <= stop.angle {
            is_between_angles
            // Hard case: The stop is "around" the 360 degrees and thus smaller than the start.
            //  Now the angle must NOT be between them.
        } else {
            !is_between_angles
        }
    }

    fn into_bounds(angle: Angle) -> f64 {
        let out_of_bounds: f64 = angle.get::<radian>();
        out_of_bounds.rem_euclid(Angle::FULL_TURN.get::<radian>())
    }

    /// Returns the angle in [-180°, 180°) in radians.
    pub(crate) fn to_signed_range(self) -> f64 {
        let angle = self.get_bounded();
        if angle < Angle::HALF_TURN.get::<radian>() {
            angle
        } else {
            angle - Angle::FULL_TURN.get::<radian>()
        }
    }
}

/// Every value that can be converted into an [`Angle`] can be converted into [`BoundedAngle`].
impl<U: Into<Angle>> From<U> for BoundedAngle {
    fn from(value: U) -> Self {
        BoundedAngle::new(value)
    }
}

#[cfg(any(test, feature = "approx"))]
impl RelativeEq for BoundedAngle {
    fn default_max_relative() -> Self::Epsilon {
        f64::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        let bounded = self.get_bounded();
        let other_bounded = other.get_bounded();

        let min = f64::min(bounded, other_bounded);
        let max = f64::max(bounded, other_bounded);

        f64::relative_eq(&min, &max, epsilon, max_relative)
            || f64::relative_eq(
                &(min + Angle::FULL_TURN.get::<radian>()),
                &max,
                epsilon,
                max_relative,
            )
    }
}

#[cfg(any(test, feature = "approx"))]
impl AbsDiffEq<Self> for BoundedAngle {
    type Epsilon = <f64 as AbsDiffEq>::Epsilon;

    fn default_epsilon() -> Self::Epsilon {
        // this is very accurate in radians
        0.000_000_001
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        Self::new(self.angle - other.angle).to_signed_range().abs() <= epsilon
    }
}

#[cfg(test)]
mod tests {
    use crate::util::BoundedAngle;
    use approx::{assert_abs_diff_eq, assert_abs_diff_ne, assert_relative_eq, assert_relative_ne};
    use rstest::rstest;
    use uom::si::angle::{degree, radian};
    use uom::si::f64::Angle;

    fn r(radians: f64) -> Angle {
        Angle::new::<radian>(radians)
    }
    fn d(degrees: f64) -> Angle {
        Angle::new::<degree>(degrees)
    }

    #[test]
    fn bounded_angle_negative_radians() {
        let out_of_bounds = -(0.5 * Angle::HALF_TURN);
        let sut = BoundedAngle::new(out_of_bounds);
        assert_eq!(sut.get_bounded(), 1.5 * Angle::HALF_TURN.get::<radian>());
    }

    #[test]
    fn bounded_angle_negative_degrees() {
        let out_of_bounds = d(-390.);
        let sut = BoundedAngle::new(out_of_bounds);
        let s = sut.get_bounded();
        assert_eq!(s, 330.0_f64.to_radians());
    }

    #[test]
    fn bounded_angle_positive_within_bounds() {
        let sut = BoundedAngle::new(Angle::HALF_TURN);
        assert_eq!(sut.get_bounded(), Angle::HALF_TURN.get::<radian>());
    }

    #[test]
    fn bounded_angle_positive_outside_bounds() {
        let out_of_bounds = Angle::FULL_TURN + r(0.9);
        let sut = BoundedAngle::new(out_of_bounds);
        let s: f64 = sut.get_bounded();
        assert_relative_eq!(s, 0.9, epsilon = 0.000_000_001);
    }

    #[rstest]
    #[case(d(0.), 0.)]
    #[case(d(180.), -180.)]
    #[case(d(359.), -1.)]
    #[case(d(90.), 90.)]
    #[case(d(270.), -90.)]
    #[case(d(-90.), -90.)]
    #[case(d(-180.), -180.)]
    #[case(d(360.), 0.)]
    #[case(d(360.+120.), 120.)]
    #[case(d(360.+340.), -20.)]
    fn bounded_angle_to_signed_range_converts_correctly(
        #[case] input: Angle,
        #[case] expected_result_in_degrees: f64,
    ) {
        let bounded = BoundedAngle::new(input);

        assert_relative_eq!(
            bounded.to_signed_range(),
            expected_result_in_degrees.to_radians(),
            epsilon = f64::EPSILON * 1000.
        );
    }

    #[rstest]
    #[case(d(10.), (d(350.), d(30.)), true)]
    #[case(d(0.), (d(270.), d(360.)), true)]
    fn bounded_angle_is_in_range_works(
        #[case] input: Angle,
        #[case] range: (Angle, Angle),
        #[case] expected_result: bool,
    ) {
        let angle = BoundedAngle::new(input);

        let (start, end) = range;
        let result = angle.is_in_range(&BoundedAngle::new(start), &BoundedAngle::new(end));
        assert_eq!(result, expected_result);
    }

    #[rstest]
    #[case(0., 0. + f64::EPSILON, true)]
    #[case(0. + f64::EPSILON, 0., true)]
    #[case(10., 2., false)]
    #[case(2., 10., false)]
    #[case(360. - f64::EPSILON * 1e3, 0., true)]
    #[case(0. + f64::EPSILON * 1e2, 360. - f64::EPSILON * 1e2, true)]
    #[case(0. + f64::EPSILON * 1e2, 360. * 2. - f64::EPSILON * 1e2, true)]
    #[case(10., 2. + 360., false)]
    fn bounded_angle_comparison(#[case] a: f64, #[case] b: f64, #[case] expected: bool) {
        let a = BoundedAngle::new(d(a));
        let b = BoundedAngle::new(d(b));

        if expected {
            assert_relative_eq!(&a, &b, epsilon = f64::EPSILON * 1e3);
            assert_abs_diff_eq!(&a, &b, epsilon = f64::EPSILON * 1e3);
        } else {
            assert_relative_ne!(&a, &b, epsilon = f64::EPSILON * 1e3);
            assert_abs_diff_ne!(&a, &b, epsilon = f64::EPSILON * 1e3);
        }
    }
}
