//! Math operations that work in both std and no-std environments.
//!
//! This module provides a unified interface for mathematical operations that can use either
//! the standard library (when the `std` feature is enabled) or `libm` (in no-std environments).

#[cfg(not(any(feature = "std", feature = "libm")))]
compile_error!(
    "sguaba requires floating-point math support. \
     Enable either the `std` feature (default) or the `libm` feature for no-std environments."
);

/// Extension trait for f64 to provide math operations in a no-std compatible way
pub(crate) trait FloatMath {
    fn sin(self) -> Self;
    fn cos(self) -> Self;
    #[allow(dead_code)]
    fn tan(self) -> Self;
    fn asin(self) -> Self;
    fn acos(self) -> Self;
    fn atan(self) -> Self;
    fn atan2(self, other: Self) -> Self;
    fn sqrt(self) -> Self;
    fn powi(self, n: i32) -> Self;
    fn abs(self) -> Self;
    fn copysign(self, sign: Self) -> Self;
}

#[cfg(feature = "std")]
impl FloatMath for f64 {
    #[inline]
    fn sin(self) -> Self {
        f64::sin(self)
    }

    #[inline]
    fn cos(self) -> Self {
        f64::cos(self)
    }

    #[inline]
    fn tan(self) -> Self {
        f64::tan(self)
    }

    #[inline]
    fn asin(self) -> Self {
        f64::asin(self)
    }

    #[inline]
    fn acos(self) -> Self {
        f64::acos(self)
    }

    #[inline]
    fn atan(self) -> Self {
        f64::atan(self)
    }

    #[inline]
    fn atan2(self, other: Self) -> Self {
        f64::atan2(self, other)
    }

    #[inline]
    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }

    #[inline]
    fn powi(self, n: i32) -> Self {
        f64::powi(self, n)
    }

    #[inline]
    fn abs(self) -> Self {
        f64::abs(self)
    }

    #[inline]
    fn copysign(self, sign: Self) -> Self {
        f64::copysign(self, sign)
    }
}

#[cfg(all(not(feature = "std"), feature = "libm"))]
impl FloatMath for f64 {
    #[inline]
    fn sin(self) -> Self {
        libm::sin(self)
    }

    #[inline]
    fn cos(self) -> Self {
        libm::cos(self)
    }

    #[inline]
    fn tan(self) -> Self {
        libm::tan(self)
    }

    #[inline]
    fn asin(self) -> Self {
        libm::asin(self)
    }

    #[inline]
    fn acos(self) -> Self {
        libm::acos(self)
    }

    #[inline]
    fn atan(self) -> Self {
        libm::atan(self)
    }

    #[inline]
    fn atan2(self, other: Self) -> Self {
        libm::atan2(self, other)
    }

    #[inline]
    fn sqrt(self) -> Self {
        libm::sqrt(self)
    }

    #[inline]
    fn powi(self, n: i32) -> Self {
        libm::pow(self, n as f64)
    }

    #[inline]
    fn abs(self) -> Self {
        libm::fabs(self)
    }

    #[inline]
    fn copysign(self, sign: Self) -> Self {
        libm::copysign(self, sign)
    }
}
