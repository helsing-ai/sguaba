//! Small wrappers for floating point math so the rest of the crate can call a
//! single function that chooses std or libm based on features.

// If the std feature is enabled, use the standard library floating point methods.
#[cfg(feature = "std")]
mod imp {
    #[inline]
    pub fn sin(x: f64) -> f64 {
        x.sin()
    }
}

// If std is not enabled, use libm's implementations
#[cfg(not(feature = "std"))]
mod imp {
    #[inline]
    pub fn sin(x: f64) -> f64 {
        libm::sin(x)
    }
}

// Public re-exports (simple names for crate use)
pub use imp::sin;

#[cfg(test)]
pub mod tests {
    #[test]
    fn sin() {
        assert_eq!(super::sin(1.2345), 0.943_983_323_944_511_1);
    }
}
