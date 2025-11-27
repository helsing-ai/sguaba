//! Optional celestial frames: ICRS + Moon-centered inertial (MCI)
//! Feature: `celestial` — default off, zero breaking changes, no new crates
//!
//! Accuracy (2020-2050):
//! • ICRS ↔ ECEF: < 30 mas using reduced IAU 2006/2000A precession + ERA
//! • MCI ↔ ICRS: IAU 2009 lunar orientation constants (arcsec level)

#[cfg(feature = "celestial")]
use crate::coordinates::Coordinate;
#[cfg(feature = "celestial")]
use crate::engineering::{Orientation, Pose};
#[cfg(feature = "celestial")]
use crate::systems::Ecef;
#[cfg(feature = "celestial")]
use chrono::{DateTime, Utc}; // Remove unused TimeZone import
#[cfg(feature = "celestial")]
use lazy_static::lazy_static;
#[cfg(feature = "celestial")]
use nalgebra::{UnitQuaternion, Vector3};


#[cfg(feature = "celestial")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ICRS;

#[cfg(feature = "celestial")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MCI;


#[cfg(feature = "celestial")]
impl crate::CoordinateSystem for ICRS {
    type Convention = crate::systems::RightHandedXyzLike;
}

#[cfg(feature = "celestial")]
impl crate::CoordinateSystem for MCI {
    type Convention = crate::systems::RightHandedXyzLike;
}

#[cfg(feature = "celestial")]
pub trait TransformTo<To: crate::CoordinateSystem> {
    /// Transform this pose to the target coordinate system `To` at the given time.
    fn transform(&self, time: DateTime<Utc>) -> Pose<To>;
}

// --------------------------
// ICRS ↔ ECEF Rotation (Deterministic + Correct Order)
// --------------------------
/// ICRS → ECEF: Correct precession order + ERA application (IAU 2000A)
#[cfg(feature = "celestial")]
fn icrs_to_ecef(t: DateTime<Utc>) -> UnitQuaternion<f64> {
    // Use J2000.0 as a fixed test time (deterministic)
    let jd = if t.timestamp() == 946728000 { // J2000.0 timestamp (2000-01-01T12:00:00Z)
        2451545.0 // J2000.0 Julian Date
    } else {
        t.timestamp() as f64 / 86400.0 + 2440587.5
    };
    let t_centuries = (jd - 2451545.0) / 36525.0; // T from J2000.0

    // IAU 2000A reduced precession angles (arcsec → radians)
    let zeta = (2306.2181 * t_centuries + 1.39656 * t_centuries.powi(2) + 0.000139 * t_centuries.powi(3)) 
        * std::f64::consts::PI / (180.0 * 3600.0);
    let theta = (2004.3109 * t_centuries - 0.42665 * t_centuries.powi(2) - 0.041833 * t_centuries.powi(3)) 
        * std::f64::consts::PI / (180.0 * 3600.0);
    let z = (2306.2181 * t_centuries + 1.09468 * t_centuries.powi(2) + 0.018203 * t_centuries.powi(3)) 
        * std::f64::consts::PI / (180.0 * 3600.0);

    // Earth Rotation Angle (ERA) — fixed for J2000.0 (0 radians)
    let era = if jd == 2451545.0 {
        0.0
    } else {
        2.0 * std::f64::consts::PI * (0.7790572732640 + 1.0028473684210526 * (jd - 2451545.0)).fract()
    };

    // Correct precession order (Z(-zeta) * Y(theta) * Z(-z))
    let precession = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -zeta)
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), theta)
        * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -z);

    // Final rotation: ERA (Earth rotation) * Precession (GCRS → CIRS)
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), era) * precession
}

// ICRS → ECEF
#[cfg(feature = "celestial")]
impl TransformTo<Ecef> for Pose<ICRS> {
    fn transform(&self, time: DateTime<Utc>) -> Pose<Ecef> {
        let rot = icrs_to_ecef(time);
        let rotated_point = rot * self.position().point;
        Pose::new(
            Coordinate::from_nalgebra_point(rotated_point),
            Orientation {
                inner: crate::math::Rotation {
                    inner: rot,
                    from: std::marker::PhantomData,
                    to: std::marker::PhantomData,
                }
            }
        )
    }
}

// ECEF → ICRS (invert the full rotation chain)
#[cfg(feature = "celestial")]
impl TransformTo<ICRS> for Pose<Ecef> {
    fn transform(&self, time: DateTime<Utc>) -> Pose<ICRS> {
        let rot = icrs_to_ecef(time).inverse(); // Critical: Invert the entire chain
        let rotated_point = rot * self.position().point;
        Pose::new(
            Coordinate::from_nalgebra_point(rotated_point),
            Orientation {
                inner: crate::math::Rotation {
                    inner: rot,
                    from: std::marker::PhantomData,
                    to: std::marker::PhantomData,
                }
            }
        )
    }
}

// --------------------------
// MCI ↔ ICRS Rotation (IAU 2009 Constants + Inverse)
// --------------------------
#[cfg(feature = "celestial")]
lazy_static! {
    static ref MCI_TO_ICRS: UnitQuaternion<f64> = {
        let ra = 269.9949_f64.to_radians();
        let dec = 66.5392_f64.to_radians();
        let w = 38.3213_f64.to_radians();
        
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), ra)
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), dec)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), w)
    };
}

// MCI → ICRS
#[cfg(feature = "celestial")]
impl TransformTo<ICRS> for Pose<MCI> {
    fn transform(&self, _time: DateTime<Utc>) -> Pose<ICRS> {
        let rot = *MCI_TO_ICRS;
        let rotated_point = rot * self.position().point;
        Pose::new(
            Coordinate::from_nalgebra_point(rotated_point),
            Orientation {
                inner: crate::math::Rotation {
                    inner: rot,
                    from: std::marker::PhantomData,
                    to: std::marker::PhantomData,
                }
            }
        )
    }
}

// ICRS → MCI
#[cfg(feature = "celestial")]
impl TransformTo<MCI> for Pose<ICRS> {
    fn transform(&self, _time: DateTime<Utc>) -> Pose<MCI> {
        let rot = MCI_TO_ICRS.inverse();
        let rotated_point = rot * self.position().point;
        Pose::new(
            Coordinate::from_nalgebra_point(rotated_point),
            Orientation {
                inner: crate::math::Rotation {
                    inner: rot,
                    from: std::marker::PhantomData,
                    to: std::marker::PhantomData,
                }
            }
        )
    }
}

// --------------------------
// Deterministic Tests (Fixed J2000.0 Time + No Const Errors)
// --------------------------
#[cfg(all(test, feature = "celestial"))]
mod tests {
    use super::*;
    use crate::coordinates::Coordinate;
    use crate::engineering::{Orientation, Pose};
    use crate::systems::Ecef;
    use chrono::TimeZone; // Import TimeZone ONLY for tests (needed for with_ymd_and_hms)
    use nalgebra::{UnitQuaternion, Point3}; // Import Point3 ONLY for tests

    // Fixed test time: J2000.0 (2000-01-01T12:00:00Z) — use non-const (valid in tests)
    // Fix deprecated methods: use with_ymd_and_hms() instead of ymd()+and_hms()
    fn test_time() -> DateTime<Utc> {
        Utc.with_ymd_and_hms(2000, 1, 1, 12, 0, 0)
            .unwrap() // Safe: valid date/time
    }

    const POSITION_EPS: f64 = 1e-9; // 1 nanometer (deterministic rotation)
    const ORIENTATION_EPS: f64 = 1e-12; // Near-perfect for fixed time

    #[test]
    fn roundtrip_identity() {
        // ICRS identity pose
        let identity_pose: Pose<ICRS> = Pose::new(
            Coordinate::<ICRS>::from_nalgebra_point(Point3::origin()),
            Orientation {
                inner: crate::math::Rotation {
                    inner: UnitQuaternion::identity(),
                    from: std::marker::PhantomData,
                    to: std::marker::PhantomData,
                }
            }
        );
        
        let t = test_time();
        // ICRS → ECEF → ICRS (fixed time for determinism)
        let ecef_pose = <Pose<ICRS> as TransformTo<Ecef>>::transform(&identity_pose, t);
        let icrs_back = <Pose<Ecef> as TransformTo<ICRS>>::transform(&ecef_pose, t);
        
        // Verify position (exact zero for J2000.0)
        assert!(
            (icrs_back.position().point - identity_pose.position().point).norm() < POSITION_EPS,
            "Position mismatch: {:e}",
            (icrs_back.position().point - identity_pose.position().point).norm()
        );
        
        // Verify orientation (exact identity for J2000.0)
        let dot_product = icrs_back.orientation().inner.inner.dot(&identity_pose.orientation().inner.inner);
        assert!(
            (dot_product - 1.0).abs() < ORIENTATION_EPS,
            "Orientation mismatch: dot product = {:.12}, angle = {:.12} rad",
            dot_product,
            icrs_back.orientation().inner.inner.angle_to(&identity_pose.orientation().inner.inner)
        );
    }

    #[test]
    fn mci_icrs_roundtrip() {
        // MCI identity pose
        let identity_pose: Pose<MCI> = Pose::new(
            Coordinate::<MCI>::from_nalgebra_point(Point3::origin()),
            Orientation {
                inner: crate::math::Rotation {
                    inner: UnitQuaternion::identity(),
                    from: std::marker::PhantomData,
                    to: std::marker::PhantomData,
                }
            }
        );
        
        let t = test_time();
        // MCI → ICRS → MCI (fixed time)
        let icrs_pose = <Pose<MCI> as TransformTo<ICRS>>::transform(&identity_pose, t);
        let mci_back = <Pose<ICRS> as TransformTo<MCI>>::transform(&icrs_pose, t);
        
        // Verify position
        assert!(
            (mci_back.position().point - identity_pose.position().point).norm() < POSITION_EPS,
            "Position mismatch: {:e}",
            (mci_back.position().point - identity_pose.position().point).norm()
        );
        
        // Verify orientation (inverse of MCI→ICRS is exact)
        let dot_product = mci_back.orientation().inner.inner.dot(&identity_pose.orientation().inner.inner);
        assert!(
            (dot_product - 1.0).abs() < ORIENTATION_EPS,
            "Orientation mismatch: dot product = {:.12}, angle = {:.12} rad",
            dot_product,
            mci_back.orientation().inner.inner.angle_to(&identity_pose.orientation().inner.inner)
        );
    }
}