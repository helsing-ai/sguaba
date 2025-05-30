## Summary

- Adds type-safe constructors for `Bearing`, `Wgs84`, `Coordinate`, and `Vector` to prevent argument order confusion
- Deprecates existing positional constructors in favor of named parameter alternatives
- Maintains backwards compatibility while encouraging safer construction patterns

## Motivation

The Internet pointed out that it's easy to get the argument order mixed up for constructors like `Bearing::new` (does azimuth or elevation come first?) and `Wgs84::new` (does latitude or longitude come first?). This PR addresses this foot-gun by requiring explicit parameter naming.

## Changes

### New Construction Mechanisms

This PR introduces two new construction patterns for `Bearing`, `Wgs84`, `Coordinate`, and `Vector`:

1. **Builder Pattern**: Uses the type-state pattern to statically enforce that all fields are set
   ```rust
   Bearing::builder()
       .azimuth(angle)
       .elevation(angle)
       .build()
   ```

2. **Components Struct**: Takes a single struct with named fields
   ```rust
   Bearing::from_components(Components {
       azimuth: angle,
       elevation: angle,
   })
   ```

### Coordinate/Vector Improvements

Since `from_cartesian` takes three arguments of the same type, it has similar confusion potential. Both `Coordinate` and `Vector` now support:
- The same two constructor patterns above
- A macro constructor with single-character indicators that validates against the coordinate system:
  ```rust
  coordinate!(n = 1, e = 0, d = 5)  // Works for NED-like systems
  coordinate!(f = 1, r = 0, d = 5)  // Works for FRD-like systems
  ```

### Deprecations

The following constructors are deprecated but retained for backwards compatibility:
- `Bearing::new`
- `Wgs84::new`
- `Coordinate::from_cartesian`
- `Vector::from_cartesian`

These will likely be removed in the next major release.

## Design Decisions

- **No changes to `from_spherical` or `from_tait_bryan_angles`**: Can be addressed in follow-up PRs
- **No newtype wrappers for components**: Would add noise without significant safety benefits
- **Intrinsic rotations unchanged**: Order matters for Tait-Bryan angles, so positional args may be clearer

## Test Plan

- [ ] All existing tests pass
- [ ] New constructors have comprehensive test coverage
- [ ] Deprecation warnings appear when using old constructors
- [ ] Macro constructor properly validates coordinate system conventions