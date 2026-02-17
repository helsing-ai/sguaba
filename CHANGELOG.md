# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Add `Orientation::from_quaternion` and `Rotation::from_quaternion`
  ([#73](https://github.com/helsing-ai/sguaba/pull/73)). 

### Changed

### Deprecated

### Removed

### Fixed

### Security

## [0.9.12]

### Fixed

- Vector accessors and constructors for types with non-zero time
  components ([#70](https://github.com/helsing-ai/sguaba/pull/70)).
- Compile failure on beta
  ([#71](https://github.com/helsing-ai/sguaba/pull/71)).

## [0.9.11]

### Changed

- Also `impl Display` for non-`&` `Wgs84`
  ([#63](https://github.com/helsing-ai/sguaba/pull/64)).

## [0.9.10]

### Changed

- Upgraded nalgebra to 0.34
  ([#48](https://github.com/helsing-ai/sguaba/pull/48)).

## [0.9.9]

### Fixed

- Fix infinite loop for certain coordinates near Earth's surface
  ([#51](https://github.com/helsing-ai/sguaba/pull/51)).

## [0.9.8]

### Changed

- Added documentation of temporal drift
  ([#45](https://github.com/helsing-ai/sguaba/pull/45)).

## [0.9.7]

### Changed

- Reduce drift in ECEF-WGS84 conversion
  ([#42](https://github.com/helsing-ai/sguaba/pull/42)).

## [0.9.6]

### Added

- Add `Vector::orientation_at_origin`
  ([#40](https://github.com/helsing-ai/sguaba/pull/40)).

## [0.9.5]

### Changed

- Uphold bearing azimuth guarantee for 90° elevation in ENU
  ([#34](https://github.com/helsing-ai/sguaba/pull/34)).

## [0.9.4]

### Added

- Argument-order-safe Tait-Bryan builder-constructor for `Orientation`
  and `Rotation` ([#28](https://github.com/helsing-ai/sguaba/pull/28)).
- Safe macros for constructing [`Bearing`] and [`Wgs84`] with
  compile-time constants
  ([#27](https://github.com/helsing-ai/sguaba/pull/27)).

### Changed

- Documented lack of normalization in elevation and latitude
  ([#26](https://github.com/helsing-ai/sguaba/pull/26)).

### Deprecated

- `Orientation::from_tait_bryan_angles` and
  `Rotation::from_tait_bryan_angles` in favour of new
  argument-order-safe builder constructors.

## [0.9.3]

### Added

- Implement `Default` for `Bearing`
  ([#17](https://github.com/helsing-ai/sguaba/pull/17)).
- Ability to turn a `Wgs84` or `Bearing` back into a builder
  ([#16](https://github.com/helsing-ai/sguaba/pull/16),
  ([#22](https://github.com/helsing-ai/sguaba/pull/22)).

### Changed

- Panic when ECEF->WGS84 conversion would produce incorrect results due
  to being outside the supported range of the underlying algorithm. Note
  that previously, this wouldn't panic, but you would instead get simply
  incorrect conversion results
  ([#24](https://github.com/helsing-ai/sguaba/pull/24)).

### Fixed

- Fixed edge case where 2π could be returned instead of 0 for very small
  negative angles
  ([#23](https://github.com/helsing-ai/sguaba/pull/23)).

## [0.9.2]

### Added

- Support for the ENU coordinate system
  ([#13](https://github.com/helsing-ai/sguaba/pull/13)).

### Changed

- Clarified deprecation notice for `from_cartesian` to note that it will
  not be removed ([#14](https://github.com/helsing-ai/sguaba/pull/14)).

## [0.9.1]

### Added

- Several constructors for `Bearing`, `Wgs84`, `Coordinate`, and
  `Vector` that mitigate the foot-gun that is argument order confusion.
  See [#8](https://github.com/helsing-ai/sguaba/pull/8) for more
  details.

### Deprecated

- `Bearing::new`, `Wgs84::new`, `Coordinate::from_cartesian`, and
  `Vector::from_cartesian` have been deprecated in favour of the newly
  added constructors.

### Removed

## [0.9.0]

Initial public release.

[unreleased]: https://github.com/helsing-ai/sguaba/compare/v0.9.12...HEAD
[0.9.12]: https://github.com/helsing-ai/sguaba/compare/v0.9.11...v0.9.12
[0.9.11]: https://github.com/helsing-ai/sguaba/compare/v0.9.10...v0.9.11
[0.9.10]: https://github.com/helsing-ai/sguaba/compare/v0.9.9...v0.9.10
[0.9.9]: https://github.com/helsing-ai/sguaba/compare/v0.9.8...v0.9.9
[0.9.8]: https://github.com/helsing-ai/sguaba/compare/v0.9.7...v0.9.8
[0.9.7]: https://github.com/helsing-ai/sguaba/compare/v0.9.6...v0.9.7
[0.9.6]: https://github.com/helsing-ai/sguaba/compare/v0.9.5...v0.9.6
[0.9.5]: https://github.com/helsing-ai/sguaba/compare/v0.9.4...v0.9.5
[0.9.4]: https://github.com/helsing-ai/sguaba/compare/v0.9.3...v0.9.4
[0.9.3]: https://github.com/helsing-ai/sguaba/compare/v0.9.2...v0.9.3
[0.9.2]: https://github.com/helsing-ai/sguaba/compare/v0.9.1...v0.9.2
[0.9.1]: https://github.com/helsing-ai/sguaba/compare/v0.9.0...v0.9.1
[0.9.0]: https://github.com/helsing-ai/sguaba/releases/tag/v0.9.0
