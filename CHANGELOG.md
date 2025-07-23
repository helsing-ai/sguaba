# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed

### Deprecated

### Removed

### Fixed

### Security

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

[unreleased]: https://github.com/helsing-ai/sguaba/compare/v0.9.3...HEAD
[0.9.3]: https://github.com/helsing-ai/sguaba/compare/v0.9.2...v0.9.3
[0.9.2]: https://github.com/helsing-ai/sguaba/compare/v0.9.1...v0.9.2
[0.9.1]: https://github.com/helsing-ai/sguaba/compare/v0.9.0...v0.9.1
[0.9.0]: https://github.com/helsing-ai/sguaba/releases/tag/v0.9.0
