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

[unreleased]: https://github.com/helsing-ai/sguaba/compare/v0.9.1...HEAD
[0.9.1]: https://github.com/helsing-ai/sguaba/compare/v0.9.0...v0.9.1
[0.9.0]: https://github.com/helsing-ai/sguaba/releases/tag/v0.9.0
