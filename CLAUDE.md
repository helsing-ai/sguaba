# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**sguaba** is a Rust library providing type-safe spatial mathematics for coordinate system transformations. It prevents coordinate system confusion through strong typing and offers both engineering-focused and math-focused APIs for rigid body transforms.

## Core Architecture

### Primary Modules
- **`coordinate_systems.rs`**: Defines coordinate system traits and conventions (NED, FRD, ENU, ECEF, WGS84)
- **`coordinates.rs`**: Type-safe position representation with `Coordinate<In>` 
- **`vectors.rs`**: Type-safe direction/displacement with `Vector<In>`
- **`engineering.rs`**: High-level API with `Pose` and `Orientation` abstractions
- **`math.rs`**: Lower-level API with `RigidBodyTransform` and `Rotation`
- **`geodedic.rs`**: WGS84 and ECEF coordinate system implementations
- **`directions.rs`**: Bearing (azimuth/elevation) representation

### Key Type System
- **`CoordinateSystem`**: Trait defining coordinate system behavior
- **`Coordinate<In>`**: Type-safe position in coordinate system `In`
- **`Vector<In>`**: Type-safe direction in coordinate system `In`
- **`RigidBodyTransform<From, To>`**: Isometry between coordinate systems
- Phantom types prevent mixing coordinates from different systems
- `unsafe` constructors enforce explicit coordinate system relationships

## Development Commands

### Build and Test
```bash
# Standard build
cargo build

# Run all tests
cargo test

# Run tests with all features
cargo test --all-features

# Run specific test module
cargo test --test <test_name>

# Run property-based tests
cargo test --test quickcheck

# Run with coverage
cargo test --all-features --no-fail-fast
```

### Code Quality
```bash
# Format code
cargo fmt

# Check formatting
cargo fmt --check

# Run clippy lints
cargo clippy --all-features --all-targets

# Check documentation
cargo doc --all-features --no-deps
```

**IMPORTANT**: Always run `cargo fmt` before committing changes to ensure consistent code formatting across the codebase.

### Examples
```bash
# Run specific example
cargo run --example pilot-as-engineer
cargo run --example pilot-as-mathematician
cargo run --example target-aquisition-and-confirmation
```

## Testing Strategy

### Frameworks Used
- **`quickcheck`**: Property-based testing for mathematical properties
- **`rstest`**: Parameterized test cases
- **`insta`**: Snapshot testing for regression prevention
- **`approx`**: Floating-point approximate equality testing

### Test Patterns
- Round-trip testing (coordinate → transform → coordinate)
- Cross-validation with external libraries (`nav-types`)
- Edge case testing (poles, date line, coordinate system boundaries)
- Property testing for mathematical invariants

### Snapshot Tests
Located in `src/snapshots/` - these test complex outputs and prevent regressions in coordinate system transformations.

## Key Development Patterns

### Coordinate System Definition
Use the `system!` macro to define type-safe coordinate systems:
```rust
system!(struct PlaneFrd using FRD);
system!(struct PlaneNed using NED);
```

### Transform Construction
Most transforms require `unsafe` to acknowledge coordinate system relationships:
```rust
// SAFETY: wgs84 represents the location of PlaneNed's origin
let ecef_to_ned = unsafe { RigidBodyTransform::ecef_to_ned_at(&wgs84) };
```

### Builder Patterns
Use builders to prevent argument order confusion:
```rust
// Bearing builder
let bearing = Bearing::builder()
    .azimuth(angle)
    .elevation(angle)
    .build();

// Tait-Bryan angle builder (enforces yaw → pitch → roll order)
let orientation = Orientation::<PlaneNed>::tait_bryan_builder()
    .yaw(angle)
    .pitch(angle)
    .roll(angle)
    .build();

let rotation = unsafe {
    Rotation::<From, To>::tait_bryan_builder()
        .yaw(angle)
        .pitch(angle)
        .roll(angle)
        .build()
};
```

## Dependencies and Features

### Core Dependencies
- **`nalgebra`**: Linear algebra operations
- **`uom`**: Unit of measurement handling with type safety

### Optional Features
- **`serde`**: Serialization support (enabled by default)
- **`approx`**: Testing utilities for floating-point comparison (enabled by default)

## MSRV and Compatibility

- **Minimum Rust version**: 1.65.0 (required by `uom` dependency)
- **Edition**: 2021
- Cross-platform support: Linux, macOS, Windows

## Common Workflows

### Aircraft Observation to World Coordinates
1. Define coordinate systems using `system!` macro
2. Create observation using `Coordinate::from_bearing()`
3. Establish transforms with `RigidBodyTransform::ecef_to_ned_at()`
4. Chain transforms using multiplication
5. Apply transforms to convert between reference frames

### Multi-Platform Coordination
1. Define platform-specific coordinate systems
2. Use engineering API for pose/orientation management
3. Apply transforms to share target information between platforms

## Safety Considerations

- Strategic use of `unsafe` for performance while maintaining type safety
- Compile-time prevention of coordinate system mixing
- Runtime validation of angle ranges and mathematical constraints
- Clear documentation of safety requirements for transform construction

## Git commit conventions

When writing commit messages, ensure that you explain any non-obvious
trade-offs we've made in the design or implementation.

Wrap any prose (but not code) in the commit message to match git commit
conventions, including the title. Also, follow semantic commit conventions for
the commit title.

When you refer to types or very short code snippets, please place them in
backticks. Also, when you have a full line of code or more than one line of
code, put them in indented code blocks please.

Make sure that any ! symbols in the git commit message are correctly escaped.

## Writing compile_fail Tests

When adding `compile_fail` doctests to verify that certain code should not compile:

### Guidelines for compile_fail Tests

1. **Test the Right Thing**: Each `compile_fail` test should target a specific error condition:
   - Method doesn't exist on the current type state (most common for builders)
   - Type parameter constraints are violated
   - Safety requirements are not met

2. **Include Clear Comments**: Explain exactly WHY the code should fail:
   ```rust
   /// ```compile_fail
   /// // Cannot call pitch before yaw - pitch() method doesn't exist on NeedsYaw state
   /// let builder = Builder::new().pitch(angle);
   /// ```
   ```

3. **Test Each Failure Mode**: For state machines like builders, test:
   - Calling methods out of order
   - Skipping required steps
   - Calling methods that don't exist on the current state

4. **Verify Tests Actually Fail**: Before committing, create temporary example files and test with `cargo run --example <name>` to ensure they fail for the correct reasons.

### Example Testing Process

```bash
# Create temporary test file
cat > examples/test_compile_fail.rs << 'EOF'
use crate::Builder;
fn main() {
    let builder = Builder::new().invalid_method();
}
EOF

# Test that it fails as expected
cargo run --example test_compile_fail 2>&1 | grep "no method named"

# Remove test file
rm examples/test_compile_fail.rs
```

5. **Check Error Messages**: The compile error should clearly indicate what went wrong and ideally suggest the correct approach.
