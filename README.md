# Njord

This library provides hard-to-misuse rigid body transforms (aka "spatial
math") for engineers with better things to think about than linear
algebra.

## Naming

> In Norse mythology,
> [Njörðr](https://en.wikipedia.org/wiki/Nj%C3%B6r%C3%B0r) [...] is
> associated with the sea, seafaring, wind, fishing, wealth, and crop
> fertility.

## Getting started

Prefer using [the generated docs](https://docs.rs/njord).

First and foremost, the library provides `Coordinate` and `Vector` types
for representing points and vectors in coordinate spaces respectively.
They are all generic over a `CoordinateSystem` so that coordinates from
one system cannot (easily) be incorrectly misused as though they were in
a different one. The `system!` macro allows you to define additional
coordinate systems with particular semantics (eg, `NedLike` or
`FrdLike`) such that you can distinguish between coordinates in, say,
`PlaneFrd` and `EmitterFrd`.

To move between coordinate systems, you'll want to use the mathematical
constructs from the `math` submodule like rigid body transforms and
rotations.

Now _technically_, those are all you need to express anything in
coordinate system math, including poses and orientation. It turns out
that everything is an isometry if you think hard enough about it. But if
your brain is more used to thinking about orientation and poses (ie,
position + orientation), you'll want to make use of the `engineering`
module which has easier-to-grasp types like `Pose` and `Orientation`.

## Examples

Assume that a pilot of a plane observes something out of their window at
a given bearing (ie, in the plane's FRD) and wants to communicate the
real-world location of that thing (ie, in WGS84).

```rust
// FRD and NED systems are "local" coordinate systems, meaning a given
// coordinate in the FRD of one plane will have a completely different
// coordinate if it were to be expressed in the FRD of another. so, to
// guard against accidentally getting them mixed up, we construct a new
// type for this plane's FRD and NED:

// the pilot observes things in FRD of the plane
system!(struct PlaneFrd using FRD);

// the pilot's instruments indicate the plane's orientation in NED
system!(struct PlaneNed using NED);

// what the pilot saw:
let observation = Coordinate::<PlaneFrd>::from_bearing(
    Bearing::new(
      Degrees::<f64>::new(20.), // clockwise from forward
      Degrees::<f64>::new(10.), // upwards from straight-ahead
    ).expect("elevation is in [-90, 90]"),
    Meters::<f64>::new(400.), // at this range
);

// where the plane was at the time (eg, from GPS):
let wgs84 = Wgs84::new(
    Degrees::<f64>::new(12.),
    Degrees::<f64>::new(30.),
    Meters::<f64>::new(1000.)
).expect("latitude is in [-90, 90]");

// where the plane was facing at the time (eg, from instrument panel);
// expressed in yaw, pitch, roll relative to North-East-Down:
let orientation_in_ned = Orientation::<PlaneNed>::from_tait_bryan_angles(
    Degrees::<f64>::new(8.),  // yaw
    Degrees::<f64>::new(45.), // pitch
    Degrees::<f64>::new(0.),  // roll
);
```

### Using the engineering-focused API

In the engineering API, we can directly talk about an object's
orientation and its "pose" in the world (ie, position + orientation).
using these, we can transform between different coordinate systems to go
from `PlaneFrd` to `PlaneNed` to `Ecef` (cartesian world location) to
WGS84.

It's worth noting here that it's not possible to go directly from NED to ECEF!

```rust
// to convert between NED and ECEF, we need a transform between the two.
// this transform depends on where on the globe you are, so it takes the WGS84 position:
let ecef_to_plane_ned = RigidBodyTransform::ecef_to_ned_at(&wgs84);

// to convert between FRD (which the observation was made in) and NED,
// we just need the plane's orientation, which we have from the instruments!
let plane_ned_to_plane_frd = orientation_in_ned.map_as_zero_in::<PlaneFrd>();

// these transformations can be chained to go from ECEF to NED.
// this chaining would fail to compile if you got the arguments wrong!
let ecef_to_plane_frd = ecef_to_plane_ned.and_then(plane_ned_to_plane_frd);

// this transform lets you go from ECEF to FRD, but transforms work both ways,
// so we can apply it in inverse to take our `Coordinate<PlaneFrd>` and produce
// a `Coordinate<Ecef>`:
let observation_in_ecef = ecef_to_plane_frd.inverse_transform(observation);

// we can then turn that into WGS84 lat/lon/altitude!
println!("{:?}", observation_in_ecef.to_wgs84());
```

## Using the math-focused API

In the math API, everything is represented in terms of transforms
between coordinate systems and the components of those transforms.


```rust
// we need to find the ECEF<>NED transform for the plane's location
let ecef_to_plane_ned = RigidBodyTransform::ecef_to_ned_at(&wgs84);
// the plane's orientation in NED is really a rotation and translation in ECEF
let pose_in_ecef = ecef_to_plane_ned * orientation_in_ned;
// that rotation and translation is exactly equal to the FRD of the plane
// we could also have just constructed this rotation directly instead of an `Orientation`
let ecef_to_frd = pose_in_ecef.map_as_zero_in::<PlaneFrd>();
// and we can apply that transform to the original observation to get it in ECEF
let observation_in_ecef: Coordinate<Ecef> = ecef_to_frd * observation;
```

## License

Licensed under either of

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
