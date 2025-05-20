# sguaba

This library provides hard-to-misuse rigid body transforms (aka "spatial
math") for engineers with other things to worry about than linear
algebra.

## Naming

In Celtic mythology, Sguaba Tuinne ("Wave-sweeper") is the
self-navigating boat of [Manannán mac
Lir](https://en.wikipedia.org/wiki/Manann%C3%A1n_mac_Lir) that is
navigated solely by the thoughts of its pilot.

## Getting started

Prefer using [the generated docs](https://docs.rs/sguaba).

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

## Primer on coordinate systems

There are a wide variety of ways to describe the locations of objects in
space. This crate exists to help you convert between those different
ways. At the time of writing, it supports four main coordinate systems:
[WGS84] (latitude and longitude), [ECEF] ("Earth-centered,
Earth-fixed"), [NED] ("North, East, Down"), and [FRD] ("Front, Right,
Down").

[WGS84] and [ECEF] are both Earth-bound coordinate systems that describe
points in space on or near Earth. They do this by describing positions
relative to Earth's major and minor axes, often by making slightly
simplifying assumptions about the Earth's shape. WGS84 does this by
using latitude and longitude (degrees north/south of the equator and
east-west of the prime meridian), while ECEF does it by placing a
coordinate system at the center of the earth and locating [the X, Y, and
Z axes][axes] towards specific points on the Earth's surface. One can
convert between them [without too much trouble][trouble].

[NED] and [FRD] on the other hand are "local" coordinate systems that
are descriptions of relative positions to the location of the observer.
[NED] is still Earth-bound in that it describes positions in terms of
how far North, East, and Down (towards Earth's core) they are relative
to the observer. [FRD], meanwhile, is a "body frame", and just describes
positions relative to the observer's concept of Forward (eg, the
direction pointing in the same direction as the nose of a plane), Right
(eg, the direction 90º to the right when viewing along Forward), and
Down (eg, down through the belly of the plane). Converting between [FRD]
and [NED] usually requires knowing the orientation of the observer
relative to North, East, and Down, and converting between [NED] and
[ECEF] (or [WGS84]) requires also knowing the position of the observer
in Earth-bound coordinates.

[WGS84](https://en.wikipedia.org/wiki/World_Geodetic_System#WGS84)
[ECEF]: https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
[NED]: https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates
[FRD]: https://en.wikipedia.org/wiki/Body_relative_direction
[axes]: https://en.wikipedia.org/wiki/Axes_conventions
[trouble]: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#Coordinate_system_conversion

## Examples

Assume that a pilot of a plane observes something out of their window at
a given bearing and elevation angles (ie, measured in the plane's FRD)
and wants to know the location of that thing in terms of Latitude and
Longitude (eg, WGS84) coordinates.

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

From there, there are two possible paths forward, one using an API that
will appeal more to folks with an engineering background, and one that
will appeal more to a math-oriented crowd. We'll explore each in turn.

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
// which we can then turn into WGS84 lat/lon/altitude!
println!("{:?}", observation_in_ecef.to_wgs84());
```

## Versioning

As with most Rust crates, this library is versioned according to
[Semantic Versioning](https://semver.org/). [Breaking changes] will only
be made with good reason, and as infrequently as is feasible. Such
changes will generally be made in releases where the major version
number is increased (note [Cargo's caveat for pre-1.x
versions][caveat]), although [limited exceptions may occur][exceptions].
Increases in the minimum supported Rust version (MSRV) are not
considered breaking, but will result in a minor version bump.

See also [the changelog](./CHANGELOG.md) for details about changes in
recent versions.

[Breaking changes]: https://doc.rust-lang.org/cargo/reference/semver.html
[exceptions]: https://rust-lang.github.io/rfcs/1105-api-evolution.html#principles-of-the-policy
[caveat]: https://doc.rust-lang.org/cargo/reference/specifying-dependencies.html#default-requirements

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
