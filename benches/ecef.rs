use std::hint::black_box;

use criterion::{Criterion, criterion_group, criterion_main};
use sguaba::{
    Coordinate,
    builder::wgs84::Components,
    math::RigidBodyTransform,
    system,
    systems::{Ecef, Wgs84},
};
use uom::si::{
    angle::degree,
    f64::{Angle, Length},
    length::meter,
};

system!(struct PlaneNed using NED);
system!(struct PlaneEnu using ENU);

fn fuji() -> Wgs84 {
    Wgs84::build(Components {
        latitude: Angle::new::<degree>(35.3619),
        longitude: Angle::new::<degree>(138.7280),
        altitude: Length::new::<meter>(2294.0),
    })
    .unwrap()
}

fn benchmark_conversions(c: &mut Criterion) {
    let ecef = Coordinate::<Ecef>::from_wgs84(&fuji());
    let mut group = c.benchmark_group("ecef_to_geodetic");

    group.bench_function("to_wgs84", |b| {
        b.iter(|| black_box(ecef).to_wgs84());
    });

    group.bench_function("to_wgs84_extended", |b| {
        b.iter(|| black_box(ecef).to_wgs84_extended());
    });

    group.finish();
}

fn benchmark_rotations(c: &mut Criterion) {
    let fuji = fuji();
    let mut group = c.benchmark_group("ecef_to_local");

    group.bench_function("ecef_to_ned_at", |b| {
        b.iter(|| unsafe {
            RigidBodyTransform::<Ecef, PlaneNed>::ecef_to_ned_at(black_box(&fuji))
        });
    });

    group.bench_function("ecef_to_enu_at", |b| {
        b.iter(|| unsafe {
            RigidBodyTransform::<Ecef, PlaneEnu>::ecef_to_enu_at(black_box(&fuji))
        });
    });

    group.finish();
}

criterion_group!(benches, benchmark_conversions, benchmark_rotations);
criterion_main!(benches);
