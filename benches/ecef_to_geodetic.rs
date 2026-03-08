use std::hint::black_box;

use criterion::{Criterion, criterion_group, criterion_main};
use sguaba::{
    Coordinate,
    builder::wgs84::Components,
    systems::{Ecef, Wgs84},
};
use uom::si::{
    angle::degree,
    f64::{Angle, Length},
    length::meter,
};

fn m(meters: f64) -> Length {
    Length::new::<meter>(meters)
}

fn d(degrees: f64) -> Angle {
    Angle::new::<degree>(degrees)
}

fn benchmark_conversions(c: &mut Criterion) {
    let fuji = (35.3619, 138.7280, 2294.0);
    let wgs84 = Wgs84::build(Components {
        latitude: d(fuji.0),
        longitude: d(fuji.1),
        altitude: m(fuji.2),
    })
    .unwrap();
    let ecef = Coordinate::<Ecef>::from_wgs84(&wgs84);

    let mut group = c.benchmark_group("ecef_to_geodetic");

    group.bench_function("to_wgs84", |b| {
        b.iter(|| black_box(ecef.to_wgs84()));
    });

    group.bench_function("to_wgs84_extended", |b| {
        b.iter(|| black_box(ecef.to_wgs84_extended()));
    });

    group.finish();
}

criterion_group!(benches, benchmark_conversions);
criterion_main!(benches);
