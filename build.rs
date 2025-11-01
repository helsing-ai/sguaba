fn main() {
    let std_enabled = std::env::var("CARGO_FEATURE_STD").is_ok();
    let libm_enabled = std::env::var("CARGO_FEATURE_LIBM").is_ok();

    if std_enabled == libm_enabled {
        panic!(
            "Features \"std\" and \"libm\" are mutually exclusive. Enable exactly one.
The crate defaults to \"libm\"; to enable std use `--no-default-features --features std`."
        );
    }
}
