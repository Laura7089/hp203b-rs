# Hardware Tests

A sub-crate for unit testing `hp203b` on real hardware with [`defmt-test`](https://github.com/knurling-rs/defmt) and [`probe-run`](https://github.com/knurling-rs/probe-run).
Setup for usage with [Bob]().

Created following the guidance [here](https://ferrous-systems.com/blog/test-driver-crate/#target-testing-with-defmt-test).

### Usage

1. Connect a debug probe to Bob
1. Connect the probe to the host PC
1. Power him on
1. Run `cargo test` in this repository

Use the `DEFMT_LOG` environment variable to configure the log level of the tests.
