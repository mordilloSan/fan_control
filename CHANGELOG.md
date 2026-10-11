# Changelog

All notable changes to this project are documented here. Versions follow
[Semantic Versioning](https://semver.org/): patch for fixes, minor for behaviour changes such as default curves
or limits, major for interface changes (module parameters, `pwmN`/`pwmN_enable` meaning).

## [1.0.0] - 2026-10-11

First release.

- Admission checks before any EC access: exact DMI identity, EC firmware 0.5, Super-I/O and controller
  signatures, canonical CPU table and system fan policy. Refuses to load on anything else.
- `hwmon` device with both fans (`fan1` CPU, `fan2` system) and the two EC temperatures, visible in `sensors`.
- Fan control per fan through `pwmN_enable`: 1 fixed value in `pwmN` on hwmon's 0..255 scale (5 per EC code,
  100 RPM), with a 30 s watchdog back to the curve; 2 driver curve; 3 firmware.
- Temperature curves as module parameters (`cpu_curve`, `system_curve`), changeable at runtime. Defaults tuned for
  silence: CPU `0:0,25:1000,45:1400,54:1800,66:firmware`, system `0:0,25:1000,40:1600,50:firmware`.
- One code per step, a curve step is left 4 °C below its temperature, refused speeds that make the fans hunt
  (1–549 RPM CPU, 1–749 RPM system), hunting detection with back-off, and a fall back to the firmware after
  repeated failed handoffs.
- Fixed limits that hand a fan to the firmware: EC CPU 80 °C, EC system 55 °C. The CPU table's critical row is
  never written.
- Both fans returned to the firmware on unload, suspend, hibernate and shutdown; states left by an interrupted
  session are repaired at load.
- Automatic loading on the F7BSI board, DKMS packaging (Linux 6.17 or later), works with Secure Boot and kernel
  lockdown.
- Releases ship a Debian/Ubuntu DKMS package and a source tarball, built by CI against the Ubuntu 26.04
  kernel headers.
