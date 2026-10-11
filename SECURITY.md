# Security Policy

## Supported Versions

Only the latest release receives security fixes. Please update before reporting.

| Version | Supported          |
| ------- | ------------------ |
| 1.1.x   | :white_check_mark: |
| < 1.1   | :x:                |

## Scope

`minisforum_ec` is a kernel module: it runs in ring 0 and talks to the embedded controller through I/O ports.
Reports are welcome for anything that lets an unprivileged user, or a bad input, reach the EC or the kernel in a
way that was not intended, for example:

- memory safety bugs in the module (overflows, use after free, races);
- the module loading, or writing to the EC, on hardware or firmware it should have refused (identity checks,
  admission checks);
- module parameters or sysfs files (`cpu_curve`, `system_curve`, `pwmN`, `pwmN_enable`) accepting input that
  bypasses the fixed limits or leaves a fan stopped at a high temperature;
- the packaging, install or release workflow shipping something other than the tagged source.

Out of scope: behaviour that needs root to change module parameters or sysfs values (root can already do this),
and unsupported boards or EC firmware versions.

## Reporting a Vulnerability

Please **do not open a public issue** for a vulnerability. Use GitHub's private reporting instead:
[Report a vulnerability](https://github.com/mordilloSan/fan_control/security/advisories/new)
(Security tab → "Report a vulnerability").

Include the version (`modinfo minisforum_ec`), your kernel version, the board and EC firmware, and the steps to
reproduce.

What to expect:

- an acknowledgement within 7 days, and an update at least every 14 days until it is resolved;
- if accepted, a fix in a new release, with credit in the changelog and advisory unless you prefer otherwise;
- if declined, an explanation of why (for example, out of scope or not reproducible).

This is a one-person project maintained in spare time, so these are best-effort targets. Please allow a
reasonable period to fix the issue before disclosing it publicly.
