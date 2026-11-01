# Security Policy

## Reporting a Vulnerability

Please report security issues privately. Do not open a public issue or discussion for them.

- GitHub: use [private vulnerability reporting](https://github.com/liquidraver/ZephCore/security/advisories/new)
  on this repository.
- Email: asdxed@tuta.io

**Security contact:** liquidraver (maintainer)

Include the firmware version or commit, the board and role (companion, repeater, room server, observer), and the
steps or packets needed to reproduce the issue.

## What to Expect

- Acknowledgement of your report, normally within a few days.
- A fix or mitigation is prepared privately, then released with a note in the release notes. Reporters are credited
  unless they ask not to be.
- Please allow time for a fix to reach deployed nodes before any public disclosure.

## Supported Versions

Security fixes go into the latest release and the `dev` branch. Older releases are not patched.

## Scope

In scope: the ZephCore firmware in this repository, including the mesh protocol implementation, companion
transports (BLE, USB, TCP/WiFi), the repeater CLI and OTA paths, and the storage and crypto code.

Out of scope: vulnerabilities in upstream Zephyr, its HALs or other west projects (report those to the respective
project), and weaknesses in the MeshCore protocol itself that apply equally to the upstream Arduino firmware
(report those to MeshCore).
