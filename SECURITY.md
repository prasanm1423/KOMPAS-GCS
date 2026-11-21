# Security and safety policy

KOMPAS controls physical aircraft, so we treat safety bugs with the same
seriousness as security vulnerabilities.

## What to report privately

- A way to bypass the pre-flight checklist or a safety limit
- Automatic behaviour that could cut motors, fly away, or ignore a land command
- Ways for another device on the network to send commands to, or hijack, a
  drone controlled by KOMPAS
- Remote code execution or other vulnerabilities in KOMPAS or its Docker image

## How to report

Please **do not** open a public issue for these. Use GitHub's private
vulnerability reporting (the "Report a vulnerability" button on the
repository's **Security** tab).

Include what you found, steps to reproduce, and the impact you expect. We aim
to acknowledge reports within 7 days.

## Known limitations

- The drone's WiFi control protocol has no authentication or encryption. Any
  device on the drone's network can send it commands. This is a property of
  the drone's firmware, not something KOMPAS can fix.
- The Docker image's web UI (port 5800) and VNC (port 5900) have no password
  by default. Set `VNC_PASSWORD` if the host is reachable from other machines.

Ordinary bugs that don't have safety impact can go in a normal issue.
