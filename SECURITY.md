# Security Policy

SWING_CAR includes software that can interact with real vehicle actuators, local networks, GNSS/NTRIP services and recorded sensor data. Treat security reports and operational data accordingly.

## Reporting a vulnerability

Do not publish exploitable vehicle-control vulnerabilities, credentials, private keys, exact private test-site locations or other sensitive operational details in a public issue.

Prefer a private GitHub Security Advisory when that feature is available for this repository. Otherwise contact the repository owner privately before disclosing sensitive details publicly.

A useful report should include the affected commit/version, affected component, reproduction conditions and expected impact. Remove real credentials, private GPS traces and unrelated personal data from logs or examples.

## Secrets and operational data

Never commit:

- NTRIP usernames/passwords or caster credentials
- Wi-Fi passwords
- API/authentication tokens
- SSH/private keys
- `.env` files containing secrets
- unsanitized RECORD sessions or field logs containing exact GNSS coordinates
- private site maps/routes or credential-bearing configuration files

Runtime secrets belong in local files or environment/configuration mechanisms that are excluded by `.gitignore`.

If a real secret is committed, removing the file in a later commit is not sufficient. Revoke/rotate the credential first, then remove it from reachable Git history and repository refs before considering the repository safe to publish.

## Vehicle safety boundary

The Raspberry Pi/Arduino safety path remains authoritative. The Windows Compute Worker and model-preview paths must not become the final motor/steering safety authority. Security changes that affect E-STOP, watchdogs, sensor-freshness checks, steering limits, output bounds or fail-closed behavior require closed-area validation on the target hardware.

## Supported code

`main` is the canonical integration branch. Historical documents under `docs/archive/` are retained for context and are not the current operational contract.
