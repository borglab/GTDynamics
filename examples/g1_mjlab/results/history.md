# Historical aggregate comparison

Detailed historical cases and rendered reports were removed. `history.json` preserves aggregate timing/accuracy summaries, configuration, input hashes, and available source/library provenance. Historical commit IDs refer to the original measurement history before branch squashing.

Four-way medians below cover returned states; validated populations differ. Compare counts and configurations before interpreting timing changes. Earlier two-cone AL/CMC and solver-setting summaries remain in JSON with their original scopes.

| Experiment | Formulation | Validated | Returned median (ms) | Errors |
|---|---|---:|---:|---:|
| results/before-dense-report-20261029/four-way-summary.json | pyramidal | 24 | 35.14 | 0 |
| results/before-dense-report-20261029/four-way-summary.json | elliptic_norm | 9 | 92.63 | 15 |
| results/before-dense-report-20261029/four-way-summary.json | signed_square | 6 | 55.78 | 18 |
| results/before-dense-report-20261029/four-way-summary.json | friction_factor | 4 | 111.90 | 0 |
| results/dense-active-set-20261029/four-way-summary.json | pyramidal | 24 | 17.27 | 0 |
| results/dense-active-set-20261029/four-way-summary.json | elliptic_norm | 9 | 53.27 | 15 |
| results/dense-active-set-20261029/four-way-summary.json | signed_square | 6 | 34.19 | 18 |
| results/dense-active-set-20261029/four-way-summary.json | friction_factor | 4 | 100.11 | 0 |
| results/gtsam-develop-ea260ff00-20261029/four-way-summary.json | pyramidal | 24 | 30.20 | 0 |
| results/gtsam-develop-ea260ff00-20261029/four-way-summary.json | elliptic_norm | 9 | 79.19 | 15 |
| results/gtsam-develop-ea260ff00-20261029/four-way-summary.json | signed_square | 6 | 48.37 | 18 |
| results/gtsam-develop-ea260ff00-20261029/four-way-summary.json | friction_factor | 4 | 101.59 | 0 |
| pre-20261031/four-way-summary.json | pyramidal | 24 | 17.27 | 0 |
| pre-20261031/four-way-summary.json | elliptic_norm | 9 | 53.27 | 15 |
| pre-20261031/four-way-summary.json | signed_square | 6 | 34.19 | 18 |
| pre-20261031/four-way-summary.json | friction_factor | 4 | 100.11 | 0 |
| current (2026-10-31) | pyramidal | 24 | 17.63 | 0 |
| current (2026-10-31) | elliptic_norm | 9 | 52.29 | 15 |
| current (2026-10-31) | signed_square | 6 | 34.30 | 18 |
| current (2026-10-31) | friction_factor | 4 | 101.36 | 0 |
