# Package Catalog Submission

This repository is a normal primitive package, not a complete robot deployment.
The default branch must expose the root `package_manifest.yaml` before Catalog
submission.

The only manual change in the `syswonder/robonix-package-catalog` PR is this
entry under `packages:`:

```yaml
  - name: robonix.primitive.yahboom.rosmaster_x3.safe_move
    repo: https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move
```

The `name` must exactly match `package.name`. Description, version, license,
tags, maintainers, and capabilities must remain in `package_manifest.yaml` and
must not be duplicated in `catalog.yaml`.

## Pre-submission state

| Requirement | State |
|---|---|
| Root `package_manifest.yaml` | Published on the default branch |
| Required Catalog metadata | Present |
| Capability names and local TOML paths | Present, eight guarded capabilities |
| Executable build/start scripts | Git mode `100755` |
| Package validation/build | PASS on Jetson staging before metadata-only Catalog update |
| Default branch contains this candidate | Published; repository workflow PASS |
| Catalog PR | `syswonder/robonix-package-catalog#21`; Catalog CI PASS, merge pending |
| Hardware acceptance | Not complete; not a Catalog metadata prerequisite, but a release Warning |

Catalog CI reads the target repository default branch through the GitHub API.
The root manifest is now visible on that branch. PR #21 contains only the
required `name + repo` entry, and its Package Catalog workflow passes. Upstream
maintainer merge is still required before the package appears in the catalog.
