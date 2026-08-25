# Robonix Catalog Entry

[Catalog PR #21](https://github.com/syswonder/robonix-package-catalog/pull/21)
passed its checks and was merged on 2026-08-05. The Catalog contains only this
package reference:

```yaml
  - name: robonix.primitive.yahboom.rosmaster_x3.safe_move
    repo: https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move
```

The name must match `package.name` in the root `package_manifest.yaml`. Version,
description, license, tags, maintainers, and capability paths are maintained in
that manifest rather than copied into the Catalog.

## Published state

| Item | Status |
|---|---|
| Root `package_manifest.yaml` | Present on the default branch |
| Catalog metadata and eight capability paths | Validated |
| Build and start scripts | Executable (`100755`) |
| Catalog PR #21 | Merged; Catalog checks passed |
| Earlier 0.08 m hardware run | Passed |
| Current 0.80 m hardware acceptance | Not run |

Catalog publication shows that the package metadata is accepted. It does not
replace the outstanding 0.80 m live test.
