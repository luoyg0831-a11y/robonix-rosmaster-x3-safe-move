# Deployment Gap

Completed: full production 0.07 m download and diff, date-stamped verified
backup, deployment and command mapping, environment/codegen source recovery,
Jetson Python 3.10 syntax, four offline suites, and isolated
`rbnx validate/build/validate`.

Published package state:

- the Catalog-facing metadata is on the GitHub default branch and the
  repository offline workflow passes;

Still open:

- the upstream Package Catalog PR #21 passes Catalog CI and is awaiting
  maintainer merge;
- production remains on the verified 0.07 m provider;
- the real ROS no-goal dry-run stopped fail-closed at `amcl_unavailable`;
- production installation of the 0.08 m candidate is not authorized;
- no 0.08 m hardware movement or repeated acceptance has been performed;
- vendor ROS package files contain unresolved license declarations and are not
  redistributed.

The dated backup must remain intact. Any later production change requires a new
hash comparison, successful fresh-AMCL/no-goal dry-run, and separate on-site
motion authorization.
