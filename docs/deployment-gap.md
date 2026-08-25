# Deployment Gap

Completed: current Jetson provider/bridge/CLI snapshot and SHA256 comparison,
deployment and command mapping, offline syntax and four test suites, safety and
public-snapshot audits, prior 0.08 m hardware loop, and 0.90 m no-motion
preview/prepare validation.

Published package state:

- the Catalog-facing metadata is on the GitHub default branch and the
  repository offline workflow passes;

Still open:

- the current 0.80 m candidate has not completed long-distance live motion or
  repeated hardware acceptance;
- a 0.90 m preview/prepare dry-run is evidence of validation only, not motion;
- vendor ROS package files contain unresolved license declarations and are not
  redistributed.

The on-device backups remain outside this repository. Any later production
change requires a new hash comparison, successful fresh-AMCL/no-goal dry-run,
and separate on-site motion authorization.
