# Remaining Work

Completed:

- provider, bridge, and CLI copied from the Jetson and compared by SHA256;
- local syntax checks, four unit-test suites, and repository checks;
- Jetson Python 3.10.20 syntax check and deployed CLI unit test;
- one earlier 0.08 m live run;
- three preview/prepare checks on an earlier 0.90 m configuration, without
  sending a goal;
- Catalog publication through PR #21.

Still required before the current 0.80 m configuration can be treated as
hardware-tested:

- a fresh no-motion run with AMCL, costmaps, `make_plan`, and `move_base` online;
- a supervised live run with independent distance and stop monitoring;
- repeat runs after reviewing localization drift and chassis behavior.

Yahboom ROS files and site maps are not included because their redistribution
status has not been confirmed.
