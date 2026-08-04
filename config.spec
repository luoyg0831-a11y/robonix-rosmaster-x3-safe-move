config:
  # string, default: 127.0.0.1.
  # ROS bridge WebSocket host. It must resolve to the loopback interface or to
  # an explicitly trusted ROS host. Failure to connect is reported as a
  # read-only readiness error; no navigation goal is sent.
  # Example: rosbridge_host: 127.0.0.1
  rosbridge_host: 127.0.0.1

  # integer, TCP port, default: 9090; range: 1..65535.
  # A non-integer or out-of-range value fails readiness before any goal
  # publication.
  # Example: rosbridge_port: 9090
  rosbridge_port: 9090
