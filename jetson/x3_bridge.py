import time
import math
import threading
import roslibpy


class X3Bridge:
    def __init__(self, host='127.0.0.1', port=9090):
        self.ros = roslibpy.Ros(host=host, port=port)
        self.cmd = None
        self._navigation_goal_topic = None
        self._navigation_cancel_topic = None
        self._navigation_goal_lock = threading.Lock()

    def connect(self):
        self.ros.run()
        if not self.ros.is_connected:
            raise RuntimeError('Failed to connect rosbridge')

    def enable_cmd_vel(self):
        raise RuntimeError(
            'direct /cmd_vel publishing is disabled; use move_base navigation'
        )

    def read_once(self, topic_name, msg_type, timeout=8):
        topic = roslibpy.Topic(self.ros, topic_name, msg_type)
        received = {}

        def callback(msg):
            received['msg'] = msg
            topic.unsubscribe()

        topic.subscribe(callback)

        deadline = time.time() + timeout
        while time.time() < deadline and 'msg' not in received:
            time.sleep(0.1)

        if 'msg' not in received:
            topic.unsubscribe()
            raise TimeoutError('No message received from ' + topic_name)

        return received['msg']

    def get_odom(self):
        msg = self.read_once('/odom', 'nav_msgs/Odometry')
        pose = msg['pose']['pose']
        twist = msg['twist']['twist']
        return {
            'x': pose['position']['x'],
            'y': pose['position']['y'],
            'orientation_z': pose['orientation']['z'],
            'orientation_w': pose['orientation']['w'],
            'linear_x': twist['linear']['x'],
            'linear_y': twist['linear']['y'],
            'angular_z': twist['angular']['z'],
        }

    def get_scan_summary(self):
        msg = self.read_once('/scan', 'sensor_msgs/LaserScan')
        ranges = msg['ranges']
        valid = [
            r for r in ranges
            if isinstance(r, (int, float)) and math.isfinite(r)
        ]
        return {
            'frame_id': msg['header']['frame_id'],
            'range_min': msg['range_min'],
            'range_max': msg['range_max'],
            'ranges_length': len(ranges),
            'valid_ranges': len(valid),
            'first_10_ranges': ranges[:10],
        }

    @staticmethod
    def _stamp_seconds(header):
        stamp = (header or {}).get('stamp', {})
        return (
            float(stamp.get('secs', 0) or 0)
            + float(stamp.get('nsecs', 0) or 0) / 1e9
        )

    def _amcl_pose_result(self, msg):
        pose = msg['pose']['pose']
        covariance = list(msg['pose'].get('covariance', []))
        stamp_sec = self._stamp_seconds(msg.get('header', {}))
        received_epoch = time.time()
        position_variance_max = None
        yaw_variance = None
        if len(covariance) >= 36:
            position_variance_max = max(
                float(covariance[0]),
                float(covariance[7]),
            )
            yaw_variance = float(covariance[35])
        return {
            'frame_id': msg['header']['frame_id'],
            'x': pose['position']['x'],
            'y': pose['position']['y'],
            'orientation_z': pose['orientation']['z'],
            'orientation_w': pose['orientation']['w'],
            'raw_pose': pose,
            'stamp_sec': stamp_sec,
            'received_epoch': received_epoch,
            'message_age_sec': max(0.0, received_epoch - stamp_sec),
            'covariance': covariance,
            'position_variance_max': position_variance_max,
            'yaw_variance': yaw_variance,
        }

    def get_amcl_pose(self):
        msg = self.read_once(
            '/amcl_pose',
            'geometry_msgs/PoseWithCovarianceStamped'
        )
        return self._amcl_pose_result(msg)

    def get_fresh_amcl_pose(self, timeout=5.0):
        """Request a no-motion AMCL update and return only the new sample."""
        timeout = float(timeout)
        if not 1.0 <= timeout <= 10.0:
            raise ValueError('fresh AMCL timeout is out of range')

        topic = roslibpy.Topic(
            self.ros,
            '/amcl_pose',
            'geometry_msgs/PoseWithCovarianceStamped',
            queue_length=10,
        )
        service = roslibpy.Service(
            self.ros,
            '/request_nomotion_update',
            'std_srvs/Empty',
        )
        received = {}
        ready = threading.Event()
        requested_epoch = time.time()

        def callback(msg):
            stamp_sec = self._stamp_seconds(msg.get('header', {}))
            if stamp_sec < requested_epoch - 0.5:
                return
            received['msg'] = msg
            ready.set()

        topic.subscribe(callback)
        try:
            service.call(
                roslibpy.ServiceRequest({}),
                timeout=min(3.0, timeout),
            )
            if not ready.wait(timeout):
                raise TimeoutError('No fresh AMCL pose after no-motion update')
            result = self._amcl_pose_result(received['msg'])
            result['freshness_verified'] = True
            result['nomotion_update_requested_epoch'] = requested_epoch
            return result
        finally:
            topic.unsubscribe()

    def get_move_base_status(self):
        msg = self.read_once(
            '/move_base/status',
            'actionlib_msgs/GoalStatusArray'
        )
        status_list = msg.get('status_list', [])
        if not status_list:
            return {
                'has_status': False,
                'status': None,
                'status_name': None,
                'text': 'no active or recent navigation goal',
                'goal_id': None,
                'active': False,
            }

        last = status_list[-1]
        status = last.get('status')
        return {
            'has_status': True,
            'status': status,
            'status_name': self._goal_status_name(status),
            'text': last.get('text'),
            'goal_id': last.get('goal_id', {}).get('id'),
            'active': status in (0, 1, 6, 7),
        }


    def get_costmap_status(self, timeout=8.0):
        """Read and summarize move_base global and local costmaps."""

        def summarize(topic_name):
            msg = self.read_once(
                topic_name,
                'nav_msgs/OccupancyGrid',
                timeout=timeout,
            )

            header = msg.get('header', {})
            info = msg.get('info', {})
            data = msg.get('data', [])

            width = int(info.get('width', 0) or 0)
            height = int(info.get('height', 0) or 0)
            resolution = float(
                info.get('resolution', 0.0) or 0.0
            )

            expected_cells = width * height
            data_length = len(data)

            unknown_cells = 0
            free_cells = 0
            occupied_ge100 = 0
            high_ge80 = 0
            nonzero_cells = 0

            for value in data:
                value = int(value)

                if value < 0:
                    unknown_cells += 1
                elif value == 0:
                    free_cells += 1
                else:
                    nonzero_cells += 1

                    if value >= 80:
                        high_ge80 += 1

                    if value >= 100:
                        occupied_ge100 += 1

            size_match = (
                expected_cells > 0
                and data_length == expected_cells
            )

            valid = (
                bool(header.get('frame_id'))
                and width > 0
                and height > 0
                and resolution > 0.0
                and size_match
            )

            origin = info.get('origin', {}).get(
                'position',
                {},
            )

            return {
                'ok': valid,
                'topic': topic_name,
                'frame_id': header.get('frame_id'),
                'resolution': resolution,
                'width': width,
                'height': height,
                'width_m': width * resolution,
                'height_m': height * resolution,
                'origin_x': origin.get('x'),
                'origin_y': origin.get('y'),
                'expected_cells': expected_cells,
                'data_length': data_length,
                'size_match': size_match,
                'unknown_cells': unknown_cells,
                'free_cells': free_cells,
                'nonzero_cells': nonzero_cells,
                'high_ge80': high_ge80,
                'occupied_ge100': occupied_ge100,
            }

        global_costmap = summarize(
            '/move_base/global_costmap/costmap'
        )

        local_costmap = summarize(
            '/move_base/local_costmap/costmap'
        )

        return {
            'ok': (
                global_costmap.get('ok') is True
                and local_costmap.get('ok') is True
            ),
            'global_costmap': global_costmap,
            'local_costmap': local_costmap,
            'read_only': True,
            'navigation_goal_sent': False,
            'cmd_vel_published': False,
        }


    def get_global_costmap_grid(self, timeout=8.0):
        """Read the complete global OccupancyGrid without controlling motion."""
        topic_name = '/move_base/global_costmap/costmap'

        msg = self.read_once(
            topic_name,
            'nav_msgs/OccupancyGrid',
            timeout=timeout,
        )

        header = msg.get('header', {})
        info = msg.get('info', {})
        data = msg.get('data', [])

        width = int(info.get('width', 0) or 0)
        height = int(info.get('height', 0) or 0)
        resolution = float(
            info.get('resolution', 0.0) or 0.0
        )

        expected_cells = width * height
        data_length = len(data)
        size_match = (
            expected_cells > 0
            and data_length == expected_cells
        )

        origin = info.get('origin', {})
        origin_position = origin.get('position', {})
        origin_orientation = origin.get('orientation', {})

        frame_id = header.get('frame_id')
        valid = (
            bool(frame_id)
            and width > 0
            and height > 0
            and resolution > 0.0
            and size_match
        )

        return {
            'ok': valid,
            'topic': topic_name,
            'frame_id': frame_id,
            'resolution': resolution,
            'width': width,
            'height': height,
            'origin_x': float(
                origin_position.get('x', 0.0) or 0.0
            ),
            'origin_y': float(
                origin_position.get('y', 0.0) or 0.0
            ),
            'origin_orientation': origin_orientation,
            'expected_cells': expected_cells,
            'data_length': data_length,
            'size_match': size_match,
            'data': data,
            'read_only': True,
            'navigation_goal_sent': False,
            'cmd_vel_published': False,
        }

    def get_tf_chain_status(
        self,
        fixed_frame='map',
        target_frame='base_footprint',
        timeout=4.0,
    ):
        """Read /tf and /tf_static and verify a frame chain."""
        topics = []
        edges = {}
        message_counts = {
            '/tf': 0,
            '/tf_static': 0,
        }

        def clean_frame(frame):
            return str(frame or '').lstrip('/')

        def make_callback(topic_name):
            def callback(message):
                message_counts[topic_name] += 1

                for transform in message.get('transforms', []):
                    parent = clean_frame(
                        transform.get(
                            'header',
                            {},
                        ).get('frame_id')
                    )
                    child = clean_frame(
                        transform.get('child_frame_id')
                    )

                    if parent and child:
                        edges[child] = {
                            'parent': parent,
                            'translation': transform.get(
                                'transform',
                                {},
                            ).get('translation', {}),
                            'rotation': transform.get(
                                'transform',
                                {},
                            ).get('rotation', {}),
                            'source_topic': topic_name,
                        }

            return callback

        def find_chain():
            fixed = clean_frame(fixed_frame)
            current = clean_frame(target_frame)

            chain = [current]
            visited = set()

            while current != fixed:
                if current in visited:
                    return {
                        'ok': False,
                        'reason': 'TF cycle detected',
                        'chain': chain,
                    }

                visited.add(current)

                item = edges.get(current)
                if not item:
                    return {
                        'ok': False,
                        'reason': (
                            'missing parent for frame: '
                            + current
                        ),
                        'chain': chain,
                    }

                current = item['parent']
                chain.append(current)

            chain.reverse()

            return {
                'ok': True,
                'chain': chain,
            }

        try:
            for topic_name in ['/tf', '/tf_static']:
                topic = roslibpy.Topic(
                    self.ros,
                    topic_name,
                    'tf2_msgs/TFMessage',
                )
                topic.subscribe(
                    make_callback(topic_name)
                )
                topics.append(topic)

            deadline = time.time() + timeout

            while time.time() < deadline:
                time.sleep(0.1)

        finally:
            for topic in topics:
                try:
                    topic.unsubscribe()
                except Exception:
                    pass

        chain_result = find_chain()

        related_frames = {}
        for frame in [
            'odom',
            'base_footprint',
            'base_link',
            'laser',
            'laser_link',
        ]:
            if frame in edges:
                related_frames[frame] = edges[frame]

        return {
            'fixed_frame': clean_frame(fixed_frame),
            'target_frame': clean_frame(target_frame),
            'message_counts': message_counts,
            'frame_count': len(edges),
            'related_frames': related_frames,
            'chain_ok': chain_result.get('ok') is True,
            'chain': chain_result.get('chain', []),
            'reason': chain_result.get('reason'),
            'read_only': True,
            'navigation_goal_sent': False,
            'cmd_vel_published': False,
        }


    def get_make_plan_status(
        self,
        test_distance=0.05,
        tolerance=0.05,
        timeout=8.0,
    ):
        """Call move_base make_plan without sending a navigation goal."""
        service_name = '/move_base/make_plan'
        expected_type = 'nav_msgs/GetPlan'

        services = self.ros.get_services()
        service_present = service_name in services

        result = {
            'service_name': service_name,
            'service_present': service_present,
            'expected_type': expected_type,
            'navigation_goal_sent': False,
            'cmd_vel_published': False,
        }

        if not service_present:
            result.update({
                'call_ok': False,
                'path_available': False,
                'reason': 'make_plan service not found',
            })
            return result

        actual_type = self.ros.get_service_type(service_name)
        type_match = actual_type == expected_type

        result['actual_type'] = actual_type
        result['type_match'] = type_match

        if not type_match:
            result.update({
                'call_ok': False,
                'path_available': False,
                'reason': 'unexpected make_plan service type',
            })
            return result

        amcl = self.get_amcl_pose()
        pose = amcl['raw_pose']

        x = float(pose['position']['x'])
        y = float(pose['position']['y'])
        z = float(pose['orientation']['z'])
        w = float(pose['orientation']['w'])

        yaw = 2.0 * math.atan2(z, w)

        goal_x = x + test_distance * math.cos(yaw)
        goal_y = y + test_distance * math.sin(yaw)

        start_pose = {
            'header': {
                'frame_id': 'map',
                'stamp': {'secs': 0, 'nsecs': 0},
            },
            'pose': pose,
        }

        goal_pose = {
            'header': {
                'frame_id': 'map',
                'stamp': {'secs': 0, 'nsecs': 0},
            },
            'pose': {
                'position': {
                    'x': goal_x,
                    'y': goal_y,
                    'z': 0.0,
                },
                'orientation': pose['orientation'],
            },
        }

        service = roslibpy.Service(
            self.ros,
            service_name,
            expected_type,
        )

        request = roslibpy.ServiceRequest({
            'start': start_pose,
            'goal': goal_pose,
            'tolerance': tolerance,
        })

        response = service.call(request, timeout=timeout)

        plan = response.get('plan', {})
        poses = plan.get('poses', [])

        result.update({
            'call_ok': True,
            'test_distance_m': test_distance,
            'tolerance_m': tolerance,
            'start_x': x,
            'start_y': y,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'plan_frame_id': plan.get(
                'header',
                {}
            ).get('frame_id', ''),
            'plan_pose_count': len(poses),
            'path_available': len(poses) > 0,
        })

        return result


    def make_plan_to_pose(
        self,
        goal_x,
        goal_y,
        goal_yaw_deg,
        tolerance=0.05,
        timeout=8.0,
    ):
        """Request a path to an arbitrary map pose without sending a goal."""
        service_name = '/move_base/make_plan'
        expected_type = 'nav_msgs/GetPlan'

        goal_x = float(goal_x)
        goal_y = float(goal_y)
        goal_yaw_deg = float(goal_yaw_deg)
        tolerance = float(tolerance)

        if not all(
            math.isfinite(value)
            for value in (
                goal_x,
                goal_y,
                goal_yaw_deg,
                tolerance,
            )
        ):
            raise ValueError('make_plan parameters must be finite')

        if tolerance < 0.0:
            raise ValueError('tolerance must be non-negative')

        result = {
            'service_name': service_name,
            'expected_type': expected_type,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw_deg': goal_yaw_deg,
            'tolerance_m': tolerance,
            'read_only': True,
            'navigation_goal_sent': False,
            'cmd_vel_published': False,
        }

        services = self.ros.get_services()
        service_present = service_name in services
        result['service_present'] = service_present

        if not service_present:
            result.update({
                'call_ok': False,
                'path_available': False,
                'plan_pose_count': 0,
                'reason': 'make_plan service not found',
            })
            return result

        actual_type = self.ros.get_service_type(service_name)
        type_match = actual_type == expected_type

        result['actual_type'] = actual_type
        result['type_match'] = type_match

        if not type_match:
            result.update({
                'call_ok': False,
                'path_available': False,
                'plan_pose_count': 0,
                'reason': 'unexpected make_plan service type',
            })
            return result

        amcl = self.get_amcl_pose()
        start_raw_pose = amcl['raw_pose']

        start_x = float(start_raw_pose['position']['x'])
        start_y = float(start_raw_pose['position']['y'])

        yaw = math.radians(goal_yaw_deg)
        goal_orientation = {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0),
        }

        start_pose = {
            'header': {
                'frame_id': 'map',
                'stamp': {'secs': 0, 'nsecs': 0},
            },
            'pose': start_raw_pose,
        }

        goal_pose = {
            'header': {
                'frame_id': 'map',
                'stamp': {'secs': 0, 'nsecs': 0},
            },
            'pose': {
                'position': {
                    'x': goal_x,
                    'y': goal_y,
                    'z': 0.0,
                },
                'orientation': goal_orientation,
            },
        }

        service = roslibpy.Service(
            self.ros,
            service_name,
            expected_type,
        )

        request = roslibpy.ServiceRequest({
            'start': start_pose,
            'goal': goal_pose,
            'tolerance': tolerance,
        })

        response = service.call(request, timeout=timeout)

        plan = response.get('plan', {})
        poses = plan.get('poses', [])

        result.update({
            'call_ok': True,
            'start_x': start_x,
            'start_y': start_y,
            'plan_frame_id': plan.get(
                'header',
                {},
            ).get('frame_id', ''),
            'plan_pose_count': len(poses),
            'path_available': len(poses) > 0,
        })

        if poses:
            last_pose = poses[-1].get('pose', {}).get(
                'position',
                {},
            )
            result['plan_last_x'] = last_pose.get('x')
            result['plan_last_y'] = last_pose.get('y')

        return result

    def get_cmd_vel_state(
        self,
        timeout=1.5,
        linear_threshold=0.02,
        angular_threshold=0.05,
    ):
        """Read one /cmd_vel message without publishing anything."""
        try:
            msg = self.read_once(
                '/cmd_vel',
                'geometry_msgs/Twist',
                timeout=timeout
            )
        except TimeoutError:
            return {
                'received': False,
                'idle': True,
                'command_active': False,
                'reason': 'no /cmd_vel message observed during timeout',
                'timeout_sec': timeout,
                'linear_threshold': linear_threshold,
                'angular_threshold': angular_threshold,
            }

        linear = msg.get('linear', {})
        angular = msg.get('angular', {})

        linear_x = float(linear.get('x', 0.0) or 0.0)
        linear_y = float(linear.get('y', 0.0) or 0.0)
        linear_z = float(linear.get('z', 0.0) or 0.0)

        angular_x = float(angular.get('x', 0.0) or 0.0)
        angular_y = float(angular.get('y', 0.0) or 0.0)
        angular_z = float(angular.get('z', 0.0) or 0.0)

        linear_peak = max(
            abs(linear_x),
            abs(linear_y),
            abs(linear_z),
        )
        angular_peak = max(
            abs(angular_x),
            abs(angular_y),
            abs(angular_z),
        )

        idle = (
            linear_peak <= linear_threshold
            and angular_peak <= angular_threshold
        )

        return {
            'received': True,
            'idle': idle,
            'command_active': not idle,
            'linear': {
                'x': linear_x,
                'y': linear_y,
                'z': linear_z,
            },
            'angular': {
                'x': angular_x,
                'y': angular_y,
                'z': angular_z,
            },
            'linear_peak': linear_peak,
            'angular_peak': angular_peak,
            'linear_threshold': linear_threshold,
            'angular_threshold': angular_threshold,
        }


    @staticmethod
    def _goal_status_name(status):
        return {
            0: 'PENDING',
            1: 'ACTIVE',
            2: 'PREEMPTED',
            3: 'SUCCEEDED',
            4: 'ABORTED',
            5: 'REJECTED',
            6: 'PREEMPTING',
            7: 'RECALLING',
            8: 'RECALLED',
            9: 'LOST',
        }.get(status, 'UNKNOWN')

    def _ensure_navigation_goal_publisher(self):
        created = False
        if self._navigation_goal_topic is None:
            topic = roslibpy.Topic(
                self.ros,
                '/move_base_simple/goal',
                'geometry_msgs/PoseStamped',
                queue_size=1,
                reconnect_on_close=True,
            )
            topic.advertise()
            self._navigation_goal_topic = topic
            created = True
            # advertise() is queued asynchronously through rosbridge.
            time.sleep(0.25)
        return self._navigation_goal_topic, created

    def _ensure_navigation_cancel_publisher(self):
        if self._navigation_cancel_topic is None:
            topic = roslibpy.Topic(
                self.ros,
                '/move_base/cancel',
                'actionlib_msgs/GoalID',
                queue_size=1,
                reconnect_on_close=True,
            )
            topic.advertise()
            self._navigation_cancel_topic = topic
            time.sleep(0.25)
        return self._navigation_cancel_topic

    def _cancel_navigation_goal(self, goal_id):
        if not goal_id:
            raise ValueError('a correlated goal ID is required for cancellation')
        topic = self._ensure_navigation_cancel_publisher()
        topic.publish(roslibpy.Message({
            'stamp': {'secs': 0, 'nsecs': 0},
            'id': str(goal_id),
        }))

    @staticmethod
    def _pose_yaw(pose):
        orientation = pose.get('orientation', {})
        z = float(orientation.get('z', 0.0) or 0.0)
        w = float(orientation.get('w', 1.0) or 1.0)
        return 2.0 * math.atan2(z, w)

    @staticmethod
    def _angle_delta(current, previous):
        return ((current - previous + math.pi) % (2.0 * math.pi)) - math.pi

    @staticmethod
    def _safe_unsubscribe(topic, errors):
        try:
            topic.unsubscribe()
        except Exception as exc:
            errors.append(type(exc).__name__)

    def send_navigation_goal_pose(
        self,
        frame_id,
        x,
        y,
        yaw_deg,
        acceptance_timeout=5.0,
        result_timeout=15.0,
        monitor_ready_timeout=3.0,
        max_linear_mps=0.10,
        max_angular_rps=0.60,
        max_odom_path_m=0.10,
        max_amcl_displacement_m=0.10,
        cancel_timeout=4.0,
    ):
        """Publish one goal and correlate move_base acceptance and terminal status.

        The method publishes only to /move_base_simple/goal. It subscribes to
        status, cmd_vel, and AMCL for evidence; it never publishes raw velocity.
        """
        frame_id = str(frame_id)
        x = float(x)
        y = float(y)
        yaw_deg = float(yaw_deg)
        acceptance_timeout = float(acceptance_timeout)
        result_timeout = float(result_timeout)
        monitor_ready_timeout = float(monitor_ready_timeout)
        max_linear_mps = float(max_linear_mps)
        max_angular_rps = float(max_angular_rps)
        max_odom_path_m = float(max_odom_path_m)
        max_amcl_displacement_m = float(max_amcl_displacement_m)
        cancel_timeout = float(cancel_timeout)

        if frame_id != 'map':
            raise ValueError('navigation goal frame must be map')
        if not all(math.isfinite(value) for value in (
            x,
            y,
            yaw_deg,
            acceptance_timeout,
            result_timeout,
            monitor_ready_timeout,
            max_linear_mps,
            max_angular_rps,
            max_odom_path_m,
            max_amcl_displacement_m,
            cancel_timeout,
        )):
            raise ValueError('navigation goal values must be finite')
        if not 0.01 <= acceptance_timeout <= 15.0:
            raise ValueError('acceptance timeout is out of range')
        if not 0.01 <= result_timeout <= 180.0:
            raise ValueError('result timeout is out of range')
        if not 0.01 <= monitor_ready_timeout <= 10.0:
            raise ValueError('monitor readiness timeout is out of range')
        if not 0.05 <= max_linear_mps <= 0.30:
            raise ValueError('linear watchdog limit is out of range')
        if not 0.20 <= max_angular_rps <= 1.20:
            raise ValueError('angular watchdog limit is out of range')
        if not 0.10 <= max_odom_path_m <= 0.20:
            raise ValueError('odometry watchdog limit is out of range')
        if not 0.10 <= max_amcl_displacement_m <= 0.20:
            raise ValueError('AMCL watchdog limit is out of range')
        if not 0.01 <= cancel_timeout <= 10.0:
            raise ValueError('cancel timeout is out of range')

        yaw_rad = math.radians(yaw_deg)
        orientation = {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw_rad / 2.0),
            'w': math.cos(yaw_rad / 2.0),
        }
        result = {
            'publish_attempted': False,
            'publish_call_returned': False,
            'publish_outcome_unknown': False,
            'publish_error_type': None,
            'cleanup_attempted': False,
            'cleanup_completed': False,
            'cleanup_error_type': None,
            'topic': '/move_base_simple/goal',
            'message_type': 'geometry_msgs/PoseStamped',
            'frame_id': 'map',
            'x': x,
            'y': y,
            'yaw_deg': yaw_deg,
            'persistent_publisher': True,
            'publisher_created': False,
            'publisher_reused': False,
            'status_monitor_ready': False,
            'amcl_monitor_ready': False,
            'odom_monitor_ready': False,
            'status_messages': 0,
            'move_base_acceptance_known': False,
            'move_base_goal_id': None,
            'move_base_status': None,
            'move_base_status_name': None,
            'move_base_status_text': None,
            'terminal_status_known': False,
            'arrival_known': False,
            'arrived': False,
            'navigation_outcome': 'not_published',
            'acceptance_timeout_sec': acceptance_timeout,
            'result_timeout_sec': result_timeout,
            'cmd_vel_messages': 0,
            'nonzero_cmd_vel_messages': 0,
            'max_linear_mps': 0.0,
            'max_angular_rps': 0.0,
            'amcl_messages': 0,
            'amcl_first_pose': None,
            'amcl_last_pose': None,
            'amcl_displacement_m': None,
            'odom_messages': 0,
            'odom_first_pose': None,
            'odom_last_pose': None,
            'odom_displacement_m': None,
            'odom_path_length_m': 0.0,
            'safety_watchdog_triggered': False,
            'safety_watchdog_reason': None,
            'safety_cancel_attempted': False,
            'safety_cancel_publish_returned': False,
            'safety_cancel_terminal_observed': False,
            'max_linear_limit_mps': max_linear_mps,
            'max_angular_limit_rps': max_angular_rps,
            'max_odom_path_m': max_odom_path_m,
            'max_amcl_displacement_m': max_amcl_displacement_m,
            'observer_cleanup_errors': [],
        }

        if not self._navigation_goal_lock.acquire(False):
            result['setup_error_type'] = 'NavigationGoalInProgress'
            result['navigation_outcome'] = 'goal_in_progress'
            return result

        status_topic = None
        cmd_vel_topic = None
        amcl_topic = None
        odom_topic = None
        status_ready = threading.Event()
        amcl_ready = threading.Event()
        odom_ready = threading.Event()
        accepted = threading.Event()
        terminal = threading.Event()
        watchdog = threading.Event()
        state_lock = threading.Lock()
        baseline_goal_ids = set()
        state = {
            'publish_started': False,
            'tracked_goal_id': None,
            'status': None,
            'status_text': None,
            'cmd_vel_messages': 0,
            'nonzero_cmd_vel_messages': 0,
            'max_linear_mps': 0.0,
            'max_angular_rps': 0.0,
            'amcl_messages': 0,
            'amcl_first_pose': None,
            'amcl_last_pose': None,
            'odom_messages': 0,
            'odom_first_pose': None,
            'odom_last_pose': None,
            'odom_path_length_m': 0.0,
            'watchdog_reason': None,
        }

        def cancel_correlated_goal():
            with state_lock:
                goal_id = state['tracked_goal_id']
            result['safety_cancel_attempted'] = True
            try:
                self._cancel_navigation_goal(goal_id)
                result['safety_cancel_publish_returned'] = True
            except Exception as exc:
                result['safety_cancel_error_type'] = type(exc).__name__
                return False
            observed = terminal.wait(cancel_timeout)
            result['safety_cancel_terminal_observed'] = observed
            return observed

        def status_callback(msg):
            entries = msg.get('status_list', [])
            with state_lock:
                result['status_messages'] += 1
                ids = {
                    entry.get('goal_id', {}).get('id')
                    for entry in entries
                    if entry.get('goal_id', {}).get('id')
                }
                if not state['publish_started']:
                    baseline_goal_ids.update(ids)
                    status_ready.set()
                    return

                if state['tracked_goal_id'] is None:
                    candidates = [
                        entry
                        for entry in entries
                        if entry.get('goal_id', {}).get('id')
                        and entry.get('goal_id', {}).get('id')
                        not in baseline_goal_ids
                    ]
                    if candidates:
                        state['tracked_goal_id'] = candidates[-1][
                            'goal_id'
                        ]['id']

                tracked = state['tracked_goal_id']
                if tracked is None:
                    return
                for entry in entries:
                    if entry.get('goal_id', {}).get('id') != tracked:
                        continue
                    code = entry.get('status')
                    state['status'] = code
                    state['status_text'] = entry.get('text')
                    if code in range(10):
                        accepted.set()
                    if code in (2, 3, 4, 5, 8, 9):
                        terminal.set()
                    break

        def cmd_vel_callback(msg):
            with state_lock:
                if not state['publish_started']:
                    return
                linear = msg.get('linear', {})
                angular = msg.get('angular', {})
                linear_peak = math.hypot(
                    float(linear.get('x', 0.0) or 0.0),
                    float(linear.get('y', 0.0) or 0.0),
                )
                angular_peak = abs(
                    float(angular.get('z', 0.0) or 0.0)
                )
                state['cmd_vel_messages'] += 1
                if linear_peak > 0.001 or angular_peak > 0.001:
                    state['nonzero_cmd_vel_messages'] += 1
                state['max_linear_mps'] = max(
                    state['max_linear_mps'],
                    linear_peak,
                )
                state['max_angular_rps'] = max(
                    state['max_angular_rps'],
                    angular_peak,
                )
                if (
                    state['watchdog_reason'] is None
                    and linear_peak > max_linear_mps + 1e-6
                ):
                    state['watchdog_reason'] = 'linear_speed_limit_exceeded'
                    watchdog.set()
                elif (
                    state['watchdog_reason'] is None
                    and angular_peak > max_angular_rps + 1e-6
                ):
                    state['watchdog_reason'] = 'angular_speed_limit_exceeded'
                    watchdog.set()

        def amcl_callback(msg):
            amcl_ready.set()
            pose = msg.get('pose', {}).get('pose', {}).get('position', {})
            try:
                sample = {
                    'x': float(pose['x']),
                    'y': float(pose['y']),
                }
            except (KeyError, TypeError, ValueError):
                return
            if not all(math.isfinite(value) for value in sample.values()):
                return
            with state_lock:
                if not state['publish_started']:
                    state['amcl_first_pose'] = sample
                    return
                if state['amcl_first_pose'] is None:
                    state['amcl_first_pose'] = sample
                state['amcl_last_pose'] = sample
                state['amcl_messages'] += 1
                first = state['amcl_first_pose']
                displacement = math.hypot(
                    sample['x'] - first['x'],
                    sample['y'] - first['y'],
                )
                if (
                    state['watchdog_reason'] is None
                    and displacement > max_amcl_displacement_m
                ):
                    state['watchdog_reason'] = (
                        'amcl_displacement_limit_exceeded'
                    )
                    watchdog.set()

        def odom_callback(msg):
            odom_ready.set()
            pose = msg.get('pose', {}).get('pose', {})
            position = pose.get('position', {})
            try:
                sample = {
                    'x': float(position['x']),
                    'y': float(position['y']),
                    'yaw': self._pose_yaw(pose),
                }
            except (KeyError, TypeError, ValueError):
                return
            if not all(math.isfinite(value) for value in sample.values()):
                return
            with state_lock:
                previous = state['odom_last_pose']
                if not state['publish_started']:
                    state['odom_first_pose'] = sample
                    state['odom_last_pose'] = sample
                    return
                if state['odom_first_pose'] is None:
                    state['odom_first_pose'] = sample
                if previous is not None:
                    state['odom_path_length_m'] += math.hypot(
                        sample['x'] - previous['x'],
                        sample['y'] - previous['y'],
                    )
                state['odom_last_pose'] = sample
                state['odom_messages'] += 1
                if (
                    state['watchdog_reason'] is None
                    and state['odom_path_length_m'] > max_odom_path_m
                ):
                    state['watchdog_reason'] = 'odom_path_limit_exceeded'
                    watchdog.set()

        try:
            status_topic = roslibpy.Topic(
                self.ros,
                '/move_base/status',
                'actionlib_msgs/GoalStatusArray',
                queue_length=100,
            )
            cmd_vel_topic = roslibpy.Topic(
                self.ros,
                '/cmd_vel',
                'geometry_msgs/Twist',
                queue_length=200,
            )
            amcl_topic = roslibpy.Topic(
                self.ros,
                '/amcl_pose',
                'geometry_msgs/PoseWithCovarianceStamped',
                queue_length=100,
            )
            odom_topic = roslibpy.Topic(
                self.ros,
                '/odom',
                'nav_msgs/Odometry',
                queue_length=200,
            )
            status_topic.subscribe(status_callback)
            cmd_vel_topic.subscribe(cmd_vel_callback)
            amcl_topic.subscribe(amcl_callback)
            odom_topic.subscribe(odom_callback)
        except Exception as exc:
            result['setup_error_type'] = type(exc).__name__
            result['navigation_outcome'] = 'monitor_setup_failed'
            cleanup_errors = result['observer_cleanup_errors']
            for topic in (
                status_topic,
                cmd_vel_topic,
                amcl_topic,
                odom_topic,
            ):
                if topic is not None:
                    self._safe_unsubscribe(topic, cleanup_errors)
            self._navigation_goal_lock.release()
            return result

        try:
            if not status_ready.wait(monitor_ready_timeout):
                result['setup_error_type'] = 'StatusMonitorTimeout'
                result['navigation_outcome'] = 'monitor_setup_failed'
                return result
            result['status_monitor_ready'] = True
            if not amcl_ready.wait(monitor_ready_timeout):
                result['setup_error_type'] = 'AmclMonitorTimeout'
                result['navigation_outcome'] = 'monitor_setup_failed'
                return result
            result['amcl_monitor_ready'] = True
            if not odom_ready.wait(monitor_ready_timeout):
                result['setup_error_type'] = 'OdomMonitorTimeout'
                result['navigation_outcome'] = 'monitor_setup_failed'
                return result
            result['odom_monitor_ready'] = True

            try:
                goal_topic, publisher_created = (
                    self._ensure_navigation_goal_publisher()
                )
                result['publisher_created'] = publisher_created
                result['publisher_reused'] = not publisher_created
            except Exception as exc:
                result['setup_error_type'] = type(exc).__name__
                result['navigation_outcome'] = 'publisher_setup_failed'
                return result

            with state_lock:
                state['publish_started'] = True
            result['publish_attempted'] = True
            try:
                goal_topic.publish(roslibpy.Message({
                    'header': {
                        'frame_id': 'map',
                        'stamp': {'secs': 0, 'nsecs': 0},
                    },
                    'pose': {
                        'position': {'x': x, 'y': y, 'z': 0.0},
                        'orientation': orientation,
                    },
                }))
                result['publish_call_returned'] = True
            except Exception as exc:
                result['publish_outcome_unknown'] = True
                result['publish_error_type'] = type(exc).__name__
                result['navigation_outcome'] = 'publish_outcome_unknown'
                return result

            if not accepted.wait(acceptance_timeout):
                result['navigation_outcome'] = 'acceptance_timeout'
                return result

            result['move_base_acceptance_known'] = True
            result_deadline = time.monotonic() + result_timeout
            while not terminal.is_set():
                if watchdog.is_set():
                    with state_lock:
                        result['safety_watchdog_reason'] = (
                            state['watchdog_reason']
                        )
                    result['safety_watchdog_triggered'] = True
                    cancel_confirmed = cancel_correlated_goal()
                    result['terminal_status_known'] = cancel_confirmed
                    result['arrival_known'] = cancel_confirmed
                    result['arrived'] = False
                    result['navigation_outcome'] = (
                        'safety_watchdog_cancelled'
                        if cancel_confirmed
                        else 'safety_watchdog_cancel_timeout'
                    )
                    return result

                remaining = result_deadline - time.monotonic()
                if remaining <= 0.0:
                    cancel_confirmed = cancel_correlated_goal()
                    result['terminal_status_known'] = cancel_confirmed
                    result['arrival_known'] = cancel_confirmed
                    result['arrived'] = False
                    result['navigation_outcome'] = (
                        'result_timeout_cancelled'
                        if cancel_confirmed
                        else 'result_timeout_cancel_timeout'
                    )
                    return result
                terminal.wait(min(0.10, remaining))

            # Allow the terminal zero velocity and final pose to arrive.
            time.sleep(0.25)
            result['terminal_status_known'] = True
            code = state['status']
            result['arrival_known'] = True
            result['arrived'] = code == 3
            result['navigation_outcome'] = (
                'succeeded' if code == 3 else 'failed'
            )
            return result
        finally:
            with state_lock:
                first_pose = state['amcl_first_pose']
                last_pose = state['amcl_last_pose']
                first_odom = state['odom_first_pose']
                last_odom = state['odom_last_pose']
                result.update({
                    'move_base_goal_id': state['tracked_goal_id'],
                    'move_base_status': state['status'],
                    'move_base_status_name': self._goal_status_name(
                        state['status']
                    ) if state['status'] is not None else None,
                    'move_base_status_text': state['status_text'],
                    'cmd_vel_messages': state['cmd_vel_messages'],
                    'nonzero_cmd_vel_messages': (
                        state['nonzero_cmd_vel_messages']
                    ),
                    'max_linear_mps': state['max_linear_mps'],
                    'max_angular_rps': state['max_angular_rps'],
                    'amcl_messages': state['amcl_messages'],
                    'amcl_first_pose': first_pose,
                    'amcl_last_pose': last_pose,
                    'odom_messages': state['odom_messages'],
                    'odom_first_pose': first_odom,
                    'odom_last_pose': last_odom,
                    'odom_path_length_m': state['odom_path_length_m'],
                    'safety_watchdog_reason': (
                        state['watchdog_reason']
                        or result.get('safety_watchdog_reason')
                    ),
                })
                if first_pose is not None and last_pose is not None:
                    result['amcl_displacement_m'] = math.hypot(
                        last_pose['x'] - first_pose['x'],
                        last_pose['y'] - first_pose['y'],
                    )
                if first_odom is not None and last_odom is not None:
                    result['odom_displacement_m'] = math.hypot(
                        last_odom['x'] - first_odom['x'],
                        last_odom['y'] - first_odom['y'],
                    )
            cleanup_errors = result['observer_cleanup_errors']
            for topic in (
                status_topic,
                cmd_vel_topic,
                amcl_topic,
                odom_topic,
            ):
                if topic is not None:
                    self._safe_unsubscribe(topic, cleanup_errors)
            self._navigation_goal_lock.release()


    def send_current_pose_goal(self):
        amcl = self.get_amcl_pose()
        pose = amcl['raw_pose']

        goal_topic = roslibpy.Topic(
            self.ros,
            '/move_base_simple/goal',
            'geometry_msgs/PoseStamped'
        )

        goal_topic.advertise()
        time.sleep(1.0)

        goal_msg = {
            'header': {
                'frame_id': 'map'
            },
            'pose': pose
        }

        goal_topic.publish(roslibpy.Message(goal_msg))
        goal_topic.unadvertise()

        return {
            'sent': True,
            'goal_x': pose['position']['x'],
            'goal_y': pose['position']['y'],
            'goal_orientation_z': pose['orientation']['z'],
            'goal_orientation_w': pose['orientation']['w'],
        }

    def stop(self, repeat=20):
        del repeat
        raise RuntimeError(
            'direct /cmd_vel publishing is disabled; use estop_hard.sh'
        )

    def close(self):
        try:
            if self._navigation_goal_topic is not None:
                self._navigation_goal_topic.unadvertise()
                self._navigation_goal_topic = None
            if self._navigation_cancel_topic is not None:
                self._navigation_cancel_topic.unadvertise()
                self._navigation_cancel_topic = None
        finally:
            self.ros.terminate()


if __name__ == '__main__':
    bridge = X3Bridge()
    bridge.connect()

    try:
        print('connected =', bridge.ros.is_connected)
        print('odom =', bridge.get_odom())
        print('scan =', bridge.get_scan_summary())
        print('amcl =', bridge.get_amcl_pose())
        print('move_base_status =', bridge.get_move_base_status())
    finally:
        bridge.close()
        print('x3_bridge safe read-only test finished')
