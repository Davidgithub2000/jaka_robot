#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dual_pick_coordinator.py

Station-level coordinator for dual-arm picking.

Inputs:
- left/right target topics: count every visual target sent to each arm.
- left/right target_result topics: count every target attempt result from each arm.
- station_start topic: optional reset/start signal from chassis or task manager.
- vision_done topic: optional reliable "no more targets in this station" signal.

Outputs:
- station_done Bool: true when the current station is finished.
- station_status String: JSON status for logging/debugging.

The coordinator does not move the chassis or the arms. It only publishes a
station-level completion signal that a chassis/navigation node can subscribe to.
"""

import json
import time
from threading import Lock

import rospy
from std_msgs.msg import Bool, Float32MultiArray, String


class DualPickCoordinator:
    def __init__(self):
        self.left_target_topic = rospy.get_param(
            "~left_target_topic", "/left_arm/custom_arm_data"
        )
        self.right_target_topic = rospy.get_param(
            "~right_target_topic", "/right_arm/custom_arm_data"
        )
        self.left_result_topic = rospy.get_param(
            "~left_result_topic", "/left_picker/target_result"
        )
        self.right_result_topic = rospy.get_param(
            "~right_result_topic", "/right_picker/target_result"
        )
        self.station_start_topic = rospy.get_param(
            "~station_start_topic", "/dual_picker/station_start"
        )
        self.vision_done_topic = rospy.get_param(
            "~vision_done_topic", "/dual_picker/vision_done"
        )
        self.station_done_topic = rospy.get_param(
            "~station_done_topic", "/dual_picker/station_done"
        )
        self.station_status_topic = rospy.get_param(
            "~station_status_topic", "/dual_picker/station_status"
        )

        self.auto_start_on_target = self._get_bool_param("~auto_start_on_target", True)
        self.require_vision_done = self._get_bool_param("~require_vision_done", False)
        self.allow_empty_station_done = self._get_bool_param(
            "~allow_empty_station_done", True
        )
        self.done_requires_all_success = self._get_bool_param(
            "~done_requires_all_success", False
        )
        self.require_arm_safe = self._get_bool_param("~require_arm_safe", True)

        self.target_quiet_s = float(rospy.get_param("~target_quiet_s", 3.0))
        self.done_hold_s = float(rospy.get_param("~done_hold_s", 0.5))
        self.check_rate_hz = float(rospy.get_param("~check_rate_hz", 10.0))

        self.lock = Lock()
        self.station_seq = int(rospy.get_param("~initial_station_seq", 0))
        self._reset_state_locked(active=False, increment_seq=False)

        self.station_done_pub = rospy.Publisher(
            self.station_done_topic, Bool, queue_size=1, latch=True
        )
        self.station_status_pub = rospy.Publisher(
            self.station_status_topic, String, queue_size=10, latch=True
        )

        self.left_target_sub = rospy.Subscriber(
            self.left_target_topic,
            Float32MultiArray,
            lambda msg: self._target_cb("left", msg),
            queue_size=20,
        )
        self.right_target_sub = rospy.Subscriber(
            self.right_target_topic,
            Float32MultiArray,
            lambda msg: self._target_cb("right", msg),
            queue_size=20,
        )
        self.left_result_sub = rospy.Subscriber(
            self.left_result_topic,
            String,
            lambda msg: self._result_cb("left", msg),
            queue_size=20,
        )
        self.right_result_sub = rospy.Subscriber(
            self.right_result_topic,
            String,
            lambda msg: self._result_cb("right", msg),
            queue_size=20,
        )
        self.station_start_sub = rospy.Subscriber(
            self.station_start_topic, Bool, self._station_start_cb, queue_size=5
        )
        self.vision_done_sub = rospy.Subscriber(
            self.vision_done_topic, Bool, self._vision_done_cb, queue_size=5
        )

        self.station_done_pub.publish(Bool(False))
        self._publish_status("node_started")

        rospy.loginfo(
            "dual_pick_coordinator started: left_target=%s, right_target=%s, "
            "left_result=%s, right_result=%s, station_done=%s, vision_done=%s, "
            "require_vision_done=%s, require_arm_safe=%s, target_quiet_s=%.3f",
            self.left_target_topic,
            self.right_target_topic,
            self.left_result_topic,
            self.right_result_topic,
            self.station_done_topic,
            self.vision_done_topic,
            self.require_vision_done,
            self.require_arm_safe,
            self.target_quiet_s,
        )

    def _get_bool_param(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        if text in ("true", "1", "yes", "y", "on"):
            return True
        if text in ("false", "0", "no", "n", "off"):
            return False
        rospy.logwarn("Cannot parse bool param %s=%s, using %s", name, value, default)
        return bool(default)

    def _is_empty_target_msg(self, msg):
        data = list(getattr(msg, "data", []))
        if len(data) < 6:
            return False
        try:
            return all(abs(float(value)) <= 1e-6 for value in data[:6])
        except (TypeError, ValueError):
            return False
    def _empty_side_result(self):
        return {
            "done": 0,
            "success": 0,
            "failed": 0,
            "arm_safe": 0,
            "arm_unsafe": 0,
            "last_arm_safe": True,
        }

    def _reset_state_locked(self, active=True, increment_seq=True):
        if increment_seq:
            self.station_seq += 1
        self.active = bool(active)
        self.done_sent = False
        self.vision_done = False
        self.started_stamp = time.time() if active else 0.0
        self.first_target_stamp = 0.0
        self.last_target_stamp = 0.0
        self.last_result_stamp = 0.0
        self.targets = {"left": 0, "right": 0}
        self.results = {
            "left": self._empty_side_result(),
            "right": self._empty_side_result(),
        }
        self.last_reason = "reset"

    def _start_station_locked(self, reason):
        self._reset_state_locked(active=True, increment_seq=True)
        self.last_reason = reason
        rospy.loginfo("Station %s started: %s", self.station_seq, reason)
        self.station_done_pub.publish(Bool(False))

    def _station_start_cb(self, msg):
        with self.lock:
            if bool(msg.data):
                self._start_station_locked("station_start")
            else:
                self._reset_state_locked(active=False, increment_seq=False)
                self.station_done_pub.publish(Bool(False))
                self.last_reason = "station_cancel_or_idle"
            self._publish_status_locked(self.last_reason)

    def _vision_done_cb(self, msg):
        if not bool(msg.data):
            return
        with self.lock:
            if not self.active:
                self._start_station_locked("vision_done_without_start")
            self.vision_done = True
            self.last_reason = "vision_done"
            self._publish_status_locked(self.last_reason)

    def _target_cb(self, side, _msg):
        if self._is_empty_target_msg(_msg):
            rospy.loginfo_throttle(5.0, "Ignore %s empty target placeholder", side)
            return

        now = time.time()
        with self.lock:
            if not self.active:
                if not self.auto_start_on_target:
                    rospy.logwarn("Ignore %s target because station is not active", side)
                    return
                self._start_station_locked("first_target")

            self.targets[side] += 1
            if self.first_target_stamp <= 0.0:
                self.first_target_stamp = now
            self.last_target_stamp = now
            self.last_reason = "{}_target_received".format(side)
            rospy.loginfo(
                "Station %s got %s target: left=%s, right=%s",
                self.station_seq,
                side,
                self.targets["left"],
                self.targets["right"],
            )
            self._publish_status_locked(self.last_reason)

    def _parse_result_msg(self, msg):
        try:
            return json.loads(str(msg.data))
        except Exception:
            return {"success": False, "reason": "invalid_result_json"}

    def _result_cb(self, side, msg):
        result = self._parse_result_msg(msg)
        success = bool(result.get("success", False))
        arm_safe = bool(result.get("arm_safe", False))

        with self.lock:
            if not self.active:
                self._start_station_locked("result_without_active_station")

            self.results[side]["done"] += 1
            if success:
                self.results[side]["success"] += 1
            else:
                self.results[side]["failed"] += 1
            if arm_safe:
                self.results[side]["arm_safe"] += 1
            else:
                self.results[side]["arm_unsafe"] += 1
            self.results[side]["last_arm_safe"] = bool(arm_safe)
            self.last_result_stamp = time.time()
            self.last_reason = "{}_result_{}".format(
                side, "success" if success else "failed"
            )
            rospy.loginfo(
                "Station %s got %s result: success=%s, arm_safe=%s, left_done=%s, right_done=%s",
                self.station_seq,
                side,
                success,
                arm_safe,
                self.results["left"]["done"],
                self.results["right"]["done"],
            )
            self._publish_status_locked(self.last_reason)

    def _total_targets_locked(self):
        return int(self.targets["left"] + self.targets["right"])

    def _total_results_locked(self):
        return int(self.results["left"]["done"] + self.results["right"]["done"])

    def _total_failed_locked(self):
        return int(self.results["left"]["failed"] + self.results["right"]["failed"])

    def _total_arm_unsafe_locked(self):
        return int(self.results["left"]["arm_unsafe"] + self.results["right"]["arm_unsafe"])

    def _current_arm_unsafe_sides_locked(self):
        unsafe_sides = []
        for side in ("left", "right"):
            if self.targets[side] > 0 or self.results[side]["done"] > 0:
                if not bool(self.results[side].get("last_arm_safe", False)):
                    unsafe_sides.append(side)
        return unsafe_sides

    def _publish_status(self, reason):
        with self.lock:
            self._publish_status_locked(reason)

    def _publish_status_locked(self, reason, done=None):
        now = time.time()
        payload = {
            "stamp": now,
            "station_seq": int(self.station_seq),
            "active": bool(self.active),
            "done": bool(self.done_sent if done is None else done),
            "vision_done": bool(self.vision_done),
            "reason": str(reason),
            "targets": {
                "left": int(self.targets["left"]),
                "right": int(self.targets["right"]),
                "total": self._total_targets_locked(),
            },
            "results": {
                "left": dict(self.results["left"]),
                "right": dict(self.results["right"]),
                "total_done": self._total_results_locked(),
                "total_failed": self._total_failed_locked(),
                "total_arm_unsafe": self._total_arm_unsafe_locked(),
                "current_arm_unsafe_sides": self._current_arm_unsafe_sides_locked(),
            },
            "timing": {
                "target_quiet_s": float(self.target_quiet_s),
                "seconds_since_last_target": (
                    now - self.last_target_stamp if self.last_target_stamp > 0 else None
                ),
                "seconds_since_last_result": (
                    now - self.last_result_stamp if self.last_result_stamp > 0 else None
                ),
            },
        }
        self.station_status_pub.publish(json.dumps(payload, ensure_ascii=False))

    def _completion_blocker_locked(self):
        if not self.active:
            return "station_not_active"
        if self.done_sent:
            return "done_already_sent"

        total_targets = self._total_targets_locked()
        total_results = self._total_results_locked()

        if self.require_vision_done and not self.vision_done:
            return "waiting_vision_done"

        if total_targets == 0:
            if self.vision_done and self.allow_empty_station_done:
                return None
            return "waiting_targets"

        if total_results < total_targets:
            return "waiting_pick_results"

        if self.done_requires_all_success and self._total_failed_locked() > 0:
            return "has_failed_targets"

        unsafe_sides = self._current_arm_unsafe_sides_locked()
        if self.require_arm_safe and unsafe_sides:
            return "waiting_arm_safe:{}".format(",".join(unsafe_sides))

        now = time.time()
        if not self.require_vision_done and not self.vision_done:
            if self.last_target_stamp <= 0.0:
                return "waiting_first_target_stamp"
            if (now - self.last_target_stamp) < self.target_quiet_s:
                return "waiting_target_quiet"

        latest_activity = max(self.last_target_stamp, self.last_result_stamp)
        if latest_activity > 0.0 and (now - latest_activity) < self.done_hold_s:
            return "waiting_done_hold"

        return None

    def _maybe_publish_station_done(self):
        with self.lock:
            blocker = self._completion_blocker_locked()
            if blocker is not None:
                return

            self.done_sent = True
            self.active = False
            self.last_reason = "station_done"
            self.station_done_pub.publish(Bool(True))
            self._publish_status_locked("station_done", done=True)
            rospy.loginfo(
                "Station %s done: targets=%s, results=%s, failed=%s",
                self.station_seq,
                self._total_targets_locked(),
                self._total_results_locked(),
                self._total_failed_locked(),
            )

    def spin(self):
        rate = rospy.Rate(max(1.0, self.check_rate_hz))
        while not rospy.is_shutdown():
            self._maybe_publish_station_done()
            rate.sleep()


def main():
    rospy.init_node("dual_pick_coordinator")
    coordinator = DualPickCoordinator()
    coordinator.spin()


if __name__ == "__main__":
    main()
