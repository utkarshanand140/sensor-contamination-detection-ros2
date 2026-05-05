#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String, Int32

import numpy as np
import struct
import cv2
import time

# ---- Thresholds ---- changed here
# LIDAR_REDUCED = 34198
# LIDAR_CRITICAL = 33556

LIDAR_REDUCED = 34500
LIDAR_CRITICAL = 33800

CAM_REDUCED = 313
CAM_CRITICAL = 201

# ---- Clean baseline thresholds ----
CLEAN_LAP_THRESHOLD = 350
CLEAN_LIDAR_THRESHOLD = 0.1

NORMAL = "NORMAL"
REDUCED = "REDUCED"
CRITICAL = "CRITICAL"


class ContaminationMonitor(Node):

    def __init__(self):
        super().__init__('contamination_monitor')

        # Subscribers
        self.create_subscription(PointCloud2, '/lidar/cloud/device_id47', self.lidar_callback, 10)
        self.create_subscription(Image, '/visionary2/bgr/device_id4', self.camera_callback, 10)

        # Publishers
        self.pub = self.create_publisher(String, '/contamination_status', 10)
        self.traffic_pub = self.create_publisher(Int32, '/trafic_light_color', 10)

        # State
        self.lidar_score = 0.0
        self.camera_score = 0.0

        self.lidar_ok = False
        self.camera_ok = False

        self.score_history = []
        self.SMOOTH_WINDOW = 5

        # Time window
        self.history = []
        self.WINDOW_DURATION = 4

        # Consensus
        self.CONSENSUS_THRESHOLD = 0.7

        # State machine
        self.current_state = NORMAL

        # Cooldown (IMPORTANT)
        self.last_change_time = time.time()
        self.COOLDOWN_TIME = 1.0  # seconds

        self.lap_var = 0.0

        self.get_logger().info("Contamination Monitor Node Started")

    # =========================
    # LiDAR
    # =========================
    def lidar_callback(self, msg):

        _, intensity = self.decode_pc2(msg)

        if len(intensity) == 0:
            self.lidar_ok = False
            return

        self.lidar_ok = True

        mean_i = float(np.mean(intensity))

        if mean_i <= LIDAR_CRITICAL:
            self.lidar_score = 1.0
        elif mean_i <= LIDAR_REDUCED:
            self.lidar_score = (LIDAR_REDUCED - mean_i) / (LIDAR_REDUCED - LIDAR_CRITICAL)
        else:
            self.lidar_score = 0.0

        self.update()

    # =========================
    # Camera
    # =========================
    def camera_callback(self, msg):

        gray = self.decode_image(msg)

        if gray is None:
            self.camera_ok = False
            return

        self.camera_ok = True

        self.lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        brightness = float(gray.mean())

        if self.lap_var < CAM_CRITICAL and brightness > 80:
            self.camera_score = 0.1
        elif self.lap_var <= CAM_CRITICAL:
            self.camera_score = 1.0
        elif self.lap_var <= CAM_REDUCED:
            self.camera_score = (CAM_REDUCED - self.lap_var) / (CAM_REDUCED - CAM_CRITICAL)
        else:
            self.camera_score = 0.0

        self.update()

    # =========================
    # Fusion
    # =========================
    def fuse_scores(self):

        if not self.lidar_ok and not self.camera_ok:
            return 0.0

        if self.lidar_ok and self.camera_ok:
            fused = 0.65 * self.lidar_score + 0.35 * self.camera_score

            if self.camera_score > 0.8 and self.lidar_score < 0.1:
                fused = min(fused, 0.7)

            if self.lidar_score > 0.6 and self.camera_score > 0.6:
                fused = max(fused, 0.6)

        elif self.lidar_ok:
            fused = self.lidar_score
        else:
            fused = self.camera_score

        return float(np.clip(fused, 0.0, 1.0))

    # =========================
    # Update + Hybrid Decision
    # =========================
    def update(self):

        fused = self.fuse_scores()
        self.get_logger().info(f"LIDAR: {self.lidar_score:.2f} | CAM: {self.camera_score:.2f} | FUSED: {fused:.2f}") #changed here

        # smoothing
        self.score_history.append(fused)
        if len(self.score_history) > self.SMOOTH_WINDOW:
            self.score_history.pop(0)

        smoothed = float(np.mean(self.score_history))

        # base classification
        if smoothed >= 0.60:
            current_sev = CRITICAL
        elif smoothed >= 0.25:
            current_sev = REDUCED
        else:
            current_sev = NORMAL

        # ---- time window ----
        now = time.time()
        self.history.append((now, current_sev))
        self.history = [(t, s) for (t, s) in self.history if now - t <= self.WINDOW_DURATION]

        severities = [s for (_, s) in self.history]

        if len(severities) < 5:
            return

        # ---- consensus ----
        counts = {
            NORMAL: severities.count(NORMAL),
            REDUCED: severities.count(REDUCED),
            CRITICAL: severities.count(CRITICAL)
        }

        total = len(severities)
        best_state = max(counts, key=counts.get)
        best_ratio = counts[best_state] / total

        # ---- clean baseline check ----
        is_clean = (self.lap_var > CLEAN_LAP_THRESHOLD) and (self.lidar_score < CLEAN_LIDAR_THRESHOLD)

        # ---- LIDAR DOMINANCE (FINAL FIX) ----  ##last_change
        if self.lidar_score > 0.4 and self.camera_score < 0.2:
            best_state = CRITICAL
            best_ratio = 1.0

        # ---- cooldown check ----
        time_since_change = now - self.last_change_time
        in_cooldown = time_since_change < self.COOLDOWN_TIME

        new_state = self.current_state

        # ---- dust escalation (NEW) ---- changed here
        if self.lidar_score > 0.5 and self.camera_score > 0.3:
            best_state = CRITICAL
            best_ratio = 1.0

        # ---- STATE MACHINE ----

        # ESCALATION (fast)
        if best_state in [REDUCED, CRITICAL] and best_ratio >= 0.4: #change here
            if best_state != self.current_state and not in_cooldown:
                new_state = best_state
                self.last_change_time = now

        # RECOVERY (strict)
        elif best_state == NORMAL and best_ratio >= self.CONSENSUS_THRESHOLD and is_clean:
            if self.current_state != NORMAL and not in_cooldown:
                new_state = NORMAL
                self.last_change_time = now

        # update state
        self.current_state = new_state

        # ---- publish ----
        status_msg = String()
        status_msg.data = self.current_state
        self.pub.publish(status_msg)

        traffic_msg = Int32()

        if self.current_state == NORMAL:
            traffic_msg.data = 2
        elif self.current_state == REDUCED:
            traffic_msg.data = 3
        else:
            traffic_msg.data = 1

        self.traffic_pub.publish(traffic_msg)

        self.get_logger().info(f"{self.current_state}")

    # =========================
    # Decode PointCloud2
    # =========================
    def decode_pc2(self, msg):

        fields = {f.name: f for f in msg.fields}
        if 'x' not in fields:
            return None, np.array([])

        ps = msg.point_step
        data = bytes(msg.data)
        n = msg.width * msg.height

        ox = fields['x'].offset
        oy = fields['y'].offset
        oz = fields['z'].offset
        oi = fields['intensity'].offset if 'intensity' in fields else None

        intensity = []

        for i in range(n):
            base = i * ps

            try:
                x = struct.unpack_from('<f', data, base + ox)[0]
                y = struct.unpack_from('<f', data, base + oy)[0]
                z = struct.unpack_from('<f', data, base + oz)[0]

                if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                    if oi is not None:
                        val = struct.unpack_from('<f', data, base + oi)[0]
                        intensity.append(val)

            except:
                continue

        return None, np.array(intensity)

    # =========================
    # Decode Image
    # =========================
    def decode_image(self, msg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        except:
            return None


def main():
    rclpy.init()
    node = ContaminationMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#================================================================

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import PointCloud2, Image
# from std_msgs.msg import String, Int32

# import numpy as np
# import struct
# import cv2
# import time

# # ---- Thresholds ----
# LIDAR_REDUCED = 34198
# LIDAR_CRITICAL = 33556

# CAM_REDUCED = 313
# CAM_CRITICAL = 201

# NORMAL = "NORMAL"
# REDUCED = "REDUCED"
# CRITICAL = "CRITICAL"


# class ContaminationMonitor(Node):

#     def __init__(self):
#         super().__init__('contamination_monitor')

#         # Subscribers
#         self.create_subscription(PointCloud2, '/lidar/cloud/device_id47', self.lidar_callback, 10)
#         self.create_subscription(Image, '/visionary2/bgr/device_id4', self.camera_callback, 10)

#         # Publishers
#         self.pub = self.create_publisher(String, '/contamination_status', 10)
#         self.traffic_pub = self.create_publisher(Int32, '/trafic_light_color', 10)

#         # State
#         self.lidar_score = 0.0
#         self.camera_score = 0.0

#         self.lidar_ok = False
#         self.camera_ok = False

#         self.score_history = []
#         self.SMOOTH_WINDOW = 5

#         # Time-window history
#         self.history = []
#         self.WINDOW_DURATION = 6.0  # seconds

#         self.lap_var = 0.0

#         self.get_logger().info("Contamination Monitor Node Started")

#     # =========================
#     # LiDAR
#     # =========================
#     def lidar_callback(self, msg):

#         _, intensity = self.decode_pc2(msg)

#         if len(intensity) == 0:
#             self.lidar_ok = False
#             return

#         self.lidar_ok = True

#         mean_i = float(np.mean(intensity))

#         if mean_i <= LIDAR_CRITICAL:
#             self.lidar_score = 1.0
#         elif mean_i <= LIDAR_REDUCED:
#             self.lidar_score = (LIDAR_REDUCED - mean_i) / (LIDAR_REDUCED - LIDAR_CRITICAL)
#         else:
#             self.lidar_score = 0.0

#         self.update()

#     # =========================
#     # Camera
#     # =========================
#     def camera_callback(self, msg):

#         gray = self.decode_image(msg)

#         if gray is None:
#             self.camera_ok = False
#             return

#         self.camera_ok = True

#         self.lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
#         brightness = float(gray.mean())

#         # Robust camera logic
#         if self.lap_var < CAM_CRITICAL and brightness > 80:
#             self.camera_score = 0.1
#         elif self.lap_var <= CAM_CRITICAL:
#             self.camera_score = 1.0
#         elif self.lap_var <= CAM_REDUCED:
#             self.camera_score = (CAM_REDUCED - self.lap_var) / (CAM_REDUCED - CAM_CRITICAL)
#         else:
#             self.camera_score = 0.0

#         self.update()

#     # =========================
#     # Fusion
#     # =========================
#     def fuse_scores(self):

#         if not self.lidar_ok and not self.camera_ok:
#             return 0.0

#         if self.lidar_ok and self.camera_ok:
#             fused = 0.65 * self.lidar_score + 0.35 * self.camera_score

#             # suppress camera-only spikes
#             if self.camera_score > 0.8 and self.lidar_score < 0.1:
#                 fused = min(fused, 0.7)

#             # strong agreement
#             if self.lidar_score > 0.6 and self.camera_score > 0.6:
#                 fused = max(fused, 0.6)

#         elif self.lidar_ok:
#             fused = self.lidar_score
#         else:
#             fused = self.camera_score

#         return float(np.clip(fused, 0.0, 1.0))

#     # =========================
#     # Update + Time Window Voting
#     # =========================
#     def update(self):

#         fused = self.fuse_scores()

#         # smoothing
#         self.score_history.append(fused)
#         if len(self.score_history) > self.SMOOTH_WINDOW:
#             self.score_history.pop(0)

#         smoothed = float(np.mean(self.score_history))

#         # base classification
#         if smoothed >= 0.60:
#             current_sev = CRITICAL
#         elif smoothed >= 0.25:
#             current_sev = REDUCED
#         else:
#             current_sev = NORMAL

#         # ---- time window ----
#         now = time.time()
#         self.history.append((now, current_sev))

#         # remove old entries
#         self.history = [(t, s) for (t, s) in self.history if now - t <= self.WINDOW_DURATION]

#         severities = [s for (_, s) in self.history]

#         if len(severities) < 5:
#             return  # not enough data

#         # ---- majority voting ----
#         if severities.count(CRITICAL) >= 3:
#             severity = CRITICAL
#         else:
#             severity = max(set(severities), key=severities.count)

#         # ---- publish ----
#         status_msg = String()
#         status_msg.data = severity
#         self.pub.publish(status_msg)

#         traffic_msg = Int32()

#         if severity == NORMAL:
#             traffic_msg.data = 2
#         elif severity == REDUCED:
#             traffic_msg.data = 3
#         else:
#             traffic_msg.data = 1

#         self.traffic_pub.publish(traffic_msg)

#         self.get_logger().info(f"{severity}")

#     # =========================
#     # Decode PointCloud2
#     # =========================
#     def decode_pc2(self, msg):

#         fields = {f.name: f for f in msg.fields}
#         if 'x' not in fields:
#             return None, np.array([])

#         ps = msg.point_step
#         data = bytes(msg.data)
#         n = msg.width * msg.height

#         ox = fields['x'].offset
#         oy = fields['y'].offset
#         oz = fields['z'].offset
#         oi = fields['intensity'].offset if 'intensity' in fields else None

#         intensity = []

#         for i in range(n):
#             base = i * ps

#             try:
#                 x = struct.unpack_from('<f', data, base + ox)[0]
#                 y = struct.unpack_from('<f', data, base + oy)[0]
#                 z = struct.unpack_from('<f', data, base + oz)[0]

#                 if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
#                     if oi is not None:
#                         val = struct.unpack_from('<f', data, base + oi)[0]
#                         intensity.append(val)

#             except:
#                 continue

#         return None, np.array(intensity)

#     # =========================
#     # Decode Image
#     # =========================
#     def decode_image(self, msg):
#         try:
#             img = np.frombuffer(msg.data, dtype=np.uint8)
#             img = img.reshape(msg.height, msg.width, 3)
#             return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         except:
#             return None


# def main():
#     rclpy.init()
#     node = ContaminationMonitor()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

