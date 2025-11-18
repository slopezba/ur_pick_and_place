# vision_pick_init/vision_node.py
import threading
import time
from collections import Counter
from statistics import mean

import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseArray, Quaternion

# ---- optional helper if your _utils is missing ----
try:
    from _utils import _improveLight
except Exception:
    def _improveLight(img, *_args, **_kw):
        return np.asarray(img)

def _mean_distance(win, depth_frame, u, v):
    try:
        pad = int((win - 1) / 2)
        vals = []
        for i in range(u - pad, u + pad + 1):
            for j in range(v - pad, v + pad + 1):
                d = depth_frame.get_distance(i, j)
                if d > 0:
                    vals.append(d)
        return mean(vals) if vals else 0.0
    except Exception:
        return 0.0

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ---- params (override via ROS params if needed) ----
        self.topic = self.declare_parameter('pose_topic', '/vision/init_poses').get_parameter_value().string_value
        self.conf_thres = float(self.declare_parameter('conf', 0.1).value)
        self.iou_thres  = float(self.declare_parameter('iou', 0.7).value)
        self.imgsz      = int(self.declare_parameter('imgsz', 640).value)
        self.visualize  = bool(self.declare_parameter('visualize', True).value)
        self.device     = self.declare_parameter('device', 'cpu').get_parameter_value().string_value
        self.model_path = self.declare_parameter('model', 'best640lobb.onnx').get_parameter_value().string_value
        self.is_obb     = bool(self.declare_parameter('obb', True).value)  # your model is obb

        # ---- ROS publisher ----
        self.pub = self.create_publisher(PoseArray, self.topic, 10)

        # ---- RealSense init ----
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth,   640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color,   640, 480, rs.format.bgr8, 30)
        prof = self.pipeline.start(cfg)

        depth_sensor = prof.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 0)
        self.align_color = rs.align(rs.stream.color)

        color_stream = prof.get_stream(rs.stream.color)
        self.intr = color_stream.as_video_stream_profile().get_intrinsics()

        # ---- YOLO init ----
        self.model = YOLO(self.model_path, task='obb' if self.is_obb else 'detect')
        self.model.overrides['device'] = self.device
        # rename classes (adjust if needed)
        self.model.names[0] = 'RipeBlackberry'
        self.model.names[1] = 'RipeStrawberry'
        self.model.names[2] = 'UnripeBlackberry'
        self.model.names[3] = 'UnripeStrawberry'

        # ---- start worker thread ----
        self.stop_flag = False
        self.worker = threading.Thread(target=self._loop, daemon=True)
        self.worker.start()
        self.get_logger().info(f'VisionNode started. Publishing PoseArray on {self.topic}')

    def _loop(self):
        prev = time.time()
        while not self.stop_flag and rclpy.ok():
            frames = self.pipeline.wait_for_frames()
            frames = self.align_color.process(frames)
            d = frames.get_depth_frame()
            c = frames.get_color_frame()
            if not d or not c:
                continue

            img = _improveLight(c.get_data(), -0.7)
            res = self.model(img, device=self.device, verbose=False,
                             conf=self.conf_thres, iou=self.iou_thres, imgsz=self.imgsz)[0]
            iterate = res.obb if self.is_obb else res.boxes

            # build PoseArray in the camera frame
            pa = PoseArray()
            pa.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_color_optical_frame')

            class_ids = []
            for det in iterate:
                if self.is_obb:
                    t = det.xywhr.cpu().data.numpy()
                    u, v = int(round(t[0][0])), int(round(t[0][1]))
                    dist = _mean_distance(11, d, u, v)
                    cls = int(det.cls)
                else:
                    t = det.xywh.cpu().data.numpy()
                    u, v = int(round(t[0][0])), int(round(t[0][1]))
                    dist = _mean_distance(7, d, u, v)
                    cls = int(det.cls)

                if dist <= 0:
                    continue

                X, Y, Z = rs.rs2_deproject_pixel_to_point(self.intr, [u, v], dist)
                pose = Pose()
                pose.position.x = float(X)
                pose.position.y = float(Y)
                pose.position.z = float(Z)
                pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # no orientation yet
                pa.poses.append(pose)
                class_ids.append(cls)

            self.pub.publish(pa)

            # optional on-screen debug
            if self.visualize:
                annotated = res.plot(line_width=1, font_size=0.5)
                now = time.time()
                fps = 1.0 / max(1e-6, now - prev); prev = now
                cv2.putText(annotated, f'FPS: {fps:.2f}', (annotated.shape[1]-150, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                cv2.putText(annotated, f'Count: {len(iterate)}', (annotated.shape[1]-150, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                try:
                    cv2.imshow('VisionNode', annotated)
                    if cv2.waitKey(1) & 0xFF == 27:
                        self.stop_flag = True
                except Exception:
                    pass

            time.sleep(0.01)

    def destroy_node(self):
        self.stop_flag = True
        try:
            self.pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
