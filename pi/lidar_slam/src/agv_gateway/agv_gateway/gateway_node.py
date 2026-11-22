"""gateway_node — ROS 2 Humble <-> WebSocket (§7) köprüsü.

Thread modeli:
  * Ana thread: asyncio döngüsü + websockets sunucusu (server.GatewayServer).
  * ROS thread: rclpy SingleThreadedExecutor.spin() — tüm abonelik/aksiyon/servis callback'leri burada.
  * asyncio -> ROS: publish() doğrudan (rclpy publish thread-safe). Aksiyon/servis çağrıları (rclpy Humble'da
    thread-safe değil) bir komut kuyruğuna konur ve GuardCondition tetiklenerek ROS thread'inde çalıştırılır.
  * ROS -> asyncio: son mesajlar kilit altında saklanır, asyncio periyodik görevleri okur;
    olaylar server.post_threadsafe() ile gönderilir.

Parametreler: port(8765) host("0.0.0.0") token("") teleop_timeout(0.5) max_lin_vel(0.8) max_ang_vel(1.0)
  telemetry_hz(10) scan_hz(5) map_period(2.0) stations_file("~/.agv/stations.yaml") map_dir("~/.agv/maps")
  map_frame("map") odom_frame("odom") base_frame("base_footprint") nav_action("navigate_to_pose")
  allow_takeover(true) estop_cancels_goal(true) status_timeout(2.0)
"""
from __future__ import annotations

import asyncio
import collections
import logging
import math
import os
import signal
import subprocess
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose

import tf2_ros

from . import protocol as P
from .server import Backend, GatewayServer

try:
    from nav2_msgs.srv import SaveMap
except ImportError:  # pragma: no cover
    SaveMap = None
try:
    from slam_toolbox.srv import SerializePoseGraph
except ImportError:  # slam_toolbox kurulu değilse serialize atlanır
    SerializePoseGraph = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def _tf_to_pose2d(tf) -> Tuple[float, float, float]:
    t = tf.transform.translation
    q = tf.transform.rotation
    return (t.x, t.y, P.quat_to_euler(q.x, q.y, q.z, q.w)[2])


class GatewayNode(Node, Backend):
    def __init__(self):
        Node.__init__(self, "gateway_node")
        dp = self.declare_parameter
        self.p = {
            "port": int(dp("port", 8765).value),
            "host": str(dp("host", "0.0.0.0").value),
            "token": str(dp("token", "").value),
            "teleop_timeout": float(dp("teleop_timeout", 0.5).value),
            "max_lin_vel": float(dp("max_lin_vel", 0.8).value),
            "max_ang_vel": float(dp("max_ang_vel", 1.0).value),
            "telemetry_hz": float(dp("telemetry_hz", 10.0).value),
            "scan_hz": float(dp("scan_hz", 5.0).value),
            "map_period": float(dp("map_period", 2.0).value),
            "stations_file": str(dp("stations_file", "~/.agv/stations.yaml").value),
            "map_dir": os.path.expanduser(str(dp("map_dir", "~/.agv/maps").value)),
            "map_frame": str(dp("map_frame", "map").value),
            "odom_frame": str(dp("odom_frame", "odom").value),
            "base_frame": str(dp("base_frame", "base_footprint").value),
            "nav_action": str(dp("nav_action", "navigate_to_pose").value),
            "allow_takeover": bool(dp("allow_takeover", True).value),
            "estop_cancels_goal": bool(dp("estop_cancels_goal", True).value),
            "status_timeout": float(dp("status_timeout", 2.0).value),
        }
        self.server: Optional[GatewayServer] = None
        self._lock = threading.Lock()

        # ---- son durumlar (kilit altında)
        self._status: Optional[Dict[str, Any]] = None
        self._status_t = 0.0
        self._mode: Optional[str] = None
        self._vel = (0.0, 0.0)
        self._imu = (0.0, 0.0, 0.0, 0.0)
        self._scan: Optional[LaserScan] = None
        self._scan_seq = 0
        self._scan_sent_seq = -1
        self._map: Optional[Dict[str, Any]] = None
        self._map_fp = None
        self._image: Optional[Tuple[int, float, bytes]] = None
        self._image_seq = 0
        self._img_fmt_warned = False
        self._dets: Optional[Tuple[int, float, List[Dict[str, Any]]]] = None
        self._det_seq = 0

        # ---- nav
        self.nav = P.NavTracker()
        self._goal_handle = None
        self._cancel_pending = False

        # ---- TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        # ---- yayıncılar
        self.pub_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.pub_teleop_src = self.create_publisher(String, "/agv/teleop_source", 10)
        self.pub_estop = self.create_publisher(Bool, "/agv/estop", 10)
        self.pub_set_mode = self.create_publisher(String, "/agv/set_mode", 10)
        self.pub_stations = self.create_publisher(String, "/agv/stations", LATCHED)

        # ---- aboneler
        self.create_subscription(String, "/agv/status", self._on_status, 10)
        self.create_subscription(String, "/agv/mode", self._on_mode, LATCHED)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, LATCHED)
        self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/detections/json", self._on_dets, 10)

        # ---- aksiyon / servisler
        self.nav_client = ActionClient(self, NavigateToPose, self.p["nav_action"])
        self.srv_map_saver = self.create_client(SaveMap, "/map_saver/save_map") if SaveMap else None
        self.srv_serialize = (self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
                              if SerializePoseGraph else None)

        # ---- asyncio -> ROS thread komut kuyruğu
        self._cmdq: "collections.deque[Callable[[], None]]" = collections.deque()
        self._gc = self.create_guard_condition(self._drain_cmds)

    # =================================================================== ROS thread yardımcıları
    def run_in_ros(self, fn: Callable[[], None]) -> None:
        self._cmdq.append(fn)
        self._gc.trigger()

    def _drain_cmds(self):
        while self._cmdq:
            fn = self._cmdq.popleft()
            try:
                fn()
            except Exception as e:  # pragma: no cover
                self.get_logger().error(f"komut hatası: {e}")

    def _post(self, obj, to=None):
        if self.server is not None:
            self.server.post_threadsafe(obj, to)

    def _log(self, level, msg):
        getattr(self.get_logger(), {"warn": "warning"}.get(level, level))(msg)
        self._post(P.msg_log(level, msg))

    # =================================================================== callback'ler
    def _on_status(self, msg: String):
        d = P.parse_status_json(msg.data)
        if d is not None:
            with self._lock:
                self._status, self._status_t = d, time.monotonic()

    def _on_mode(self, msg: String):
        with self._lock:
            self._mode = msg.data

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            r = p = y = 0.0  # oryantasyon yok
        else:
            r, p, y = P.quat_to_euler(q.x, q.y, q.z, q.w)
        with self._lock:
            self._imu = (r, p, y, msg.angular_velocity.z)

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            self._scan = msg
            self._scan_seq += 1

    def _on_map(self, msg: OccupancyGrid):
        info = msg.info
        q = info.origin.orientation
        origin = (info.origin.position.x, info.origin.position.y, P.quat_to_euler(q.x, q.y, q.z, q.w)[2])
        fp = P.map_fingerprint(info.width, info.height, info.resolution, origin, msg.data)
        with self._lock:
            if fp == self._map_fp:
                return
            self._map_fp = fp
            seq = (self._map["seq"] + 1) if self._map else 1
            self._map = {"seq": seq, "width": info.width, "height": info.height,
                         "resolution": info.resolution, "origin": origin, "data": msg.data}

    def _on_image(self, msg: CompressedImage):
        if "jpeg" not in msg.format.lower() and "jpg" not in msg.format.lower():
            if not self._img_fmt_warned:
                self._img_fmt_warned = True
                self.get_logger().warning(f"JPEG olmayan sıkıştırılmış görüntü atlanıyor: {msg.format}")
            return
        with self._lock:
            self._image_seq += 1
            self._image = (self._image_seq, time.time(), bytes(msg.data))

    def _on_dets(self, msg: String):
        items = P.parse_detections_json(msg.data)
        with self._lock:
            self._det_seq += 1
            self._dets = (self._det_seq, time.time(), items)

    # =================================================================== TF
    def _lookup_pose(self, target: str, source: str) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        return _tf_to_pose2d(tf)

    def _robot_pose(self) -> Tuple[Optional[Tuple[float, float, float]], Optional[str]]:
        for frame in (self.p["map_frame"], self.p["odom_frame"]):
            pose = self._lookup_pose(frame, self.p["base_frame"])
            if pose is not None:
                return pose, frame
        return None, None

    # =================================================================== Backend: okumalar
    def get_telemetry(self) -> Dict[str, Any]:
        with self._lock:
            fresh = time.monotonic() - self._status_t < self.p["status_timeout"]
            status = dict(self._status) if (self._status and fresh) else None
            mode, vel, imu = self._mode, self._vel, self._imu
        pose, frame = self._robot_pose()
        nav_state, goal = self.nav_state()
        return P.msg_telemetry(time.time(), status, mode, pose, vel, imu, nav_state, goal, frame)

    def current_mode(self) -> Optional[str]:
        with self._lock:
            if self._status and "mode" in self._status:
                return str(self._status["mode"])
            return self._mode

    def get_scan(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            scan, seq = self._scan, self._scan_seq
        if scan is None or seq == self._scan_sent_seq:
            return None
        self._scan_sent_seq = seq
        laser = scan.header.frame_id or "laser_link"
        frame, sensor = None, None
        for fr in (self.p["map_frame"], self.p["odom_frame"]):
            sensor = self._lookup_pose(fr, laser)
            if sensor is not None:
                frame = fr
                break
        if sensor is None:
            # CONTRACT-NOTE: TF yoksa noktalar lazer frame'inde, "frame": "<laser_link>" ile gönderilir.
            frame = laser
        pts = P.scan_to_points(list(scan.ranges), scan.angle_min, scan.angle_increment,
                               scan.range_min, scan.range_max, sensor, P.MAX_SCAN_POINTS)
        pose, pframe = self._robot_pose()
        if pframe != frame and frame in (self.p["map_frame"], self.p["odom_frame"]):
            pose = self._lookup_pose(frame, self.p["base_frame"])
        return {"t": time.time(), "pose": pose, "points": pts, "frame": frame}

    def get_map(self):
        with self._lock:
            return self._map

    def get_image(self):
        with self._lock:
            return self._image

    def get_detections(self):
        with self._lock:
            return self._dets

    # =================================================================== Backend: komutlar
    def publish_teleop(self, vx: float, wz: float) -> None:
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        # CONTRACT-NOTE: source is announced right BEFORE each Twist (mode_logic.TeleopSourceTracker)
        self.pub_teleop_src.publish(String(data="app"))
        self.pub_teleop.publish(t)

    def publish_estop(self, active: bool) -> None:
        self.pub_estop.publish(Bool(data=bool(active)))

    def publish_set_mode(self, mode: str) -> None:
        self.pub_set_mode.publish(String(data=mode))

    def stations_changed(self, json_str: str) -> None:
        self.pub_stations.publish(String(data=json_str))

    # ---- nav
    def nav_state(self):
        with self._lock:
            return self.nav.state, self.nav.goal

    def send_goal(self, x: float, y: float, yaw: float) -> Tuple[bool, str]:
        if not self.nav_client.server_is_ready():
            return False, f"Nav2 '{self.p['nav_action']}' aksiyon sunucusu hazır değil"
        with self._lock:
            token = self.nav.start((x, y, yaw))
            self._cancel_pending = False
        self.run_in_ros(lambda: self._ros_send_goal(token, x, y, yaw))
        return True, f"hedef gönderildi ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)"

    def _ros_send_goal(self, token, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.p["map_frame"]
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = P.yaw_to_quat(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_goal_response(token, f))

    def _on_goal_response(self, token, fut):
        try:
            gh = fut.result()
        except Exception as e:
            gh = None
            self._log("error", f"hedef gönderilemedi: {e}")
        if gh is None or not gh.accepted:
            with self._lock:
                self.nav.rejected(token)
            self._log("warn", "Nav2 hedefi reddetti")
            return
        with self._lock:
            current = token == self.nav.token
            cancel = self._cancel_pending and current
            if current:
                self._goal_handle = gh
        if not current or cancel:
            gh.cancel_goal_async()  # eski / iptal istenmiş hedef
            if not current:
                return
        gh.get_result_async().add_done_callback(lambda f: self._on_goal_result(token, f))

    def _on_goal_result(self, token, fut):
        try:
            status = fut.result().status
        except Exception:
            status = P.GOAL_STATUS_ABORTED
        with self._lock:
            st = self.nav.finished(token, status)
            if st is not None:
                self._goal_handle = None
            goal = self.nav.goal
        if st is not None:
            lvl = "info" if st == P.NAV_SUCCEEDED else "warn"
            self._log(lvl, f"navigasyon sonucu: {st}" + (f" ({goal[0]:.2f}, {goal[1]:.2f})" if goal else ""))

    def cancel_goal(self) -> Tuple[bool, str]:
        with self._lock:
            if self.nav.state != P.NAV_ACTIVE:
                return False, "aktif hedef yok"
            gh = self._goal_handle
            if gh is None:
                self._cancel_pending = True  # kabul gelince iptal edilecek
            self.nav.canceled_locally()
            self._goal_handle = None
        if gh is not None:
            self.run_in_ros(lambda: gh.cancel_goal_async())
        return True, "hedef iptal edildi"

    # ---- save_map
    def _call_service(self, client, req, timeout: float):
        """Herhangi bir (ROS olmayan) thread'den servis çağrısı; çağrı ROS thread'inde yapılır."""
        if client is None:
            return None, "servis tipi kurulu değil"
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=1.0):
            return None, f"{client.srv_name} servisi yok"
        done = threading.Event()
        box: Dict[str, Any] = {}

        def _start():
            f = client.call_async(req)
            box["f"] = f
            f.add_done_callback(lambda _f: done.set())
        self.run_in_ros(_start)
        if not done.wait(timeout):
            return None, f"{client.srv_name} zaman aşımı"
        try:
            return box["f"].result(), ""
        except Exception as e:
            return None, str(e)

    def save_map(self, name: str, reply) -> None:
        threading.Thread(target=self._save_map_worker, args=(name, reply), daemon=True).start()

    def _save_map_worker(self, name: str, reply):
        os.makedirs(self.p["map_dir"], exist_ok=True)
        base = os.path.join(self.p["map_dir"], name)
        msgs, ok_any, ok_pgm = [], False, False

        # 1) slam_toolbox pose graph (lokalizasyon modunda tekrar yüklemek için)
        if SerializePoseGraph is not None:
            req = SerializePoseGraph.Request()
            req.filename = base
            res, err = self._call_service(self.srv_serialize, req, 15.0)
            if res is not None:
                # Humble slam_toolbox: response.result (uint8, 0 = SUCCESS); eski sürümlerde boş response.
                r = getattr(res, "result", 0)
                good = (r == 0)
                ok_any |= good
                msgs.append(f"posegraph {'OK' if good else 'HATA(' + str(r) + ')'}: {base}.posegraph")
            else:
                msgs.append(f"posegraph atlandı: {err}")
        else:
            msgs.append("posegraph atlandı: slam_toolbox kurulu değil")

        # 2) nav2 map_saver servisi, yoksa map_saver_cli
        if self.srv_map_saver is not None and self.srv_map_saver.service_is_ready():
            req = SaveMap.Request()
            req.map_topic = "map"
            req.map_url = base
            req.image_format = "pgm"
            req.map_mode = "trinary"
            req.free_thresh = 0.25
            req.occupied_thresh = 0.65
            res, err = self._call_service(self.srv_map_saver, req, 15.0)
            ok_pgm = bool(res is not None and res.result)
            msgs.append(f"map_saver {'OK' if ok_pgm else 'HATA ' + err}: {base}.yaml/.pgm")
        if not ok_pgm:
            try:
                cp = subprocess.run(
                    ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", base,
                     "--ros-args", "-p", "save_map_timeout:=10.0", "-p", "map_subscribe_transient_local:=true"],
                    capture_output=True, text=True, timeout=30)
                ok_pgm = cp.returncode == 0 and os.path.exists(base + ".yaml")
                tail = (cp.stdout + cp.stderr).strip().splitlines()[-1:] if not ok_pgm else []
                msgs.append(f"map_saver_cli {'OK' if ok_pgm else 'HATA'}: {base}.yaml/.pgm"
                            + (f" ({tail[0]})" if tail else ""))
            except (OSError, subprocess.TimeoutExpired) as e:
                msgs.append(f"map_saver_cli çalıştırılamadı: {e}")
        ok = ok_pgm or ok_any
        reply(ok, "; ".join(msgs))


# ======================================================================================= main
def main(args=None):
    logging.basicConfig(level=logging.INFO, format="[gateway] %(levelname)s %(message)s")
    rclpy.init(args=args)
    node = GatewayNode()
    stations = P.StationStore(node.p["stations_file"])
    server = GatewayServer(
        node, stations, host=node.p["host"], port=node.p["port"], token=node.p["token"],
        teleop_timeout=node.p["teleop_timeout"], max_lin=node.p["max_lin_vel"], max_ang=node.p["max_ang_vel"],
        telemetry_hz=node.p["telemetry_hz"], scan_hz=node.p["scan_hz"], map_period=node.p["map_period"],
        allow_takeover=node.p["allow_takeover"], estop_cancels_goal=node.p["estop_cancels_goal"])
    node.server = server

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, name="ros_spin", daemon=True)
    ros_thread.start()
    node.get_logger().info(f"WebSocket ws://{node.p['host']}:{node.p['port']} "
                           f"(token {'açık' if node.p['token'] else 'kapalı'}), istasyonlar: {stations.path}")

    async def _amain():
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, server.request_stop)
            except (NotImplementedError, RuntimeError):
                pass
        await server.serve_forever()

    try:
        asyncio.run(_amain())
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_teleop(0.0, 0.0)
        except Exception:
            pass
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
