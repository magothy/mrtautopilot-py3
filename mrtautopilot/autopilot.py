import time
import threading
import multiprocessing
import socket
import asyncio
import logging
import struct
from dataclasses import dataclass
from enum import Enum


import mrtmavlink


@dataclass
class HealthItem:
    health_id: str
    status_id: int
    description: str


HEALTH_ITEMS = [
    HealthItem(
        health_id="health-monitor-status",
        status_id=0x10000000,
        description="Health Monitor is not properly configured",
    ),
    HealthItem(
        health_id="geo-fence", status_id=0x100000, description="Geofence is breached"
    ),
    HealthItem(
        health_id="mcu-timeout",
        status_id=0x8000,
        description="MCU Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="gps-timeout",
        status_id=0x20,
        description="GPS Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="ahrs-timeout",
        status_id=0x200000,
        description="AHRS Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="depth-timeout",
        status_id=0x400000,
        description="Altimeter Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="joystick-timeout",
        status_id=0x10000,
        description="Joystick Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="low-fuel",
        status_id=0x2000000,
        description="Low Fuel or Battery",
    ),
    HealthItem(
        health_id="oc-timeout",
        status_id=0x8000000,
        description="Operator Console Communication Timeout",
    ),
    HealthItem(
        health_id="low-disk-space", status_id=0x1000000, description="Low Disk Space"
    ),
]


class HealthResponseType(Enum):
    Ignore = 0
    Halt = 1
    Loiter = 2
    GoRally = 3
    GoFirst = 4
    GoLast = 5
    GoLaunch = 6
    Custom = 7


@dataclass
class HealthResponse:
    item: HealthItem
    response: HealthResponseType


class MagothyCustomMainMode(Enum):
    MANUAL = 1
    GUIDED = 2
    SIMPLE = 3


class MagothyCustomSubModeGuided(Enum):
    READY = 1
    MISSION = 2
    LOITER = 3
    UNHEALTHY_MISSION = 4
    MISSION_PLANNING = 5
    UNHEALTHY_MISSION_PLANNING = 6


class MagothyCustomSubModeManualBitmask(Enum):
    STABILIZED_SURGE = 0x01
    STABILIZED_SWAY = 0x02
    STABILIZED_DEPTH_HEAVE = 0x04
    STABILIZED_ALTITUDE_HEAVE = 0x08
    STABILIZED_ROLL = 0x10
    STABILIZED_PITCH = 0x20
    STABILIZED_YAW = 0x40
    STABILIZED_DEGRADED = 0x80


@dataclass
class LowBandwidth:
    def __init__(self, msg, health_items: list[HealthItem]):
        self.main_mode = MagothyCustomMainMode((msg.custom_mode >> 16) & 0xFF)
        self.sub_mode = MagothyCustomSubModeGuided((msg.custom_mode >> 24) & 0xFF)

        self.latitude_deg = msg.lat / 1e7 if msg.lat != 0x7FFFFFFF else None
        self.longitude_deg = msg.lon / 1e7 if msg.lon != 0x7FFFFFFF else None
        self.battery_voltage_V = (
            msg.voltage_battery / 1000 if msg.voltage_battery != 0xFFFF else None
        )
        self.battery_current_A = (
            msg.current_battery / 100 if msg.current_battery >= 0 else None
        )
        self.battery_soc = msg.battery_remaining if msg.battery_remaining >= 0 else None
        self.mission_item_seq = msg.mission_seq if msg.mission_seq != 0xFF else None
        self.speed_mps = msg.speed / 100 if msg.speed != 0xFFFF else None
        self.course_deg = msg.course / 100 if msg.course != 0xFFFF else None
        self.heading_deg = msg.heading / 100 if msg.heading != 0xFFFF else None
        self.num_satellites = msg.satellites_visible
        self.target_speed_mps = (
            msg.desired_speed / 100 if msg.desired_speed != 0xFFFF else None
        )
        self.target_course_deg = (
            msg.desired_course / 100 if msg.desired_course != 0xFFFF else None
        )
        self.is_position_independent = msg.is_position_independent == 1
        self.position_error_m = (
            msg.position_error / 100 if msg.position_error != 0xFFFF else None
        )

        self.enabled_health_items = []
        self.triggered_health_items = []
        for item in health_items:
            if msg.onboard_control_sensors_health & item.status_id == 0:
                self.triggered_health_items.append(item)

        self.fault_response = None
        if self.main_mode == MagothyCustomMainMode.GUIDED and self.sub_mode in [
            MagothyCustomSubModeGuided.UNHEALTHY_MISSION,
            MagothyCustomSubModeGuided.UNHEALTHY_MISSION_PLANNING,
        ]:
            details = (msg.custom_mode >> 8) & 0xFF

            response_type = HealthResponseType((details >> 5) & 0x07)
            status_id = 1 << (details & 0x1F)

            for item in health_items:
                if status_id == item.status_id:
                    self.fault_response = HealthResponse(item, response_type)
                    break

            assert self.fault_response is not None

    main_mode: MagothyCustomMainMode
    sub_mode: MagothyCustomSubModeGuided
    latitude_deg: float | None
    longitude_deg: float | None
    battery_voltage_V: float | None
    battery_current_A: float | None
    battery_soc: int | None
    mission_item_seq: int | None
    speed_mps: float | None
    course_deg: float | None
    heading_deg: float | None
    num_satellites: int
    target_speed_mps: float | None
    target_course_deg: float | None
    is_position_independent: bool
    position_error_m: float | None
    triggered_health_items: list[HealthItem]
    fault_response: HealthResponse | None



class SendMavlink:
    def __init__(self, sock):
        self.address = ("127.0.0.1", 14551)
        self.sock = sock

    def write(self, data: bytes):
        self.sock.sendto(data, self.address)


class MavlinkThread:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.system_id: int | None = None

        self.is_started = False
        self.is_started_cv = threading.Condition()
        self.conn = mrtmavlink.MAVLink(
            SendMavlink(self.sock), 254, mrtmavlink.MAV_COMP_ID_SYSTEM_CONTROL
        )

        self.is_done = False
        self.thread = threading.Thread(target=self._thread, name="MrtMavlink")

        self.low_bandwidth_queue = multiprocessing.Queue(maxsize=1)

    def __enter__(self):
        self.start()

    def __exit__(self, _type, _value, _traceback):
        self.stop()

    def start(self):
        self.thread.start()

        with self.is_started_cv:
            while not self.is_started:
                self.is_started_cv.wait()

        return self.thread

    def stop(self):
        self.is_done = True
        self.thread.join()

    async def handle_data(self, loop: asyncio.AbstractEventLoop):
        while not self.is_done:
            data = await loop.sock_recv(self.sock, 280)

            messages = self.conn.parse_buffer(data)
            if messages:
                for msg in messages:
                    self._msg_callback(msg)

    def _msg_callback(self, msg: mrtmavlink.MAVLink_message):
        logging.debug(f"Received: {msg.msgname}")

        sys_id = msg.get_srcSystem()
        if self.system_id != sys_id:
            self.system_id = msg.get_srcSystem()
            logging.info(f"Setting System ID to {sys_id}")

        if msg.get_msgId() == mrtmavlink.MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH:
            data = LowBandwidth(msg, HEALTH_ITEMS)

            if not self.low_bandwidth_queue.full():
                self.low_bandwidth_queue.put_nowait(data)

    def send_heartbeat(self):
        self.loop.create_task(self._send_heartbeat())

    async def _send_heartbeat(self):
        logging.info("Sending Heartbeat")
        self.conn.heartbeat_send(
            mrtmavlink.MAV_TYPE_GCS, mrtmavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )

    async def _run(self):
        with self.is_started_cv:
            self.is_started = True
            self.is_started_cv.notify()

        self.loop = asyncio.get_running_loop()
        self.loop.create_task(self.handle_data(self.loop))

        while not self.is_done:
            logging.debug("Sending system time")
            self._send_system_time()
            await asyncio.sleep(1.0)

    def _thread(self):
        asyncio.run(self._run())

    def _send_system_time(self):
        self.conn.system_time_send(int(time.time() * 1e6), 0)

    def send_autopilot_stop(self):
        self.loop.create_task(self._send_autopilot_stop())

    async def _send_autopilot_stop(self):
        if not self.system_id:
            logging.warn("System ID not set, not sending STOP")
            return
        logging.info("Sending STOP")

        # custom_mode is a uint32
        # param2 is a float
        # convert custom_mode to a float via struct.unpack
        custom_mode = bytearray(
            [
                0,
                0,
                MagothyCustomMainMode.GUIDED.value,
                MagothyCustomSubModeGuided.READY.value,
            ]
        )

        self.conn.command_long_send(
            self.system_id,  # target_system
            mrtmavlink.MAV_COMP_ID_AUTOPILOT1,  # target_component
            mrtmavlink.MAV_CMD_DO_SET_MODE,  # command
            0,  # confirmation
            0,  # param1
            struct.unpack("f", custom_mode)[0],  # param2 (custom_mode)
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

    def send_waypoint(self, lat_deg: float, lon_deg: float, speed_mps: float):
        self.loop.create_task(self._send_waypoint(lat_deg, lon_deg, speed_mps))

    async def _send_waypoint(self, lat_deg: float, lon_deg: float, speed_mps: float):
        if not self.system_id:
            logging.warn("System ID not set, not sending waypoint")
            return
        logging.info(
            f"Sending Waypoint to lat {lat_deg}°, lon {lon_deg}°, speed {speed_mps} m/s"
        )

        self.conn.command_long_send(
            self.system_id,  # target_system
            mrtmavlink.MAV_COMP_ID_AUTOPILOT1,  # target_component
            mrtmavlink.MAV_CMD_WAYPOINT_USER_1,  # command
            0,  # confirmation
            0,  # param1
            speed_mps,  # param2 (speed_mps)
            0,  # param3
            0,  # param4
            lat_deg * 1e7,  # param5 (latitude_deg * 1e7)
            lon_deg * 1e7,  # param6 (longitude_deg * 1e7)
            0,  # param7
        )