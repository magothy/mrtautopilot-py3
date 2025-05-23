import time
import threading
import multiprocessing
import socket
import asyncio
import datetime
import logging
import struct
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import List, Union, Tuple, Callable

import mrtmavlink
from . import mission

import zstandard


SYSTEM_ID = 254  # system id for the mavlink thread


@dataclass
class HealthItem:
    health_id: str
    status_id: int
    description: str


HEALTH_ITEMS = [
    HealthItem(
        health_id="health-monitor-status",
        status_id=mission.HealthId.HealthMonitor.value,
        description="Health Monitor is not properly configured",
    ),
    HealthItem(
        health_id="geo-fence",
        status_id=mission.HealthId.GeoFence.value,
        description="Geofence is breached",
    ),
    HealthItem(
        health_id="mcu-timeout",
        status_id=mission.HealthId.McuTimeout.value,
        description="MCU Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="gps-timeout",
        status_id=mission.HealthId.GpsTimeout.value,
        description="GPS Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="ahrs-timeout",
        status_id=mission.HealthId.AhrsTimeout.value,
        description="AHRS Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="depth-timeout",
        status_id=mission.HealthId.DepthTimeout.value,
        description="Altimeter Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="joystick-timeout",
        status_id=mission.HealthId.JoystickTimeout.value,
        description="Joystick Data Is Missing or Stale",
    ),
    HealthItem(
        health_id="low-fuel",
        status_id=mission.HealthId.LowFuel.value,
        description="Low Fuel or Battery",
    ),
    HealthItem(
        health_id="oc-timeout",
        status_id=mission.HealthId.OcTimeout.value,
        description="Operator Console Communication Timeout",
    ),
    HealthItem(
        health_id="low-disk-space",
        status_id=mission.HealthId.LowDiskSpace.value,
        description="Low Disk Space",
    ),
]


# https://stackoverflow.com/a/30357446
def crc16(data: bytes) -> int:
    crc: int = 0xFFFF
    msb = crc >> 8
    lsb = crc & 255
    for c in data:
        x = c ^ msb
        x ^= x >> 4
        msb = (lsb ^ (x >> 3) ^ (x << 4)) & 255
        lsb = (x ^ (x << 5)) & 255
    return (msb << 8) + lsb


@dataclass
class HealthResponse:
    item: HealthItem
    response: mission.FaultResponseType


class MagothyCustomMainMode(Enum):
    MANUAL = 1
    GUIDED = 2
    SIMPLE = 3


class MagothyCustomSubModeGuided(Enum):
    UNSET = 0
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


class AutopilotMode(Enum):
    Unknown = 0
    Standby = 1
    Manual = 2
    HealthyMission = 3
    UnhealthyMission = 4
    Loiter = 5
    MissionPlanning = 6
    UnhealthyMissionPlanning = 7


class ExpectedMissionAck:
    def __init__(
        self,
        xfr: Union[mrtmavlink.MAVLink_magothy_mission_transfer_message, None] = None,
    ):
        self.target_system = SYSTEM_ID
        if xfr is None:
            self.session_id = 0
            self.chunk_index = 0
            self.num_chunk = 0
            self.crc16 = 0
        else:
            self.session_id = xfr.session_id
            self.chunk_index = xfr.chunk_index
            self.num_chunk = xfr.num_chunk
            self.crc16 = xfr.crc16

    def equals(self, ack: mrtmavlink.MAVLink_magothy_mission_ack_message) -> bool:
        return (
            self.target_system == ack.target_system
            and self.session_id == ack.session_id
            and self.chunk_index == ack.chunk_index
            and self.num_chunk == ack.num_chunk
            and self.crc16 == ack.crc16
        )

    def __repr__(self):
        return f"ExpectedMissionAck(sys_id={self.target_system}, session={self.session_id}, chunk={self.chunk_index}, num_chunk={self.num_chunk}, crc16={self.crc16})"


@dataclass
class LowBandwidth:
    def __init__(
        self,
        msg: mrtmavlink.MAVLink_magothy_low_bandwidth_message,
        health_items: List[HealthItem],
    ):
        self.main_mode = MagothyCustomMainMode((msg.custom_mode >> 16) & 0xFF)
        self.sub_mode = MagothyCustomSubModeGuided((msg.custom_mode >> 24) & 0xFF)

        self.autopilot_mode = AutopilotMode.Unknown
        if self.main_mode == MagothyCustomMainMode.MANUAL:
            self.autopilot_mode = AutopilotMode.Manual
        elif self.main_mode == MagothyCustomMainMode.GUIDED:
            if self.sub_mode == MagothyCustomSubModeGuided.READY:
                self.autopilot_mode = AutopilotMode.Standby
            elif self.sub_mode == MagothyCustomSubModeGuided.MISSION:
                self.autopilot_mode = AutopilotMode.HealthyMission
            elif self.sub_mode == MagothyCustomSubModeGuided.UNHEALTHY_MISSION:
                self.autopilot_mode = AutopilotMode.UnhealthyMission
            elif self.sub_mode == MagothyCustomSubModeGuided.LOITER:
                self.autopilot_mode = AutopilotMode.Loiter
            elif self.sub_mode == MagothyCustomSubModeGuided.MISSION_PLANNING:
                self.autopilot_mode = AutopilotMode.MissionPlanning
            elif self.sub_mode == MagothyCustomSubModeGuided.UNHEALTHY_MISSION_PLANNING:
                self.autopilot_mode = AutopilotMode.UnhealthyMissionPlanning
            else:
                logging.debug(f"Unknown sub mode: {self.sub_mode}")
        else:
            logging.debug(f"Unknown main mode: {self.main_mode}")

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

            response_type = mission.FaultResponseType((details >> 5) & 0x07)
            status_id = 1 << (details & 0x1F)

            for item in health_items:
                if status_id == item.status_id:
                    self.fault_response = HealthResponse(item, response_type)
                    break

            assert self.fault_response is not None

    autopilot_mode: AutopilotMode
    main_mode: MagothyCustomMainMode
    sub_mode: MagothyCustomSubModeGuided
    latitude_deg: Union[float, None]
    longitude_deg: Union[float, None]
    battery_voltage_V: Union[float, None]
    battery_current_A: Union[float, None]
    battery_soc: Union[int, None]
    mission_item_seq: Union[int, None]
    speed_mps: Union[float, None]
    course_deg: Union[float, None]
    heading_deg: Union[float, None]
    num_satellites: int
    target_speed_mps: Union[float, None]
    target_course_deg: Union[float, None]
    is_position_independent: bool
    position_error_m: Union[float, None]
    triggered_health_items: List[HealthItem]
    fault_response: Union[HealthResponse, None]


class GpsFixType(Enum):
    NoGps = 0
    NoFix = 1
    Fix2d = 2
    Fix3d = 3
    Dgps = 4
    RtkFloat = 5
    RtkFixed = 6
    Static = 7
    Ppp = 8


@dataclass
class Gps2Raw:
    def __init__(self, msg: mrtmavlink.MAVLink_gps2_raw_message):
        self.timestamp = datetime.datetime.fromtimestamp(
            msg.time_usec / 1e6, tz=datetime.timezone.utc
        )
        self.fix_type = GpsFixType(msg.fix_type)

        if msg.lat != 0x7FFFFFFF:
            self.latitude_deg = msg.lat / 1e7
        else:
            self.latitude_deg = None

        if msg.lon != 0x7FFFFFFF:
            self.longitude_deg = msg.lon / 1e7
        else:
            self.longitude_deg = None

        if msg.alt != 0x7FFFFFF:
            self.altitude_m = msg.alt / 1000
        else:
            self.altitude_m = None

        if msg.eph != 0xFFFF:
            self.hdop = msg.eph / 100
        else:
            self.hdop = None

        if msg.epv != 0xFFFF:
            self.vdop = msg.epv / 100
        else:
            self.vdop = None

        if msg.vel != 0xFFFF:
            self.velocity_mps = msg.vel / 100
        else:
            self.velocity_mps = None

        if msg.cog != 0xFFFF:
            self.course_deg = msg.cog / 100
        else:
            self.course_deg = None

        if msg.satellites_visible != 0xFF:
            self.num_set = msg.satellites_visible
        else:
            self.num_set = None

    timestamp: datetime.datetime
    fix_type: GpsFixType
    latitude_deg: Union[float, None]
    longitude_deg: Union[float, None]
    altitude_m: Union[float, None]
    hdop: Union[float, None]
    vdop: Union[float, None]
    velocity_mps: Union[float, None]
    course_deg: Union[float, None]
    num_set: Union[int, None]


@dataclass
class ParamUint8:
    value: int


@dataclass
class ParamInt8:
    value: int


@dataclass
class ParamUint16:
    value: int


@dataclass
class ParamInt16:
    value: int


@dataclass
class ParamUint32:
    value: int


@dataclass
class ParamInt32:
    value: int


@dataclass
class ParamReal32:
    value: float


ParamValueType = Union[
    ParamUint8,
    ParamInt8,
    ParamUint16,
    ParamInt16,
    ParamUint32,
    ParamInt32,
    ParamReal32,
]


@dataclass
class ParamValue:
    @staticmethod
    def from_mavlink(msg: mrtmavlink.MAVLink_param_value_message):
        buffer = struct.pack("<f", msg.param_value)
        if msg.param_type == mrtmavlink.MAV_PARAM_TYPE_UINT8:
            value = ParamUint8(int(struct.unpack("BBBB", buffer)[0]))
        elif msg.param_type == mrtmavlink.MAV_PARAM_TYPE_INT8:
            value = ParamInt8(int(struct.unpack("bBBB", buffer)[0]))
        elif msg.param_type == mrtmavlink.MAV_PARAM_TYPE_UINT16:
            value = ParamUint16(int(struct.unpack("<HBB", buffer)[0]))
        elif msg.param_type == mrtmavlink.MAV_PARAM_TYPE_INT16:
            value = ParamInt16(int(struct.unpack("<hBB", buffer)[0]))
        elif msg.param_type == mrtmavlink.MAV_PARAM_TYPE_UINT32:
            value = ParamUint32(int(struct.unpack("<I", buffer)[0]))
        elif msg.param_type == mrtmavlink.MAV_PARAM_TYPE_INT32:
            value = ParamInt32(int(struct.unpack("<i", buffer)[0]))
        elif msg.param_type == mrtmavlink.MAV_PARAM_TYPE_REAL32:
            value = ParamReal32(float(struct.unpack("<f", buffer)[0]))
        else:
            raise ValueError(f"Unsupported parameter type: {msg.param_type}")

        return ParamValue(msg.param_id, value, msg.param_index, msg.param_count)

    def to_mavlink(self) -> mrtmavlink.MAVLink_param_value_message:
        if isinstance(self.value, ParamUint8):
            typ = mrtmavlink.MAV_PARAM_TYPE_UINT8
            value = struct.pack("BBBB", self.value.value, 0, 0, 0)
        elif isinstance(self.value, ParamInt8):
            typ = mrtmavlink.MAV_PARAM_TYPE_INT8
            value = struct.pack("bBBB", self.value.value, 0, 0, 0)
        elif isinstance(self.value, ParamUint16):
            typ = mrtmavlink.MAV_PARAM_TYPE_UINT16
            value = struct.pack("<HBB", self.value.value, 0, 0)
        elif isinstance(self.value, ParamInt16):
            typ = mrtmavlink.MAV_PARAM_TYPE_INT16
            value = struct.pack("<hBB", self.value.value, 0, 0)
        elif isinstance(self.value, ParamUint32):
            typ = mrtmavlink.MAV_PARAM_TYPE_UINT32
            value = struct.pack("<I", self.value.value)
        elif isinstance(self.value, ParamInt32):
            typ = mrtmavlink.MAV_PARAM_TYPE_INT32
            value = struct.pack("<i", self.value.value)
        elif isinstance(self.value, ParamReal32):
            typ = mrtmavlink.MAV_PARAM_TYPE_REAL32
            value = struct.pack("<f", self.value.value)
        else:
            raise ValueError(f"Unsupported parameter type: {type(self.value)}")

        return mrtmavlink.MAVLink_param_value_message(
            param_id=self.name.encode("utf-8"),
            param_value=struct.unpack("f", value)[0],
            param_type=typ,
            param_index=self.param_idx,
            param_count=self.num_param,
        )

    name: str
    value: ParamValueType
    param_idx: int
    num_param: int


class MavlinkThread:
    def __init__(
        self,
        bind_address: str = "0.0.0.0",
        remote_address: str = "127.0.0.1",
        remote_port: int = 14551,
        multicast_group: str = "",
        multicast_port: int = 14550,
    ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.msg_callback: Callable[[mrtmavlink.MAVLink_message], None] = lambda x: None

        if multicast_group:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            self.sock.bind((bind_address, multicast_port))

            mreq = struct.pack(
                "4sl", socket.inet_aton(multicast_group), socket.INADDR_ANY
            )
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        else:
            self.sock.bind((bind_address, 0))

        self.system_id: Union[int, None] = None

        self.remote_address = remote_address
        self.remote_port = remote_port
        self.remote_address_list = [remote_address]
        if remote_address == "127.0.0.1":
            try:
                self.remote_address_list.extend(
                    socket.gethostbyname_ex(socket.gethostname())[2]
                )
            except socket.gaierror:
                logging.warning("Failed to get local IP address")

        self.is_started = False
        self.is_started_cv = threading.Condition()
        self.conn = mrtmavlink.MAVLink(
            self, SYSTEM_ID, mrtmavlink.MAV_COMP_ID_SYSTEM_CONTROL
        )

        self.is_done = False
        self.thread = threading.Thread(target=self._thread, name="MrtMavlink")

        self.low_bandwidth_queue: multiprocessing.Queue[LowBandwidth] = (
            multiprocessing.Queue(maxsize=5)
        )

        self.low_bandwidth_queue_fileobj: int = (
            self.low_bandwidth_queue._reader.fileno()  # type: ignore
        )

        self.is_sending_mission = False
        self.mission_session_id = 0
        self.expected_mission_ack = ExpectedMissionAck()
        self.expected_cmd_uuid = 0
        self.ack_result: Union[int, None] = None

        self.is_requesting_parameters = False
        self.param_value: Union[ParamValue, None] = None

    def write(self, data: bytes):
        self.sock.sendto(data, (self.remote_address, self.remote_port))

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, _type, _value, _traceback):  # type: ignore
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

    def connection_made(self, transport: asyncio.DatagramTransport):
        pass

    def datagram_received(self, data: bytes, addr: Tuple[str, int]):
        messages = self.conn.parse_buffer(data)
        if addr[0] not in self.remote_address_list:
            return

        if addr[1] != self.remote_port:
            self.remote_port = addr[1]
            logging.info(f"Setting Remote Port to {self.remote_port}")

        if messages:
            for msg in messages:
                self._msg_callback(msg)

    async def _notify_param_result(self, result: ParamValue):
        async with self.param_cond:
            self.param_result = result
            self.param_cond.notify()

    async def _notify_ack_result(self, result: int):
        async with self.ack_cond:
            self.ack_result = result
            self.ack_cond.notify()

    def register_message_callback(
        self, callback: Callable[[mrtmavlink.MAVLink_message], None]
    ):
        self.msg_callback = callback

    def _msg_callback(self, msg: mrtmavlink.MAVLink_message):
        self.msg_callback(msg)
        sys_id = msg.get_srcSystem()
        logging.debug(f"Received: {msg.msgname}, sys_id={sys_id}, {msg}")
        if self.system_id != sys_id:
            self.system_id = msg.get_srcSystem()
            logging.info(f"Setting System ID to {sys_id}")

        if type(msg) is mrtmavlink.MAVLink_magothy_low_bandwidth_message:
            data = LowBandwidth(msg, HEALTH_ITEMS)

            if not self.low_bandwidth_queue.full():
                self.low_bandwidth_queue.put_nowait(data)
        elif type(msg) is mrtmavlink.MAVLink_statustext_message:
            if (
                msg.severity == mrtmavlink.MAV_SEVERITY_CRITICAL
                or msg.severity == mrtmavlink.MAV_SEVERITY_EMERGENCY
                or msg.severity == mrtmavlink.MAV_SEVERITY_ALERT
            ):
                logging.critical(f"Vehicle Status Text: {msg.text}")
            elif msg.severity == mrtmavlink.MAV_SEVERITY_ERROR:
                logging.error(f"Vehicle Status Text: {msg.text}")
            elif (
                msg.severity == mrtmavlink.MAV_SEVERITY_WARNING
                or msg.severity == mrtmavlink.MAV_SEVERITY_NOTICE
            ):
                logging.warning(f"Vehicle Status Text: {msg.text}")
            elif msg.severity == mrtmavlink.MAV_SEVERITY_INFO:
                logging.info(f"Vehicle Status Text: {msg.text}")
            else:
                logging.debug(f"Vehicle Status Text: {msg.text}")
        elif type(msg) is mrtmavlink.MAVLink_magothy_mission_ack_message:
            if self.expected_mission_ack.equals(msg):
                logging.debug(f"Received Mission Ack: {msg}")
                self.loop.create_task(self._notify_ack_result(msg.result))
        elif type(msg) is mrtmavlink.MAVLink_command_ack_message:
            if (
                msg.command == mrtmavlink.MAV_CMD_DO_SET_MISSION_CURRENT
                and msg.target_system == self.system_id
                and msg.result_param2 == self.expected_cmd_uuid
            ):
                logging.debug(f"Received Command Ack: {msg}")
                self.loop.create_task(self._notify_ack_result(msg.result))
        elif type(msg) is mrtmavlink.MAVLink_param_value_message:
            param = ParamValue.from_mavlink(msg)
            logging.debug(f"Received Param Value: {param}")
            self.loop.create_task(self._notify_param_result(param))

    def send_heartbeat(self):
        self.loop.call_soon_threadsafe(lambda: self._send_heartbeat())

    def _send_heartbeat(self):
        logging.info("Sending Heartbeat")
        self.conn.heartbeat_send(
            mrtmavlink.MAV_TYPE_GCS, mrtmavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )

    def get_parameter(
        self,
        id: str,
        callback: Callable[[Union[ParamValue, None]], None],
    ):
        self.loop.create_task(self._get_parameter(id, callback))

    async def _get_parameter(
        self,
        id: str,
        callback: Callable[[Union[ParamValue, None]], None],
    ):
        assert len(id) <= 16
        if self.system_id is None:
            logging.info("Failed to get parameter, system_id not set")
            return

        if self.is_requesting_parameters:
            logging.warning("Already requesting parameters")
            callback(None)
            return

        RETRY_PERIOD_S = 0.25
        try:
            self.is_requesting_parameters = True

            logging.info(f"Getting Param {id}")

            self.param_result = None
            self.conn.param_request_read_send(
                self.system_id,
                mrtmavlink.MAV_COMP_ID_AUTOPILOT1,
                id.encode("utf-8"),
                -1,
            )
            try:
                async with self.param_cond:
                    await asyncio.wait_for(self.param_cond.wait(), RETRY_PERIOD_S)
            except asyncio.TimeoutError:
                logging.error("Timeout waiting for parameter ack")
                callback(None)
                return

            if self.param_result is not None:
                if self.param_result.name == id:
                    callback(self.param_result)
                else:
                    logging.warning(
                        f"Set parameter {id} but got {self.param_result.name}"
                    )
                    callback(None)
            else:
                logging.warning(f"Set parameter {id} but got None")
                callback(None)
        finally:
            self.is_requesting_parameters = False

    def set_parameter(
        self,
        id: str,
        value: ParamValueType,
        callback: Callable[[Union[ParamValue, None]], None] = lambda x: None,
    ):
        self.loop.create_task(self._set_parameter(id, value, callback))

    async def _set_parameter(
        self,
        id: str,
        value: ParamValueType,
        callback: Callable[[Union[ParamValue, None]], None],
    ):
        assert len(id) <= 16
        if self.system_id is None:
            logging.info("Failed to set parameter, system_id not set")
            return

        if self.is_requesting_parameters:
            logging.warning("Already requesting parameters")
            callback(None)
            return

        RETRY_PERIOD_S = 0.25
        try:
            self.is_requesting_parameters = True

            logging.info(f"Setting Param {id} to {value}")

            if isinstance(value, ParamUint8):
                val_bytes = struct.pack("BBBB", int(value.value), 0, 0, 0)
                typ = mrtmavlink.MAV_PARAM_TYPE_UINT8
            elif isinstance(value, ParamInt8):
                val_bytes = struct.pack("bBBB", int(value.value), 0, 0, 0)
                typ = mrtmavlink.MAV_PARAM_TYPE_INT8
            elif isinstance(value, ParamUint16):
                val_bytes = struct.pack("<HBB", int(value.value), 0, 0)
                typ = mrtmavlink.MAV_PARAM_TYPE_UINT16
            elif isinstance(value, ParamInt16):
                val_bytes = struct.pack("<hBB", int(value.value), 0, 0)
                typ = mrtmavlink.MAV_PARAM_TYPE_INT16
            elif isinstance(value, ParamUint32):
                val_bytes = struct.pack("<I", int(value.value))
                typ = mrtmavlink.MAV_PARAM_TYPE_UINT32
            elif isinstance(value, ParamInt32):
                val_bytes = struct.pack("<i", int(value.value))
                typ = mrtmavlink.MAV_PARAM_TYPE_INT32
            elif isinstance(value, ParamReal32):
                val_bytes = struct.pack("<f", value.value)
                typ = mrtmavlink.MAV_PARAM_TYPE_REAL32
            else:
                assert False, "unsupported type"

            self.param_result = None
            self.conn.param_set_send(
                self.system_id,
                mrtmavlink.MAV_COMP_ID_AUTOPILOT1,
                id.encode("utf-8"),
                struct.unpack("f", val_bytes)[0],
                typ,
            )
            try:
                async with self.param_cond:
                    await asyncio.wait_for(self.param_cond.wait(), RETRY_PERIOD_S)
            except asyncio.TimeoutError:
                logging.error("Timeout waiting for parameter ack")
                callback(None)
                return

            if self.param_result is not None:
                if self.param_result.name == id:
                    callback(self.param_result)
                else:
                    logging.warning(
                        f"Set parameter {id} but got {self.param_result.name}"
                    )
                    callback(None)
            else:
                logging.warning(f"Set parameter {id} but got None")
                callback(None)
        finally:
            self.is_requesting_parameters = False

    def set_motor_enablement(self, enable: bool):
        logging.info(f"Setting Motor Enablement to {enable}")
        self.set_parameter("MOTOR_ENABLE", ParamUint8(1 if enable else 0))

    async def _run(self):
        with self.is_started_cv:
            self.is_started = True
            self.is_started_cv.notify()

        self.loop = asyncio.get_running_loop()
        self.ack_cond = asyncio.Condition()
        self.param_cond = asyncio.Condition()
        await self.loop.create_datagram_endpoint(lambda: self, sock=self.sock)  # type: ignore

        while not self.is_done:
            logging.debug("Sending system time")
            self._send_system_time()
            await asyncio.sleep(1.0)

    def _thread(self):
        asyncio.run(self._run())

    def _send_system_time(self):
        self.conn.system_time_send(int(time.time() * 1e6), 0)

    def _send_param_request_list(self):
        self.conn.param_request_list_send(
            self.system_id,  # target_system
            mrtmavlink.MAV_COMP_ID_AUTOPILOT1,  # target_component
        )

    def request_parameters(self, callback: Callable[[List[ParamValue]], None]):
        self.loop.create_task(self._request_parameters(callback))

    async def _request_parameters(self, callback: Callable[[List[ParamValue]], None]):
        if self.system_id is None:
            logging.warning("System ID not set, not requesting parameters")
            return

        if self.is_requesting_parameters:
            logging.warning("Already requesting parameters")
            return

        RETRY_PERIOD_S = 0.25
        params: List[ParamValue] = []
        try:
            self.is_requesting_parameters = True
            self._send_param_request_list()
            while True:
                try:
                    self.param_result = None
                    async with self.param_cond:
                        await asyncio.wait_for(self.param_cond.wait(), RETRY_PERIOD_S)

                    if self.param_result is not None:
                        params.append(self.param_result)
                        if (
                            self.param_result.param_idx
                            == self.param_result.num_param - 1
                        ):
                            break
                except asyncio.TimeoutError:
                    logging.error("Timeout waiting for parameter")
                    break

        finally:
            self.is_requesting_parameters = False
            self.param_result = None

        callback(params)

    def send_autopilot_stop(self):
        self.loop.call_soon_threadsafe(lambda: self._send_autopilot_stop())

    def _send_autopilot_stop(self):
        if self.system_id is None:
            logging.warning("System ID not set, not sending STOP")
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
        self.loop.call_soon_threadsafe(
            lambda: self._send_waypoint(lat_deg, lon_deg, speed_mps)
        )

    def _send_waypoint(self, lat_deg: float, lon_deg: float, speed_mps: float):
        if self.system_id is None:
            logging.warning("System ID not set, not sending waypoint")
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

    def send_loiter(self, speed_mps: float, radius_m: float, duration_s: float):
        self.loop.call_soon_threadsafe(
            lambda: self._send_loiter(speed_mps, radius_m, duration_s)
        )

    def _send_loiter(self, speed_mps: float, radius_m: float, duration_s: float):
        if self.system_id is None:
            logging.warning("System ID not set, not sending loiter")
            return
        logging.info(
            f"Sending Loiter: speed {speed_mps} m/s, radius {radius_m} m, duration {duration_s} s"
        )

        self.conn.command_long_send(
            self.system_id,  # target_system
            mrtmavlink.MAV_COMP_ID_AUTOPILOT1,  # target_component
            mrtmavlink.MAV_CMD_WAYPOINT_USER_4,  # command
            0,  # confirmation
            duration_s,  # param1 - duration (seconds)
            speed_mps,  # param2 - speed (m/s)
            radius_m,  # param3 - radiums (m)
            0,  # param4 - unused
            float("nan"),  # param5 (latitude_deg * 1e7)
            float("nan"),  # param6 (longitude_deg * 1e7)
            0,  # param7 - unused
        )

    def send_protobuf_proxy(self, proto_id: int, buf: bytes):
        self.loop.call_soon_threadsafe(lambda: self._send_protobuf_proxy(proto_id, buf))

    def _send_protobuf_proxy(self, proto_id: int, buf: bytes):
        MAX_BUF_LEN = 251
        buf_len = len(buf)

        assert buf_len <= MAX_BUF_LEN
        logging.info(f"Sending Protobuf Proxy with ID {proto_id} and len {buf_len}")

        pad_len = MAX_BUF_LEN - buf_len
        buf += b"\0" * pad_len
        assert len(buf) == MAX_BUF_LEN

        self.conn.magothy_protobuf_proxy_send(proto_id, False, buf_len, buf)

    def send_mission(self, mission: mission.Mission):
        self.loop.create_task(self._send_mission(mission))

    async def _send_mission(self, mission: mission.Mission):
        if self.is_sending_mission:
            logging.warning("Already uploading mission")
            return

        if self.system_id is None:
            logging.warning("System ID not set, not uploading mission")
            return
        if len(mission.mission_items) == 0:
            logging.warning("Mission is empty, not uploading")
            return

        logging.info(f"Uploading Mission with {len(mission.mission_items)} items")

        try:
            self.is_sending_mission = True
            self.mission_session_id += 1

            mission_buf = mission.to_proto().SerializeToString()
            buf = zstandard.compress(mission_buf)

            UPLOAD_TIMEOUT_S = 0.5
            RETRY_PERIOD_S = 0.1
            max_retries = int(round(UPLOAD_TIMEOUT_S / RETRY_PERIOD_S))

            xfr = mrtmavlink.MAVLink_magothy_mission_transfer_message(
                target_system=self.system_id,
                session_id=self.mission_session_id,
                filename_index=0xFF,
                chunk_index=0,
                num_chunk=0,
                crc16=crc16(buf),
                payload_len=0,
                payload=bytes(),
            )
            max_payload_len = xfr.lengths[xfr.fieldnames.index("payload")]
            xfr.num_chunk = len(buf) // max_payload_len + (
                1 if len(buf) % max_payload_len > 0 else 0
            )

            for chunk_index in range(xfr.num_chunk):
                xfr.chunk_index = chunk_index
                xfr.payload_len = min(
                    max_payload_len, len(buf) - chunk_index * max_payload_len
                )
                begin = chunk_index * max_payload_len
                end = begin + xfr.payload_len
                xfr.payload = buf[begin:end]
                xfr.payload_len = len(xfr.payload)
                xfr.payload = xfr.payload + bytes(
                    max_payload_len - len(xfr.payload)
                )  # must pad to max_payload_len with zeros

                self.expected_mission_ack = ExpectedMissionAck(xfr)
                for retry_num in range(max_retries):
                    logging.debug(
                        f"Uploading mission chunk {chunk_index}/{xfr.num_chunk} ({retry_num}/{max_retries})"
                    )
                    self.ack_result = None
                    self.conn.send(xfr)

                    try:
                        async with self.ack_cond:
                            await asyncio.wait_for(self.ack_cond.wait(), RETRY_PERIOD_S)
                    except asyncio.TimeoutError:
                        continue

                    if self.ack_result is None:
                        continue

                    if self.ack_result == mrtmavlink.MAV_RESULT_ACCEPTED:
                        logging.debug(
                            f"Received ack for chunk {chunk_index + 1}/{xfr.num_chunk}"
                        )

                        break
                    else:
                        logging.error(
                            f"Received nack for chunk {chunk_index + 1}/{xfr.num_chunk} - code ({self.ack_result})"
                        )
                        return

            if self.ack_result is None:
                logging.error("Mission upload timed out")
                return

            logging.info("Mission upload successful, starting mission")

            uid = uuid.uuid4()
            uuid_params = struct.unpack("<ffff", uid.bytes)
            self.expected_cmd_uuid = struct.unpack("<iiii", uid.bytes)[0]

            # we have successfully sent the mission, now start it
            cmd = mrtmavlink.MAVLink_command_long_message(
                target_system=self.system_id,
                target_component=mrtmavlink.MAV_COMP_ID_AUTOPILOT1,
                command=mrtmavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                confirmation=1,
                param1=0,  # current index - always 0
                param2=0,
                param3=0,
                param4=uuid_params[0],
                param5=uuid_params[1],
                param6=uuid_params[2],
                param7=uuid_params[3],
            )

            for retry_num in range(max_retries):
                logging.debug(f"Set mission index: ({retry_num + 1}/{max_retries})")

                self.ack_result = None
                self.conn.send(cmd)

                try:
                    async with self.ack_cond:
                        await asyncio.wait_for(self.ack_cond.wait(), RETRY_PERIOD_S)
                except asyncio.TimeoutError:
                    continue

                if self.ack_result is None:
                    continue

                if self.ack_result == mrtmavlink.MAV_RESULT_ACCEPTED:
                    logging.info("Successfully set mission index")
                    break
                else:
                    logging.error(
                        f"Received nack for mission index - code ({self.ack_result})"
                    )
                    return

        finally:
            self.is_sending_mission = False

    def set_attitude(
        self,
        roll_rad: float = 0,
        pitch_rad: float = 0,
        yaw_rad: float = 0,
        roll_rate_rps: float = 0,
        pitch_rate_rps: float = 0,
        yaw_rate_rps: float = 0,
    ):
        self.loop.call_soon_threadsafe(
            lambda: self._set_attitude(
                roll_rad,
                pitch_rad,
                yaw_rad,
                roll_rate_rps,
                pitch_rate_rps,
                yaw_rate_rps,
            )
        )

    def _set_attitude(
        self,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        roll_rate_rps: float,
        pitch_rate_rps: float,
        yaw_rate_rps: float,
    ):
        self.conn.attitude_send(
            0,
            roll_rad,
            pitch_rad,
            yaw_rad,
            roll_rate_rps,
            pitch_rate_rps,
            yaw_rate_rps,
        )
