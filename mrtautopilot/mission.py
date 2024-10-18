from __future__ import annotations

from dataclasses import dataclass
from typing import List, Union, Dict
from enum import Enum
import math
import random

import mrtmavlink
import mrtproto.autopilot_pb2 as proto


@dataclass
class Position:
    lat_deg: float
    lon_deg: float


@dataclass
class Circle:
    origin: Position
    radius_m: float


@dataclass
class Polygon:
    vertices: List[Position]


Area = Union[Circle, Polygon]


@dataclass
class Waypoint:
    lat_deg: float
    lon_deg: float
    speed_mps: float


class DriftLoiter:
    def __init__(
        self,
        lat_deg: float,
        lon_deg: float,
        speed_mps: float,
        radius_m: float,
        duration_s: float = 60 * 60 * 24 * 365,
    ):
        self.lat_deg = lat_deg
        self.lon_deg = lon_deg
        self.speed_mps = speed_mps
        self.radius_m = radius_m
        self.duration_s = duration_s

    def __repr__(self) -> str:
        return f"DriftLoiter(lat_deg={self.lat_deg}, lon_deg={self.lon_deg}, speed_mps={self.speed_mps}, radius_m={self.radius_m}, duration_s={self.duration_s})"


class Orbit:
    def __init__(
        self,
        lat_deg: float,
        lon_deg: float,
        speed_mps: float,
        radius_m: float,
        is_clockwise: bool,
        duration_s: float = 60 * 60 * 24 * 365,
    ):
        self.lat_deg = lat_deg
        self.lon_deg = lon_deg
        self.speed_mps = speed_mps
        self.radius_m = radius_m
        self.is_clockwise = is_clockwise
        self.duration_s = duration_s

    def __repr__(self) -> str:
        return f"Orbit(lat_deg={self.lat_deg}, lon_deg={self.lon_deg}, speed_mps={self.speed_mps}, radius_m={self.radius_m}, is_clockwise={self.is_clockwise}, duration_s={self.duration_s})"


class Milling:
    def __init__(
        self,
        area: Area,
        speed_mps: float,
        duration_s: float = 60 * 60 * 24 * 365,
    ):
        assert type(area) in [Circle, Polygon]

        self.area = area
        self.speed_mps = speed_mps
        self.duration_s = duration_s

    def __repr__(self) -> str:
        return f"Milling(area={self.area}, speed_mps={self.speed_mps}, duration_s={self.duration_s})"


class FaultResponseType(Enum):
    Ignore = 0
    Halt = 1
    Loiter = 2
    GoRally = 3
    GoFirst = 4
    GoLast = 5
    GoLaunch = 6
    Custom = 7


class HealthId(Enum):
    HealthMonitor = 0x10000000
    GeoFence = 0x100000
    McuTimeout = 0x8000
    GpsTimeout = 0x20
    AhrsTimeout = 0x200000
    DepthTimeout = 0x400000
    JoystickTimeout = 0x10000
    LowFuel = 0x2000000
    OcTimeout = 0x8000000
    LowDiskSpace = 0x1000000


DEFAULT_FAULT_RESPONSE = {
    HealthId.HealthMonitor: FaultResponseType.Halt,
    HealthId.GeoFence: FaultResponseType.Ignore,
    HealthId.McuTimeout: FaultResponseType.Halt,
    HealthId.GpsTimeout: FaultResponseType.Halt,
    HealthId.AhrsTimeout: FaultResponseType.Halt,
    HealthId.DepthTimeout: FaultResponseType.Halt,
    HealthId.JoystickTimeout: FaultResponseType.Halt,
    HealthId.LowFuel: FaultResponseType.Halt,
    HealthId.OcTimeout: FaultResponseType.Halt,
    HealthId.LowDiskSpace: FaultResponseType.Ignore,
}


@dataclass
class FaultConfig:
    fault_responses: Dict[HealthId, FaultResponseType]
    loiter_radius_m: float
    loiter_duration_s: float
    response_speed_mps: float


class KeepInCirlce(Circle):
    pass


class KeepOutCircle(Circle):
    pass


class KeepInPolygon(Polygon):
    pass


class KeepOutPolygon(Polygon):
    pass


Fence = List[Union[KeepInCirlce, KeepOutCircle, KeepInPolygon, KeepOutPolygon]]


def _fix_indices(
    items: List[mrtmavlink.MAVLink_mission_item_int_message],
) -> List[mrtmavlink.MAVLink_mission_item_int_message]:
    for i, item in enumerate(items):
        item.seq = i
    return items


def _rally_items(mission: Mission) -> List[mrtmavlink.MAVLink_mission_item_int_message]:
    out: List[mrtmavlink.MAVLink_mission_item_int_message] = []
    for rally_point in mission.rally_points:
        out.append(
            mrtmavlink.MAVLink_mission_item_int_message(
                target_system=0,
                target_component=0,
                seq=0,
                frame=mrtmavlink.MAV_FRAME_MISSION,
                command=mrtmavlink.MAV_CMD_NAV_RALLY_POINT,
                mission_type=mrtmavlink.MAV_MISSION_TYPE_RALLY,
                current=0,
                autocontinue=0,
                param1=0,
                param2=0,
                param3=0,
                param4=0,
                x=int(rally_point.lat_deg * 1.0e7),
                y=int(rally_point.lon_deg * 1.0e7),
                z=0,
            )
        )
    return _fix_indices(out)


def make_circle(
    circle: Union[KeepInCirlce, KeepOutCircle],
) -> mrtmavlink.MAVLink_mission_item_int_message:
    assert type(circle) in [KeepInCirlce, KeepOutCircle]

    command = (
        mrtmavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
        if type(circle) is KeepInCirlce
        else mrtmavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
    )

    return mrtmavlink.MAVLink_mission_item_int_message(
        target_system=0,
        target_component=0,
        seq=0,
        frame=mrtmavlink.MAV_FRAME_MISSION,
        command=command,
        mission_type=mrtmavlink.MAV_MISSION_TYPE_FENCE,
        current=0,
        autocontinue=0,
        param1=circle.radius_m,
        param2=0,
        param3=0,
        param4=0,
        x=int(circle.origin.lat_deg * 1.0e7),
        y=int(circle.origin.lon_deg * 1.0e7),
        z=0,
    )


def make_polygon(
    polygon: Union[KeepInPolygon, KeepOutPolygon],
) -> List[mrtmavlink.MAVLink_mission_item_int_message]:
    assert type(polygon) in [KeepInPolygon, KeepOutPolygon]

    command = (
        mrtmavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        if type(polygon) is KeepInPolygon
        else mrtmavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
    )

    out: List[mrtmavlink.MAVLink_mission_item_int_message] = []
    for vertex in polygon.vertices:
        out.append(
            mrtmavlink.MAVLink_mission_item_int_message(
                target_system=0,
                target_component=0,
                seq=0,
                frame=mrtmavlink.MAV_FRAME_MISSION,
                command=command,
                mission_type=mrtmavlink.MAV_MISSION_TYPE_FENCE,
                current=0,
                autocontinue=0,
                param1=len(polygon.vertices),
                param2=0,
                param3=0,
                param4=0,
                x=int(vertex.lat_deg * 1.0e7),
                y=int(vertex.lon_deg * 1.0e7),
                z=0,
            )
        )

    return out


def _fence_items(mission: Mission) -> List[mrtmavlink.MAVLink_mission_item_int_message]:
    out: List[mrtmavlink.MAVLink_mission_item_int_message] = []

    for shape in mission.fence:
        if type(shape) is KeepInCirlce:
            out.append(make_circle(shape))
        elif type(shape) is KeepOutCircle:
            out.append(make_circle(shape))
        elif type(shape) is KeepInPolygon:
            out.extend(make_polygon(shape))
        elif type(shape) is KeepOutPolygon:
            out.extend(make_polygon(shape))
        else:
            raise ValueError(f"Unknown shape type: {type(shape)}")

    return _fix_indices(out)


def _speed_item(speed_mps: float) -> mrtmavlink.MAVLink_mission_item_int_message:
    return mrtmavlink.MAVLink_mission_item_int_message(
        target_system=0,
        target_component=0,
        seq=0,
        frame=mrtmavlink.MAV_FRAME_MISSION,
        command=mrtmavlink.MAV_CMD_DO_CHANGE_SPEED,
        mission_type=mrtmavlink.MAV_MISSION_TYPE_MISSION,
        current=0,
        autocontinue=0,
        param1=0,
        param2=speed_mps,
        param3=0,
        param4=0,
        x=0,
        y=0,
        z=0,
    )


def _speed_set_point(
    last: float,
    current: float,
    items: List[mrtmavlink.MAVLink_mission_item_int_message],
) -> float:
    if math.isfinite(current) and abs(last - current) > 0.1:
        items.append(_speed_item(current))
        return current
    return last


def _mission_items(
    mission: Mission,
) -> List[mrtmavlink.MAVLink_mission_item_int_message]:
    assert len(mission.mission_items) > 0

    out: List[mrtmavlink.MAVLink_mission_item_int_message] = []

    last_speed_mps = mission.mission_items[0].speed_mps

    # add initial speed
    out.append(_speed_item(last_speed_mps))

    # add fault config
    out.append(
        mrtmavlink.MAVLink_mission_item_int_message(
            target_system=0,
            target_component=0,
            seq=0,
            frame=mrtmavlink.MAV_FRAME_GLOBAL,
            command=mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE_PARAMS,
            mission_type=mrtmavlink.MAV_MISSION_TYPE_MISSION,
            current=0,
            autocontinue=0,
            param1=mission.fault_config.loiter_radius_m,
            param2=mission.fault_config.loiter_duration_s,
            param3=mission.fault_config.response_speed_mps,
            param4=0,
            x=0,
            y=0,
            z=0,
        )
    )

    # add fault responses
    fault_responses = dict(DEFAULT_FAULT_RESPONSE)
    for health_id, response in mission.fault_config.fault_responses.items():
        fault_responses[health_id] = response

    for health_id, response in fault_responses.items():
        out.append(
            mrtmavlink.MAVLink_mission_item_int_message(
                target_system=0,
                target_component=0,
                seq=0,
                frame=mrtmavlink.MAV_FRAME_MISSION,
                command=mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE,
                mission_type=mrtmavlink.MAV_MISSION_TYPE_MISSION,
                current=0,
                autocontinue=0,
                param1=health_id.value,
                param2=response.value,
                param3=0,
                param4=0,
                x=0,
                y=0,
                z=0,
            )
        )

    # add mission items
    for item in mission.mission_items:
        if type(item) is Waypoint:
            last_speed_mps = _speed_set_point(last_speed_mps, item.speed_mps, out)
            out.append(
                mrtmavlink.MAVLink_mission_item_int_message(
                    target_system=0,
                    target_component=0,
                    seq=0,
                    frame=mrtmavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command=mrtmavlink.MAV_CMD_NAV_WAYPOINT,
                    mission_type=mrtmavlink.MAV_MISSION_TYPE_MISSION,
                    current=0,
                    autocontinue=1,
                    param1=0,
                    param2=0,
                    param3=0,
                    param4=math.nan,
                    x=int(item.lat_deg * 1.0e7),
                    y=int(item.lon_deg * 1.0e7),
                    z=0,
                )
            )
        elif type(item) is DriftLoiter:
            last_speed_mps = _speed_set_point(last_speed_mps, item.speed_mps, [])
            out.append(
                mrtmavlink.MAVLink_mission_item_int_message(
                    target_system=0,
                    target_component=0,
                    seq=0,
                    frame=mrtmavlink.MAV_FRAME_GLOBAL,
                    command=mrtmavlink.MAV_CMD_WAYPOINT_USER_4,
                    mission_type=mrtmavlink.MAV_MISSION_TYPE_MISSION,
                    current=0,
                    autocontinue=1,
                    param1=item.duration_s,
                    param2=last_speed_mps,
                    param3=item.radius_m,
                    param4=0,
                    x=int(item.lat_deg * 1.0e7),
                    y=int(item.lon_deg * 1.0e7),
                    z=0,
                )
            )
        elif type(item) is Orbit:
            last_speed_mps = _speed_set_point(last_speed_mps, item.speed_mps, [])
            out.append(
                mrtmavlink.MAVLink_mission_item_int_message(
                    target_system=0,
                    target_component=0,
                    seq=0,
                    frame=mrtmavlink.MAV_FRAME_GLOBAL,
                    command=mrtmavlink.MAV_CMD_WAYPOINT_USER_3,
                    mission_type=mrtmavlink.MAV_MISSION_TYPE_MISSION,
                    current=0,
                    autocontinue=1,
                    param1=item.duration_s,
                    param2=last_speed_mps,
                    param3=item.radius_m,
                    param4=0 if item.is_clockwise else 1,
                    x=int(item.lat_deg * 1.0e7),
                    y=int(item.lon_deg * 1.0e7),
                    z=0,
                )
            )
        elif type(item) is Milling:
            last_speed_mps = _speed_set_point(last_speed_mps, item.speed_mps, [])

            assert type(item.area) in [Circle, Polygon]

            a = item.area
            id = random.random()

            out.append(
                mrtmavlink.MAVLink_mission_item_int_message(
                    target_system=0,
                    target_component=0,
                    seq=0,
                    frame=mrtmavlink.MAV_FRAME_GLOBAL,
                    command=mrtmavlink.MAV_CMD_DO_MILLING,
                    mission_type=mrtmavlink.MAV_MISSION_TYPE_MISSION,
                    current=0,
                    autocontinue=1,
                    param1=item.duration_s,
                    param2=last_speed_mps,
                    param3=a.radius_m if type(a) is Circle else math.nan,
                    param4=0 if type(a) is Circle else id,
                    x=int(a.origin.lat_deg * 1.0e7) if type(a) is Circle else 0,
                    y=int(a.origin.lon_deg * 1.0e7) if type(a) is Circle else 0,
                    z=0,
                )
            )

            if type(item.area) is Polygon:
                assert len(item.area.vertices) >= 3
                for i, vertex in enumerate(item.area.vertices):
                    out.append(
                        mrtmavlink.MAVLink_mission_item_int_message(
                            target_system=0,
                            target_component=0,
                            seq=0,
                            frame=mrtmavlink.MAV_FRAME_GLOBAL,
                            command=mrtmavlink.MAV_CMD_DO_SET_MILLING_POLYGON_VERTEX,
                            mission_type=mrtmavlink.MAV_MISSION_TYPE_MISSION,
                            current=0,
                            autocontinue=0,
                            param1=i,
                            param2=len(item.area.vertices) - 1,
                            param3=0,
                            param4=id,
                            x=int(vertex.lat_deg * 1.0e7),
                            y=int(vertex.lon_deg * 1.0e7),
                            z=0,
                        )
                    )

    return _fix_indices(out)


@dataclass
class MavlinkMission:
    mission_items: List[mrtmavlink.MAVLink_mission_item_int_message]
    fence_items: List[mrtmavlink.MAVLink_mission_item_int_message]
    rally_items: List[mrtmavlink.MAVLink_mission_item_int_message]


MissionItem = Union[Waypoint, DriftLoiter, Orbit, Milling]
MissionItems = List[MissionItem]


def _to_proto(
    mav: mrtmavlink.MAVLink_mission_item_int_message,
) -> proto.MavlinkMissionItemInt:
    p = proto.MavlinkMissionItemInt()
    p.target_system = mav.target_system
    p.target_component = mav.target_component
    p.seq = mav.seq
    p.frame = mav.frame
    p.command = mav.command
    p.mission_type = mav.mission_type
    p.current = mav.current == 1
    p.autocontinue = mav.autocontinue == 1
    p.param1 = mav.param1
    p.param2 = mav.param2
    p.param3 = mav.param3
    p.param4 = mav.param4
    p.x = mav.x
    p.y = mav.y
    p.z = mav.z
    return p


@dataclass
class Mission:
    mission_items: MissionItems
    fence: Fence
    rally_points: List[Position]
    fault_config: FaultConfig

    def to_mavlink(self) -> MavlinkMission:
        assert type(self.fault_config) is FaultConfig
        assert type(self.rally_points) is list
        assert type(self.fence) is list
        assert type(self.mission_items) is list

        return MavlinkMission(
            mission_items=_mission_items(self),
            fence_items=_fence_items(self),
            rally_items=_rally_items(self),
        )

    def to_proto(self) -> proto.MavlinkMission:
        mav = self.to_mavlink()
        p = proto.MavlinkMission()
        for item in mav.mission_items:
            p.mission_items.append(_to_proto(item))
        for item in mav.fence_items:
            p.fence_items.append(_to_proto(item))
        for item in mav.rally_items:
            p.rally_items.append(_to_proto(item))
        return p
