import unittest
import math

from mrtautopilot import mission
import mrtmavlink


class TestMission(unittest.TestCase):
    def test_small_mission(self):
        cmd = mission.Mission(
            fault_config=mission.FaultConfig(
                fault_responses={},
                loiter_radius_m=30,
                loiter_duration_s=365 * 24 * 60 * 60,
                response_speed_mps=2,
            ),
            fence=[],
            rally_points=[],
            mission_items=[
                mission.Waypoint(lat_deg=38.8, lon_deg=-77.4, speed_mps=3),
                mission.Waypoint(lat_deg=38.7, lon_deg=-77.3, speed_mps=6),
                mission.Waypoint(lat_deg=38.6, lon_deg=-77.2, speed_mps=10),
                mission.DriftLoiter(
                    lat_deg=38.5,
                    lon_deg=-77.1,
                    speed_mps=3,
                    radius_m=50,
                    duration_s=360000,
                ),
            ],
        )

        mav = cmd.to_mavlink()

        ### check rally items ###
        self.assertEqual(len(mav.rally_items), 0)

        ### check fence items ###
        self.assertEqual(len(mav.fence_items), 0)

        ### check mission items ###
        self.assertEqual(len(mav.mission_items), 18)

        # set initial speed
        i = mav.mission_items[0]
        self.assertEqual(i.seq, 0)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_CHANGE_SPEED)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param2, 3)

        # set fault parameters
        i = mav.mission_items[1]
        self.assertEqual(i.seq, 1)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE_PARAMS)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, 30)
        self.assertEqual(i.param2, 365 * 24 * 60 * 60)
        self.assertEqual(i.param3, 2)

        # fault responses
        i = mav.mission_items[2]
        self.assertEqual(i.seq, 2)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.HealthMonitor.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[3]
        self.assertEqual(i.seq, 3)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.GeoFence.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Ignore.value)

        i = mav.mission_items[4]
        self.assertEqual(i.seq, 4)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.McuTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[5]
        self.assertEqual(i.seq, 5)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.GpsTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[6]
        self.assertEqual(i.seq, 6)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.AhrsTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[7]
        self.assertEqual(i.seq, 7)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.DepthTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[8]
        self.assertEqual(i.seq, 8)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.JoystickTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[9]
        self.assertEqual(i.seq, 9)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.LowFuel.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[10]
        self.assertEqual(i.seq, 10)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.OcTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[11]
        self.assertEqual(i.seq, 11)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.LowDiskSpace.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Ignore.value)

        # waypoint #1
        i = mav.mission_items[12]
        self.assertEqual(i.seq, 12)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertTrue(math.isnan(i.param4))
        self.assertEqual(i.x, 38.8e7)
        self.assertEqual(i.y, -77.4e7)

        # set speed
        i = mav.mission_items[13]
        self.assertEqual(i.seq, 13)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_CHANGE_SPEED)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param2, 6)

        # waypoint #2
        i = mav.mission_items[14]
        self.assertEqual(i.seq, 14)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertTrue(math.isnan(i.param4))
        self.assertEqual(i.x, 38.7e7)
        self.assertEqual(i.y, -77.3e7)

        # set speed
        i = mav.mission_items[15]
        self.assertEqual(i.seq, 15)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_CHANGE_SPEED)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param2, 10)

        # waypoint #3
        i = mav.mission_items[16]
        self.assertEqual(i.seq, 16)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertTrue(math.isnan(i.param4))
        self.assertEqual(i.x, 38.6e7)
        self.assertEqual(i.y, -77.2e7)

        # drift loiter
        i = mav.mission_items[17]
        self.assertEqual(i.seq, 17)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_WAYPOINT_USER_4)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertEqual(i.param1, 360000)
        self.assertEqual(i.param2, 3)
        self.assertEqual(i.param3, 50)
        self.assertEqual(i.x, 38.5e7)
        self.assertEqual(i.y, -77.1e7)

    def test_full_mission(self):
        cmd = mission.Mission(
            fault_config=mission.FaultConfig(
                fault_responses={
                    mission.HealthId.LowFuel: mission.FaultResponseType.GoRally,
                    mission.HealthId.OcTimeout: mission.FaultResponseType.Loiter,
                },
                loiter_radius_m=30,
                loiter_duration_s=365 * 24 * 60 * 60,
                response_speed_mps=2,
            ),
            fence=[
                mission.KeepInCirlce(
                    origin=mission.Position(lat_deg=5, lon_deg=-6), radius_m=25
                ),
                mission.KeepOutPolygon(
                    [
                        mission.Position(lat_deg=7, lon_deg=-8),
                        mission.Position(lat_deg=9, lon_deg=-10),
                        mission.Position(lat_deg=11, lon_deg=-12),
                    ]
                ),
                mission.KeepInPolygon(
                    [
                        mission.Position(lat_deg=13, lon_deg=-14),
                        mission.Position(lat_deg=15, lon_deg=-16),
                        mission.Position(lat_deg=17, lon_deg=-18),
                        mission.Position(lat_deg=19, lon_deg=-20),
                    ]
                ),
                mission.KeepOutCircle(
                    origin=mission.Position(lat_deg=21, lon_deg=-22), radius_m=42
                ),
            ],
            rally_points=[
                mission.Position(lat_deg=1, lon_deg=-2),
                mission.Position(lat_deg=3, lon_deg=-4),
            ],
            mission_items=[
                mission.Waypoint(lat_deg=23, lon_deg=-24, speed_mps=612),
                mission.DriftLoiter(
                    lat_deg=25,
                    lon_deg=-26,
                    speed_mps=611,
                    radius_m=222,
                    duration_s=1212,
                ),
                mission.Orbit(
                    lat_deg=27,
                    lon_deg=-28,
                    speed_mps=623,
                    radius_m=232,
                    is_clockwise=True,
                    duration_s=1225,
                ),
                mission.Milling(
                    area=mission.Circle(
                        origin=mission.Position(lat_deg=29, lon_deg=-30),
                        radius_m=242,
                    ),
                    speed_mps=37,
                    duration_s=1337,
                ),
                mission.Milling(
                    area=mission.Polygon(
                        [
                            mission.Position(lat_deg=31, lon_deg=-32),
                            mission.Position(lat_deg=33, lon_deg=-34),
                            mission.Position(lat_deg=35, lon_deg=-36),
                        ]
                    ),
                    speed_mps=512,
                    duration_s=45,
                ),
                mission.Waypoint(lat_deg=37, lon_deg=-38, speed_mps=39),
            ],
        )

        mav = cmd.to_mavlink()

        ### check rally items ###
        self.assertEqual(len(mav.rally_items), 2)

        i = mav.rally_items[0]
        self.assertEqual(i.seq, 0)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_NAV_RALLY_POINT)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_RALLY)
        self.assertEqual(i.x, 1e7)
        self.assertEqual(i.y, -2e7)

        i = mav.rally_items[1]
        self.assertEqual(i.seq, 1)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_NAV_RALLY_POINT)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_RALLY)
        self.assertEqual(i.x, 3e7)
        self.assertEqual(i.y, -4e7)

        ### check fence items ###
        self.assertEqual(len(mav.fence_items), 9)

        # keep in circle
        i = mav.fence_items[0]
        self.assertEqual(i.seq, 0)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_FENCE)
        self.assertEqual(i.param1, 25)
        self.assertEqual(i.x, 5e7)
        self.assertEqual(i.y, -6e7)

        # keep out polygon
        i = mav.fence_items[1]
        self.assertEqual(i.seq, 1)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(
            i.command, mrtmavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        )
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_FENCE)
        self.assertEqual(i.param1, 3)
        self.assertEqual(i.x, 7e7)
        self.assertEqual(i.y, -8e7)

        i = mav.fence_items[2]
        self.assertEqual(i.seq, 2)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(
            i.command, mrtmavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        )
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_FENCE)
        self.assertEqual(i.param1, 3)
        self.assertEqual(i.x, 9e7)
        self.assertEqual(i.y, -10e7)

        i = mav.fence_items[3]
        self.assertEqual(i.seq, 3)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(
            i.command, mrtmavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        )
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_FENCE)
        self.assertEqual(i.param1, 3)
        self.assertEqual(i.x, 11e7)
        self.assertEqual(i.y, -12e7)

        # keep in polygon
        i = mav.fence_items[4]
        self.assertEqual(i.seq, 4)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(
            i.command, mrtmavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        )
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_FENCE)
        self.assertEqual(i.param1, 4)
        self.assertEqual(i.x, 13e7)
        self.assertEqual(i.y, -14e7)

        i = mav.fence_items[5]
        self.assertEqual(i.seq, 5)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(
            i.command, mrtmavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        )
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_FENCE)
        self.assertEqual(i.param1, 4)
        self.assertEqual(i.x, 15e7)
        self.assertEqual(i.y, -16e7)

        i = mav.fence_items[6]
        self.assertEqual(i.seq, 6)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(
            i.command, mrtmavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        )
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_FENCE)
        self.assertEqual(i.param1, 4)
        self.assertEqual(i.x, 17e7)
        self.assertEqual(i.y, -18e7)

        i = mav.fence_items[7]
        self.assertEqual(i.seq, 7)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(
            i.command, mrtmavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        )
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_FENCE)
        self.assertEqual(i.param1, 4)
        self.assertEqual(i.x, 19e7)
        self.assertEqual(i.y, -20e7)

        # keep out circle
        i = mav.fence_items[8]
        self.assertEqual(i.seq, 8)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_FENCE)
        self.assertEqual(i.param1, 42)
        self.assertEqual(i.x, 21e7)
        self.assertEqual(i.y, -22e7)

        ### check mission items ###
        self.assertEqual(len(mav.mission_items), 22)

        # set initial speed
        i = mav.mission_items[0]
        self.assertEqual(i.seq, 0)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_CHANGE_SPEED)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param2, 612)

        # set fault parameters
        i = mav.mission_items[1]
        self.assertEqual(i.seq, 1)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE_PARAMS)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, 30)
        self.assertEqual(i.param2, 365 * 24 * 60 * 60)
        self.assertEqual(i.param3, 2)

        # fault responses
        i = mav.mission_items[2]
        self.assertEqual(i.seq, 2)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.HealthMonitor.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[3]
        self.assertEqual(i.seq, 3)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.GeoFence.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Ignore.value)

        i = mav.mission_items[4]
        self.assertEqual(i.seq, 4)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.McuTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[5]
        self.assertEqual(i.seq, 5)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.GpsTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[6]
        self.assertEqual(i.seq, 6)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.AhrsTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[7]
        self.assertEqual(i.seq, 7)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.DepthTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[8]
        self.assertEqual(i.seq, 8)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.JoystickTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Halt.value)

        i = mav.mission_items[9]
        self.assertEqual(i.seq, 9)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.LowFuel.value)
        self.assertEqual(i.param2, mission.FaultResponseType.GoRally.value)

        i = mav.mission_items[10]
        self.assertEqual(i.seq, 10)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.OcTimeout.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Loiter.value)

        i = mav.mission_items[11]
        self.assertEqual(i.seq, 11)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_FAULT_RESPONSE)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, mission.HealthId.LowDiskSpace.value)
        self.assertEqual(i.param2, mission.FaultResponseType.Ignore.value)

        # waypoint
        i = mav.mission_items[12]
        self.assertEqual(i.seq, 12)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertTrue(math.isnan(i.param4))
        self.assertEqual(i.x, 23e7)
        self.assertEqual(i.y, -24e7)

        # drift loiter
        i = mav.mission_items[13]
        self.assertEqual(i.seq, 13)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_WAYPOINT_USER_4)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertEqual(i.param1, 1212)
        self.assertEqual(i.param2, 611)
        self.assertEqual(i.param3, 222)
        self.assertEqual(i.x, 25e7)
        self.assertEqual(i.y, -26e7)

        # orbit
        i = mav.mission_items[14]
        self.assertEqual(i.seq, 14)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_WAYPOINT_USER_3)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertEqual(i.param1, 1225)
        self.assertEqual(i.param2, 623)
        self.assertEqual(i.param3, 232)
        self.assertEqual(i.param4, 0)
        self.assertEqual(i.x, 27e7)
        self.assertEqual(i.y, -28e7)

        # milling circle
        i = mav.mission_items[15]
        self.assertEqual(i.seq, 15)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_MILLING)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertEqual(i.param1, 1337)
        self.assertEqual(i.param2, 37)
        self.assertEqual(i.param3, 242)
        self.assertEqual(i.x, 29e7)
        self.assertEqual(i.y, -30e7)

        # milling polygon
        i = mav.mission_items[16]
        self.assertEqual(i.seq, 16)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_MILLING)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertEqual(i.param1, 45)
        self.assertEqual(i.param2, 512)
        self.assertTrue(math.isnan(i.param3))
        milling_id = i.param4

        i = mav.mission_items[17]
        self.assertEqual(i.seq, 17)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_MILLING_POLYGON_VERTEX)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, 0)
        self.assertEqual(i.param2, 2)
        self.assertEqual(i.param4, milling_id)
        self.assertEqual(i.x, 31e7)
        self.assertEqual(i.y, -32e7)

        i = mav.mission_items[18]
        self.assertEqual(i.seq, 18)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_MILLING_POLYGON_VERTEX)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, 1)
        self.assertEqual(i.param2, 2)
        self.assertEqual(i.param4, milling_id)
        self.assertEqual(i.x, 33e7)
        self.assertEqual(i.y, -34e7)

        i = mav.mission_items[19]
        self.assertEqual(i.seq, 19)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_SET_MILLING_POLYGON_VERTEX)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param1, 2)
        self.assertEqual(i.param2, 2)
        self.assertEqual(i.param4, milling_id)
        self.assertEqual(i.x, 35e7)
        self.assertEqual(i.y, -36e7)

        # set speed for next waypoint
        i = mav.mission_items[20]
        self.assertEqual(i.seq, 20)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_MISSION)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_DO_CHANGE_SPEED)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.param2, 39)

        # final waypoint
        i = mav.mission_items[21]
        self.assertEqual(i.seq, 21)
        self.assertEqual(i.frame, mrtmavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        self.assertEqual(i.command, mrtmavlink.MAV_CMD_NAV_WAYPOINT)
        self.assertEqual(i.mission_type, mrtmavlink.MAV_MISSION_TYPE_MISSION)
        self.assertEqual(i.autocontinue, 1)
        self.assertTrue(math.isnan(i.param4))
        self.assertEqual(i.x, 37e7)
        self.assertEqual(i.y, -38e7)


if __name__ == "__main__":
    unittest.main()
