import datetime
import unittest

import mrtmavlink
import mrtautopilot


class TestGps2Raw(unittest.TestCase):
    def message(self, altitude=0x7FFFFFFF, **fields):
        values = dict(
            time_usec=0,
            fix_type=mrtmavlink.GPS_FIX_TYPE_NO_FIX,
            lat=0x7FFFFFFF,
            lon=0x7FFFFFFF,
            alt=altitude,
            eph=0xFFFF,
            epv=0xFFFF,
            vel=0xFFFF,
            cog=0xFFFF,
            satellites_visible=0xFF,
            dgps_numch=0,
            dgps_age=0,
        )
        values.update(fields)
        message = mrtmavlink.MAVLink_gps2_raw_message(**values)
        wire = message.pack(mrtmavlink.MAVLink(None))
        return mrtmavlink.MAVLink(None).parse_char(wire)

    def test_unavailable_fields(self):
        gps = mrtautopilot.Gps2Raw(self.message())
        self.assertEqual(gps.fix_type, mrtautopilot.GpsFixType.NoFix)
        self.assertEqual(gps.timestamp, datetime.datetime(1970, 1, 1, tzinfo=datetime.timezone.utc))
        for field in ("latitude_deg", "longitude_deg", "altitude_m", "hdop",
                      "vdop", "speed_mps", "course_deg", "num_satellites"):
            with self.subTest(field=field):
                self.assertIsNone(getattr(gps, field))

    def test_altitude_conversion(self):
        for altitude in (0, -1250, 12345, 0x7FFFFFF):
            with self.subTest(altitude=altitude):
                gps = mrtautopilot.Gps2Raw(self.message(altitude))
                self.assertEqual(gps.altitude_m, altitude / 1000)

    def test_all_exposed_units_and_zero_values(self):
        gps = mrtautopilot.Gps2Raw(self.message(
            altitude=-1250, time_usec=1700000000123456,
            fix_type=mrtmavlink.GPS_FIX_TYPE_3D_FIX,
            lat=380000000, lon=-750000000, eph=123, epv=456,
            vel=514, cog=27000, satellites_visible=8,
        ))
        self.assertEqual(gps.timestamp, datetime.datetime(
            2023, 11, 14, 22, 13, 20, 123456, tzinfo=datetime.timezone.utc))
        self.assertEqual(gps.fix_type, mrtautopilot.GpsFixType.Fix3d)
        self.assertEqual(gps.latitude_deg, 38.0)
        self.assertEqual(gps.longitude_deg, -75.0)
        self.assertEqual(gps.altitude_m, -1.25)
        self.assertEqual(gps.hdop, 1.23)
        self.assertEqual(gps.vdop, 4.56)
        self.assertEqual(gps.speed_mps, 5.14)
        self.assertEqual(gps.course_deg, 270.0)
        self.assertEqual(gps.num_satellites, 8)
        zero = mrtautopilot.Gps2Raw(self.message(
            altitude=0, lat=0, lon=0, eph=0, epv=0, vel=0, cog=0, satellites_visible=0))
        for field in ("latitude_deg", "longitude_deg", "altitude_m", "hdop",
                      "vdop", "speed_mps", "course_deg", "num_satellites"):
            with self.subTest(field=field):
                self.assertEqual(getattr(zero, field), 0)

    def test_fix_types_and_coordinate_boundaries(self):
        for fix_type in range(9):
            with self.subTest(fix_type=fix_type):
                gps = mrtautopilot.Gps2Raw(self.message(fix_type=fix_type))
                self.assertEqual(gps.fix_type.value, fix_type)
        for sign in (-1, 1):
            gps = mrtautopilot.Gps2Raw(self.message(lat=sign * 900000000, lon=sign * 1800000000))
            self.assertEqual(gps.latitude_deg, sign * 90)
            self.assertEqual(gps.longitude_deg, sign * 180)

    def test_wire_decoder_preserves_dgps_and_yaw(self):
        # These fields are available on raw messages, not exposed by Gps2Raw.
        for yaw in (0, 0xFFFF, 36000, 9000):
            with self.subTest(yaw=yaw):
                message = self.message(dgps_numch=8, dgps_age=1500, yaw=yaw)
                self.assertEqual(message.dgps_numch, 8)
                self.assertEqual(message.dgps_age, 1500)
                self.assertEqual(message.yaw, yaw)
