import unittest

import mrtmavlink
import mrtautopilot


class TestGps2Raw(unittest.TestCase):
    def message(self, altitude=0x7FFFFFFF):
        return mrtmavlink.MAVLink_gps2_raw_message(
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

    def test_unavailable_fields(self):
        gps = mrtautopilot.Gps2Raw(self.message())
        self.assertEqual(gps.fix_type, mrtautopilot.GpsFixType.NoFix)
        for field in ("latitude_deg", "longitude_deg", "altitude_m", "hdop",
                      "vdop", "speed_mps", "course_deg", "num_satellites"):
            with self.subTest(field=field):
                self.assertIsNone(getattr(gps, field))

    def test_altitude_conversion(self):
        for altitude in (0, -1250, 12345, 0x7FFFFFF):
            with self.subTest(altitude=altitude):
                gps = mrtautopilot.Gps2Raw(self.message(altitude))
                self.assertEqual(gps.altitude_m, altitude / 1000)
