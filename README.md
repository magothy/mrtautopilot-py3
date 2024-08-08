# MRT Autopilot Python3 API

This repository contains the Python3 API for the MRT Autopilot.

## Installation

```
pip3 install git+https://github.com/magothy/mrtautopilot-py3@v4
```

## Usage

Example usage:

```python
#!/usr/bin/env python3

import logging
import selectors

import mrtautopilot

# example for send_protobuf_proxy
from mrtproto import Magothy_pb2
PROTO_ID = 0x5433  # defined by protocol


def main():
    mavlink = mrtautopilot.MavlinkThread()

    proto = Magothy_pb2.VehicleData()
    proto.position.latitude_deg = 38.3
    proto.position.longitude_deg = -77.1

    def got_low_bandwidth():
        d: mrtautopilot.LowBandwidth = mavlink.low_bandwidth_queue.get_nowait()
        logging.info(f"Got Vehicle Data: {d.latitude_deg}, {d.longitude_deg}")

    try:
        sel = selectors.DefaultSelector()
        sel.register(
            mavlink.low_bandwidth_queue_fileobj,
            selectors.EVENT_READ,
            got_low_bandwidth,
        )

        count = 0
        mavlink.start()
        while True:
            mavlink.send_heartbeat()

            count += 1

            if count == 5:
                mavlink.send_protobuf_proxy(PROTO_ID, proto.SerializeToString())

            if count == 10:
                mavlink.set_motor_enablement(True)
                mavlink.send_waypoint(38.3, -77.1, 3.0)

            if count == 20:
                mavlink.set_motor_enablement(False)
                mavlink.send_autopilot_stop()

            for key, mask in sel.select():
                key.data()

    finally:
        mavlink.stop()


def setup_logging():
    logging.basicConfig(
        format="%(asctime)s.%(msecs)03d | %(levelname)8s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)


if __name__ == "__main__":
    setup_logging()
    try:
        main()
    except KeyboardInterrupt:
        pass
```
