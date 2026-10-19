#!/usr/bin/env python3
"""Publishes a synthetic sensor_msgs/Image on /test_image, no camera or extra
ROS2 packages needed - just rclpy + sensor_msgs, both already required by
osiris_agent itself. Exists to exercise the JPEG re-encode path
(payload_encoding.py's _reencode_image_jpeg) end to end without real hardware.

Usage:
    python3 fake_camera.py                      # 320x240 rgb8 @ 5Hz (defaults)
    python3 fake_camera.py --width 1920 --height 1080 --hz 2 --encoding bgr8
    python3 fake_camera.py --topic /camera/image_raw

What to actually check once this is running and osiris_agent/the gateway are up:
  - Subscribe to the topic from Graph - it should show up as
    sensor_msgs/msg/Image with no crash or warning in the agent's log.
  - The agent's own log should NOT print an "encoding '...' not supported"
    warning (that would mean the encoding this script picked isn't in
    IMAGE_ENCODING_PIL_MODES - only matters if you pass --encoding something
    other than rgb8/bgr8/mono8/rgba8/bgra8).
  - Whatever ends up persisted/broadcast for this topic's data.data should be
    a much smaller byte array than width*height*channels would predict (that's
    the JPEG re-encode actually working, not just the binary framing) - e.g.
    320x240 rgb8 raw is 230,400 bytes; JPEG'd at quality 70 a solid-color or
    gradient test pattern like this one should be well under 5,000.
  - Nothing else on the connection should hang or drop - the gateway
    shouldn't report a binary frame arriving without its JSON header, and no
    other topic's messages should misbehave.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

ENCODING_CHANNELS = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, 'rgba8': 4, 'bgra8': 4}


def build_frame(width: int, height: int, encoding: str, t: float) -> bytes:
    """A cheap animated gradient (not a static solid color) so successive
    frames actually differ - a solid color would compress to almost nothing
    regardless of whether JPEG encoding is working correctly, and wouldn't
    catch a bug in per-frame handling."""
    channels = ENCODING_CHANNELS[encoding]
    phase = int((math.sin(t) + 1) / 2 * 255)
    row = bytearray()
    for x in range(width):
        v = (x * 255 // max(width - 1, 1) + phase) % 255
        if channels == 1:
            row += bytes([v])
        elif channels == 3:
            row += bytes([v, 255 - v, phase])
        else:  # 4 channels
            row += bytes([v, 255 - v, phase, 255])
    return bytes(row) * height


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/test_image')
    ap.add_argument('--width', type=int, default=320)
    ap.add_argument('--height', type=int, default=240)
    ap.add_argument('--hz', type=float, default=5.0)
    ap.add_argument('--encoding', default='rgb8', choices=list(ENCODING_CHANNELS))
    args = ap.parse_args()

    rclpy.init()
    node = Node('fake_camera')
    pub = node.create_publisher(Image, args.topic, 10)
    channels = ENCODING_CHANNELS[args.encoding]
    step = channels * args.width
    frame_count = 0
    print(f"Publishing {args.width}x{args.height} {args.encoding} on {args.topic} @ {args.hz}Hz "
          f"({args.width * args.height * channels} bytes/frame raw) - Ctrl+C to stop")

    def publish():
        nonlocal frame_count
        msg = Image()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'fake_camera'
        msg.height = args.height
        msg.width = args.width
        msg.encoding = args.encoding
        msg.is_bigendian = 0
        msg.step = step
        msg.data = build_frame(args.width, args.height, args.encoding, time.time())
        pub.publish(msg)
        frame_count += 1
        if frame_count % max(int(args.hz), 1) == 0:
            print(f"  published frame {frame_count}")

    node.create_timer(1.0 / args.hz, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
