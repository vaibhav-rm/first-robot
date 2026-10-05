"""Crop a saved occupancy grid to a fixed window around the start pose.

SLAM Toolbox grows its occupancy grid without bound as the pose graph
expands, so the saved map's origin and dimensions depend on how far the
graph drifted rather than on the area that was actually explored. A run
that stayed inside x=[-2.7, 2.7] still produced a map spanning
x=-10.8m, because drift in the pose graph is what expands the grid.

This crops the map to a fixed window centred on the world origin, which is
where the robot spawns. The result has identical dimensions on every run
regardless of drift, so the grid size in RViz is stable and predictable.

Usage:
    ros2 run myrobot_controller crop_map --ros-args \
        -p map_in:=/path/to/raw_map.pgm -p map_out:=/path/to/cropped.pgm
"""
import struct
import sys

import numpy as np
import yaml

from rclpy.node import Node

# Fixed window. The arena perimeter walls have inner faces at x=+-3.1 and
# y=-3.5..2.7; this window sits just inside them so the whole explored
# region is retained and the fixed border lands on mapped wall rather than
# on unknown padding.
WINDOW_X = (-3.1, 3.1)
WINDOW_Y = (-3.5, 2.7)


def read_pgm(path):
    """Read a binary (P5) or ASCII (P2) PGM into a 2D uint8 array."""
    with open(path, 'rb') as f:
        data = f.read()

    # Strip comments; PGM allows them between header tokens.
    def tokens():
        for part in data.split(b'#')[0].split():
            yield part

    it = tokens()
    magic = next(it)
    width = int(next(it))
    height = int(next(it))
    maxval = int(next(it))

    if magic == b'P5':
        # Binary: pixel data starts right after the single whitespace
        # character following maxval.
        header_end = data.index(b'\n', data.index(str(maxval).encode()))
        offset = header_end + 1
        while data[offset:offset + 1].isspace():
            offset += 1
        px = np.frombuffer(data, dtype=np.uint8, count=width * height,
                           offset=offset)
    elif magic == b'P2':
        px = np.array([int(t) for t in it], dtype=np.uint8)
    else:
        raise ValueError(f'unsupported PGM magic {magic!r}')

    return px.reshape(height, width)


def write_pgm(path, arr):
    height, width = arr.shape
    header = f'P5\n{width} {height}\n255\n'.encode()
    with open(path, 'wb') as f:
        f.write(header)
        f.write(arr.astype(np.uint8).tobytes())


class CropMap(Node):
    def __init__(self):
        super().__init__('crop_map')
        self.declare_parameter('map_in', '')
        self.declare_parameter('map_out', '')

        map_in = self.get_parameter('map_in').value
        map_out = self.get_parameter('map_out').value
        if not map_in or not map_out:
            self.get_logger().error('map_in and map_out are required')
            sys.exit(1)

        yaml_path = map_in.rsplit('.', 1)[0] + '.yaml'
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)

        grid = read_pgm(map_in)
        res = float(meta['resolution'])
        ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
        height, width = grid.shape

        def col(world_x):
            return int(np.floor((world_x - ox) / res))

        def row(world_y):
            # Row 0 is the top of the grid, i.e. the highest world y, so
            # rows count down as y increases.
            return int(round((oy + height * res - world_y) / res))

        x0, x1 = col(WINDOW_X[0]), col(WINDOW_X[1])
        y0, y1 = row(WINDOW_Y[1]), row(WINDOW_Y[0])

        # Clamp to the grid actually produced; if the map is smaller than the
        # window we keep what exists rather than padding with unknown cells,
        # which would falsely enlarge the map.
        x0, x1 = max(0, x0), min(width, x1)
        y0, y1 = max(0, y0), min(height, y1)
        if x1 <= x0 or y1 <= y0:
            self.get_logger().error(
                f'crop window does not intersect the map '
                f'({width}x{height} at origin [{ox}, {oy}])')
            sys.exit(1)

        cropped = grid[y0:y1, x0:x1]

        # Unknown cells outside any explored region stay unknown, but we
        # deliberately leave them as-is rather than filling them in.
        write_pgm(map_out, cropped)

        new_meta = dict(meta)
        new_meta['image'] = map_out.rsplit('/', 1)[-1]
        # World y of the top row retained, which is the new map origin.
        new_meta['origin'] = [ox + x0 * res,
                              oy + (height - y0) * res,
                              float(meta['origin'][2])]
        with open(map_out.rsplit('.', 1)[0] + '.yaml', 'w') as f:
            yaml.safe_dump(new_meta, f, default_flow_style=False)

        free = int((cropped == 254).sum())
        occ = int((cropped == 0).sum())
        unknown = int((cropped == 205).sum())
        total = cropped.size
        self.get_logger().info(
            f'Cropped {width}x{height} -> {cropped.shape[1]}x{cropped.shape[0]} '
            f'@ {res}m/cell, origin '
            f'[{new_meta["origin"][0]:.3f}, {new_meta["origin"][1]:.3f}]')
        self.get_logger().info(
            f'free {free} ({free / total * 100:.1f}%), '
            f'occupied {occ} ({occ / total * 100:.1f}%), '
            f'unknown {unknown} ({unknown / total * 100:.1f}%)')


def main():
    import rclpy
    rclpy.init()
    node = CropMap()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
