#!/usr/bin/env python3
"""Compare a post-voxel threshold offline; never initializes ROS or changes data."""
import argparse
import json
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from record_pointcloud_frames import field_array


def keep_intensity(values, threshold):
    if not np.isfinite(threshold) or not 0 <= threshold <= 255:
        raise ValueError('threshold must be finite and within [0,255]')
    if not np.isfinite(values).all():
        raise ValueError('non-finite intensity')
    return values > threshold


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_dir', type=Path)
    parser.add_argument('--threshold', type=float, default=3.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert keep_intensity(np.array([0., 2., 3., 3.01]), 3).tolist() == [False, False, False, True]
    rows = []
    for path in sorted(args.capture_dir.glob('cloud_*.cdr')):
        msg = deserialize_message(path.read_bytes(), PointCloud2)
        if msg.header.frame_id != 'base_link':
            raise ValueError('expected base_link post-voxel output')
        fields = {f.name: f for f in msg.fields}
        x, y, intensity = [field_array(msg, fields[k]).ravel() for k in ('x', 'y', 'intensity')]
        keep = keep_intensity(intensity, args.threshold)
        roi = (x > .45) & (x < .60) & (abs(y) < .18)
        rows.append({'file': path.name, 'baseline': len(x), 'retained': int(keep.sum()),
                     'suspected_roi_baseline': int(roi.sum()),
                     'suspected_roi_retained': int((roi & keep).sum())})
    if not rows:
        raise ValueError('no cloud_*.cdr files')
    result = {'source': str(args.capture_dir.resolve()), 'stage': 'post_voxel',
              'remove_intensity_le': args.threshold, 'frames': rows,
              'totals': {k: sum(r[k] for r in rows) for k in rows[0] if k != 'file'},
              'limitation': 'ROI is suspected only; no obstacle-material, costmap or localization acceptance'}
    # Exclusive create prevents overwriting prior experiment evidence.
    with args.output.open('x') as out:
        json.dump(result, out, indent=2)
    print(json.dumps(result['totals']))


if __name__ == '__main__':
    main()
