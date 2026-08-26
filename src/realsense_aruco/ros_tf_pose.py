"""Print one live base-to-tool transform from ROS TF."""

from __future__ import annotations

import argparse
import json

from .robot_pose_provider import RosTfPoseProvider
from .transforms import transform_dict


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-frame", default="base")
    parser.add_argument("--tool-frame", default="tool0")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    provider = RosTfPoseProvider(args.base_frame, args.tool_frame)
    try:
        matrix, timestamp = provider.get(args.timeout)
    finally:
        provider.close()
    output = transform_dict(matrix, args.base_frame, args.tool_frame)
    output["timestamp"] = timestamp
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
