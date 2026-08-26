"""Generate a printable ArUco marker image."""

from __future__ import annotations

import argparse

import cv2

from .aruco_pose import dictionary_from_name


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--id", type=int, default=0)
    parser.add_argument("--pixels", type=int, default=1000)
    parser.add_argument("--border-bits", type=int, default=1)
    parser.add_argument("--output", default="aruco_marker_0.png")
    args = parser.parse_args()
    if args.pixels < 100:
        raise ValueError("pixels must be at least 100")
    dictionary = dictionary_from_name(args.dictionary)
    image = cv2.aruco.generateImageMarker(dictionary, args.id, args.pixels, borderBits=args.border_bits)
    if not cv2.imwrite(args.output, image):
        raise OSError(f"could not write {args.output}")
    print(f"Saved marker {args.id} from {args.dictionary} to {args.output}")
    print("Print without scaling, then measure the black-square side and set marker.size_m to that value.")
