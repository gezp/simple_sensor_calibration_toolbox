#!/usr/bin/env python3
# Copyright 2026 Gezp (https://github.com/gezp).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generate a printable ChArUco board."""

import argparse
import sys

import cv2

DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in (
        'DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_4X4_1000',
        'DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250', 'DICT_5X5_1000',
        'DICT_6X6_50', 'DICT_6X6_100', 'DICT_6X6_250', 'DICT_6X6_1000',
        'DICT_7X7_50', 'DICT_7X7_100', 'DICT_7X7_250', 'DICT_7X7_1000',
        'DICT_ARUCO_ORIGINAL',
    )
    if hasattr(cv2.aruco, name)
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='charuco.png')
    parser.add_argument('--squares-x', type=int, default=6)
    parser.add_argument('--squares-y', type=int, default=8)
    parser.add_argument('--square-length', type=float, default=0.04)
    parser.add_argument('--marker-length', type=float, default=0.03)
    parser.add_argument('--dictionary', default='DICT_5X5_1000', choices=sorted(DICTIONARIES))
    parser.add_argument('--pixels-per-meter', type=float, default=4000.0)
    parser.add_argument('--margin-meters', type=float, default=0.01)
    args = parser.parse_args()

    if args.marker_length >= args.square_length:
        print('marker-length must be smaller than square-length', file=sys.stderr)
        return 1
    if args.squares_x < 2 or args.squares_y < 2:
        print('squares-x and squares-y must be at least 2', file=sys.stderr)
        return 1

    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARIES[args.dictionary])
    board = cv2.aruco.CharucoBoard_create(
        args.squares_x, args.squares_y, args.square_length,
        args.marker_length, dictionary,
    )
    width = args.squares_x * args.square_length + 2.0 * args.margin_meters
    height = args.squares_y * args.square_length + 2.0 * args.margin_meters
    size = (
        max(1, int(round(width * args.pixels_per_meter))),
        max(1, int(round(height * args.pixels_per_meter))),
    )
    margin = int(round(args.margin_meters * args.pixels_per_meter))
    image = board.draw(size, marginSize=margin, borderBits=1)
    if not cv2.imwrite(args.output, image):
        print(f'failed to write {args.output}', file=sys.stderr)
        return 1
    print(f'wrote {args.output} ({size[0]}x{size[1]} px)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
