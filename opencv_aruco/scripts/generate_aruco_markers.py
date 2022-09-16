#!/usr/bin/env python3
import numpy as np
import argparse
import cv2
import sys


ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

parser = argparse.ArgumentParser()
parser.add_argument(
    "-o", "--output", 
    default=".",
    help="path to save aruco marker",
)
parser.add_argument(
    "-i", "--id", 
    type=int, 
    default=-1,
    help="id of aruco marker to generate. Random if not specified",
)
parser.add_argument(
    "-t", "--type", 
    type=str, 
    default="DICT_4X4_50", 
    help="type of aruco marker to generate",
)
parser.add_argument(
    "-s", "--size", 
    type=int, 
    default=200, 
    help="size of the output aruco marker",
)
args = vars(parser.parse_args())

# Check to see if the dictionary is supported
if ARUCO_DICT.get(args["type"], None) is None:
	print(f"ArUCo marker type '{args['type']}' is not supported")
	sys.exit(0)

marker_dict= cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
size = args["size"]
marker_id = args["id"] if args["id"] > 0 else np.random.randint(50)
print(marker_id)

marker = np.zeros((size, size, 1), dtype="uint8")
cv2.aruco.drawMarker(marker_dict, marker_id, size, marker, 1)

# Save the tag generated
file_name = f'{args["output"]}/{args["type"]}_{marker_id}.png'
cv2.imwrite(file_name, marker)
# cv2.imshow("ArUCo Marker", marker)
# cv2.waitKey(0)
# cv2.destroyAllWindows()