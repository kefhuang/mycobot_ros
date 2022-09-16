# The generated image is suitable to print in one a4 paper

import numpy as np
import argparse
import cv2
import sys


marker_dict= cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

single_size = 100 
six_marks = np.full((5 * single_size, 3 * single_size, 1), 255, dtype="uint8")

for i in range(6):
	marker_id = i + 1
	marker = np.zeros((single_size, single_size, 1), dtype="uint8")
	cv2.aruco.drawMarker(marker_dict, marker_id, single_size, marker, 1)

	row_start = int(i / 2) * 2 * single_size
	row_end = row_start + 100
	col_start = (i % 2) * 2 * single_size 
	col_end = col_start + 100
	six_marks[row_start:row_end, col_start:col_end, :] = marker

# Save the tag generated
file_name = f'./six_markers.png'
cv2.imwrite(file_name, six_marks)
# cv2.imshow("ArUCo Marker", six_marks)
# cv2.waitKey(0)
# cv2.destroyAllWindows()