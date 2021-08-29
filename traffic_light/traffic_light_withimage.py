#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
"""
vid = cv2.VideoCapture('final0826.avi')

while True:
    ret, frame = vid.read()
    if not ret:
        break
    if ret:
        cv2.imshow('video', frame)
        roi = frame[:120, 160:480]
        cv2.imshow('roi', roi)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        cv2.imshow('hsv', hsv)


    if cv2.waitKey(33) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
"""

def color_detect(hsv, arr1, arr2):
	lower = np.array(arr1)
	upper = np.array(arr2)
	mask = cv2.inRange(hsv, lower, upper)
	return mask
"""
img = cv2.imread('Green.JPG')
img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)
roi = img[:120, 160:400]
gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (3, 3), 0)
edge = cv2.Canny(gray, 75, 200)
while True:
	cv2.imshow('canny', edge)

	edge, cnts, _ = cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]

	for c in cnts:
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.02*peri, True)

		if len(approx) == 4:
			screenCnt = approx
			break
	cv2.drawContours(roi, [screenCnt], -1, (0, 255, 0), 2)
	cv2.imshow('Outline', roi)

	if cv2.waitKey(33) & 0xFF == ord('q'):
		break
"""

img = cv2.imread('Yellow.JPG')
img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)

roi = img[:120, 160:400]
#cv2.imshow('roi', roi)

hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#cv2.imshow('hsv', hsv)
h, s, v = cv2.split(hsv)
roi_cpy = v.copy()
#cv2.imshow('v', v)

range_L = cv2.inRange(v, 220, 255)
#cv2.imshow('Light', range_L)

circles = cv2.HoughCircles(range_L, cv2.HOUGH_GRADIENT, 1, 100, param1=250, param2=10, minRadius=10, maxRadius=30)

print(circles)

if circles is not None:
	i = circles[0][0]
	cv2.circle(roi, (i[0], i[1]), i[2], (0, 0, 255), 5)

#cv2.imshow("result", roi)
#cv2.imshow("after", range_L)

ret, thr = cv2.threshold(roi_cpy, 180, 255, 0)
#cv2.imshow("thr", thr)
_, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#cv2.drawContours(roi, contours, -1, (0, 255, 0), 1)
#cv2.imshow('contour', roi)

cnts = sorted(contours, key = cv2.contourArea, reverse = True)

for c in cnts:
	peri = cv2.arcLength(c, True)
	approx = cv2.approxPolyDP(c, 0.02*peri, True)
	cnt_width = max(approx[0][0][0], approx[1][0][0], approx[2][0][0], approx[3][0][0]) - min(approx[0][0][0], approx[1][0][0], approx[2][0][0], approx[3][0][0])
	cnt_height = max(approx[0][0][1], approx[1][0][1], approx[2][0][1], approx[3][0][1]) - min(approx[0][0][1], approx[1][0][1], approx[2][0][1], approx[3][0][1])
	
	if len(approx) == 4 and cnt_width < 120 and cnt_width > 80:
		screenCnt = approx
		break
		print(approx)

		cv2.drawContours(roi, [screenCnt], -1, (0, 255, 0), 2)
cv2.imshow('Outline', roi)

print(len(cnts))

cv2.waitKey(0)



