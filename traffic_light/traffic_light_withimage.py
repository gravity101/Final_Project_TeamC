#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2

# 이미지 읽기
img = cv2.imread('Yellow.JPG')
img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)

# ROI 및 HSV 전처리
roi = img[:120, 160:400]
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(hsv)
range_L = cv2.inRange(v, 220, 255)

# 신호 부분 원으로 표시
circles = cv2.HoughCircles(range_L, cv2.HOUGH_GRADIENT, 1, 100, param1=250, param2=10, minRadius=10, maxRadius=30)
if circles is not None:
	i = circles[0][0]
	cv2.circle(roi, (i[0], i[1]), i[2], (0, 0, 255), 5)

# 원의 중심 표시
circle_x = circles[0][0][0]
circle_y = circles[0][0][1]
cv2.circle(roi, (circle_x, circle_y), 2, (0, 0, 0), 5)

# threshold 후 윤곽선 찾기
ret, thr = cv2.threshold(v, 180, 255, 0)
_, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cnts = sorted(contours, key = cv2.contourArea, reverse = True)

for c in cnts:
	peri = cv2.arcLength(c, True)
	approx = cv2.approxPolyDP(c, 0.02*peri, True)

	if len(approx) == 4:
		cnt_width_max = max(approx[0][0][0], approx[1][0][0], approx[2][0][0], approx[3][0][0])
		cnt_width_min = min(approx[0][0][0], approx[1][0][0], approx[2][0][0], approx[3][0][0])
		cnt_height_max = max(approx[0][0][1], approx[1][0][1], approx[2][0][1], approx[3][0][1])
		cnt_height_min = min(approx[0][0][1], approx[1][0][1], approx[2][0][1], approx[3][0][1])
		cnt_width = cnt_width_max - cnt_width_min
		cnt_height = cnt_height_max - cnt_height_min

	# 사각형 표시
	if len(approx) == 4 and cnt_width < 120 and cnt_width > 80:
		screenCnt = approx
		print(approx)
		cv2.drawContours(roi, [screenCnt], -1, (0, 255, 0), 2)

		# 사각형의 중심 표시
		moment = cv2.moments(c)
		rect_x = int(moment['m10']/moment['m00'])
		rect_y = int(moment['m01']/moment['m00'])
		cv2.circle(roi, (rect_x, rect_y), 2, (0, 0, 0), 5)

		if circle_x < cnt_width_min:
			print("Outside")
		elif circle_x > cnt_width_max:
			print("Outside")
		elif circle_y < cnt_height_min:
			print("Outside")
		elif circle_y > cnt_height_max:
			print("Outside")
		elif circle_x < cnt_width_min + cnt_width_min/3:
			print("Red")
		elif circle_x > cnt_width_max - cnt_width_max/3:
			print("Green")
		else:
			print("Yellow")

		break


cv2.imshow('Outline', roi)

cv2.waitKey(0)



