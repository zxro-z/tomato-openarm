import cv2
import numpy as np

# 500x500 검은 배경 생성
img = np.zeros((500, 500, 3), dtype=np.uint8)

# 3px부터 10px까지 다양한 크기의 타원형 점(마커) 그리기
# (중심좌표, (단축, 장축), 회전각, 시작각, 끝각, 색상(빨강), 두께)
marker_sizes = [3, 4, 5, 6, 7, 8, 9, 10]
for i, size in enumerate(marker_sizes):
    center = (50 + i * 50, 50 + (i % 3) * 100)
    # 단축(minor axis)이 size가 되도록 설정
    cv2.ellipse(img, center, (size, size + 2), i * 15, 0, 360, (0, 0, 255), -1)

# 이미지 저장
cv2.imwrite('test_markers.png', img)