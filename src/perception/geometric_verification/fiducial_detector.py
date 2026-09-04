import cv2
from matplotlib import markers
import numpy as np
from scipy.spatial import distance

def detect_fiducial_markers(image_path, k_px=5):
    # 1. 이미지 로드
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError("이미지를 찾을 수 없습니다.")

    # 2. HSV 색공간 변환
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 3. 빨간색 마커 HSV 임계값 설정 (현재 이미지 기준)
    # 빨간색은 HSV 공간에서 Hue 값이 0 근처와 180 근처에 분포함
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    
    # 4. 노이즈 제거 (Opening)
    kernel = np.ones((3,3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # 5. 외곽선(Contour) 검출
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected_markers = []
    output_img = img.copy()
    
    for cnt in contours:
        if len(cnt) < 5: continue # 최소한의 점이 있어야 타원 피팅 가능
        
        # 6. 타원 피팅
        ellipse = cv2.fitEllipse(cnt)
        (x, y), (ma, MA), angle = ellipse
        minor_axis = min(ma, MA)
        
        # 7. 기하학적 필터링 (단축 >= k_px)
        if minor_axis >= k_px:
            detected_markers.append({'center': (int(x), int(y)), 'minor_axis': minor_axis})
            # 시각화: 검출된 마커 표시
            cv2.ellipse(output_img, ellipse, (0, 255, 0), 2)
            cv2.circle(output_img, (int(x), int(y)), 2, (255, 0, 0), -1)
    
    return detected_markers, output_img

def match_markers_to_ids(detected_markers, marker_db, threshold=20):
    if not detected_markers: return {}
    
    detected_points = [m['center'] for m in detected_markers]
    ref_ids = list(marker_db.keys())
    ref_points = list(marker_db.values())
    
    dist_matrix = distance.cdist(detected_points, ref_points, 'euclidean')
    
    assigned_ids = {}
    for i, detected_point in enumerate(detected_points):
        closest_ref_idx = np.argmin(dist_matrix[i])
        if dist_matrix[i][closest_ref_idx] < threshold:
            assigned_ids[ref_ids[closest_ref_idx]] = detected_point
            
    return assigned_ids

markers, result_img = detect_fiducial_markers('test_markers.png', k_px=5)

# fiducial_detector.py의 실행부(매칭 로직)에 추가할 marker_db
# 3D 좌표를 2D 이미지 좌표로 투영한 결과(예상값)를 담는 데이터베이스입니다.

marker_db = {
    'M_001': (150, 250), # ID 0 대응
    'M_002': (299, 250), # ID 1 대응
    'M_003': (99, 149),  # ID 2 대응
    'M_004': (249, 150), # ID 3 대응
    'M_005': (399, 150), # ID 4 대응
    'M_006': (50, 50),   # ID 5 대응
    'M_007': (199, 49),  # ID 6 대응
    'M_008': (349, 50),  # ID 7 대응
}

# 기하 매칭 수행
identified_markers = match_markers_to_ids(markers, marker_db)

print(f"검출된 마커 개수: {len(markers)}")
print(f"매칭된 마커 개수: {len(identified_markers)}")

for marker_id, center in identified_markers.items():
    print(f"매칭 성공: {marker_id} -> 이미지 좌표 {center}")

cv2.imwrite('result_markers.png', result_img)