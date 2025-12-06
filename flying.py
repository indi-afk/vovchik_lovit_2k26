import rospy
import cv2
import numpy as np
import math
from math import nan
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
from clover import long_callback, srv
import tf2_ros
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from std_msgs.msg import String

rospy.init_node('flight')

camera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)
camera_matrix = np.float64(camera_info.K).reshape(3, 3)
distortion = np.float64(camera_info.D).flatten()
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


bridge = CvBridge() #мост для изображения
tubes = rospy.Publisher('/tubes', String, queue_size=1) #создаем топик

global map_points
map_points = []  # список точек в map 

def navigate_wait(x=0, y=0, z=0.8, yaw=float(0), speed=0.3, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        global cords_actual
        cords_actual = get_telemetry(frame_id='aruco_map') #получение  текущих координат в map
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def draw_line(frame, point1, point2):
    """
    добавляет на изображение линию от точки до точки
    """
    
    cv2.line(frame, tuple(point1), tuple(point2), (0, 0, 255), 1)

    
def img_xy_to_point(xy, dist):
    """
    преревод координат из точки на изображении в main_camera_optical (по примеру red_circle)
    """
    xy = cv2.undistortPoints(xy, camera_matrix, distortion, P=camera_matrix)[0][0]
    global Width_C, Hight_C
    Width_C, Hight_C = camera_info.width // 2, camera_info.height // 2
    xy -= camera_info.width // 2, camera_info.height // 2
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)

def pixel_to_map_transform_using_existing(img_xy_to_point_func, pixel_x, pixel_y, tf_buffer, header_frame_id, timeout=0.2):
    # print('cord_fin_cord')
    """
    переход из main_camera_optical в map (по примеру red_circle)
    """
    try:
        z = get_telemetry('terrain').z
      
        xy3d = img_xy_to_point_func((pixel_x, pixel_y), z)
        
        # Создаем PointStamped (как в исходной программе)
        target = PointStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = header_frame_id
        target.point = xy3d
        
        # Преобразуем в систему map
        setpoint = tf_buffer.transform(target, 'aruco_map', timeout=rospy.Duration(timeout))
        
        return (setpoint.point.x, setpoint.point.y)
    except Exception as e:
        rospy.logwarn(f"Transform failed: {e}")
        return None

def find_right_angles_in_yellow_contours(image, min_distance=0.75):
    """
    ищем прямые углы и получаем координаты углов (то бишь врезок) с проверкой на минимальное расстояние
    """
    if image is None:
        return []
    
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # желтый в hsv
    mask1 = cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))
    mask2 = cv2.inRange(hsv, (15, 80, 80), (35, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)
    
    # заполнение и очистка
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   
    
    # Находим контуры 
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    result_image = image.copy()
    
    
    for cnt in contours:
        #пропускаем мелкие контуры
        if cv2.contourArea(cnt) < 500:
            continue
            
        # нахождение периметра для апроксимации
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
            
        #апроксимация с коэфицентом
        epsilon = 0.015 * perimeter  
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        
        if len(approx) < 3:
            continue
        
        points = approx.reshape(-1, 2)
        
        for j in range(len(points)):
            #берем три последовательные точки
            a = points[(j - 1) % len(points)]
            b = points[j]
            c = points[(j + 1) % len(points)]
            
            #векторы
            ba = a - b
            bc = c - b
            
            #скалярное произведение и длины векторов
            dot = np.dot(ba, bc)
            norm_ba = np.linalg.norm(ba)
            norm_bc = np.linalg.norm(bc)
            
            #чтобы не делить на 0
            if norm_ba == 0 or norm_bc == 0:
                continue
            
            # фильтр по минимальной длине стороны
            if norm_ba < 10 or norm_bc < 10:
                continue
                
            cos_a = dot / (norm_ba * norm_bc) #косинус угла 
            cos_a = np.clip(cos_a, -1.0, 1.0) #условие от 1 до -1 чтобы учесть оштбки округления
            angle = np.degrees(np.arccos(cos_a))# угол через арккосинус
            
            # проверка угла с небольшим люфтом
            if 85 <= angle <= 95:  
                #корды вершины
                pixel_x = int(b[0])
                pixel_y = int(b[1])
                
                map_coords = pixel_to_map_transform_using_existing(img_xy_to_point_func=img_xy_to_point,pixel_x=pixel_x,pixel_y=pixel_y,tf_buffer=tf_buffer,header_frame_id='main_camera_optical')
                #трансформация из пикселя в map
                if map_coords:
                    map_x, map_y = map_coords
                    
                    # проверка на расстояние
                    is_far_enough = True
                    for existing_x, existing_y in map_points:
                        distance = math.sqrt((map_x - existing_x)**2 + (map_y - existing_y)**2)
                        if distance < min_distance:
                            is_far_enough = False
                            break
                    
                    if is_far_enough and scan_flag:
                        map_points.append((map_x, map_y))
                        print('New suspect pipe connection at: ', map_x, ' ', map_y)
                        
                        #зеленый кружок если находим
                        cv2.circle(result_image, (pixel_x, pixel_y), 6, (0, 255, 0), -1)
        
        # синий контур трубы
        cv2.drawContours(result_image, [approx], 0, (255, 0, 0), 1)
    
    #публикуем изображение 
    cv2.imshow('Actual image', result_image)
    cv2.waitKey(1)
    
    #возвращаем углы
    return map_points


@long_callback
def image_callback(data):
    global img_actual
    img_actual = bridge.imgmsg_to_cv2(data, 'bgr8')  # текущее изображение
    find_right_angles_in_yellow_contours(img_actual) #проверка на углы

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

def find_average_yellow_pixels_on_circle(frame, circle_radius, center=None, debug=True):
    print('Path point found!')
    """
    ищет на окружности с заданным центром и радиусом среднюю из лежащих на окружности желтых точек
    возвращает координаты
    """
    # размеры
    height, width = frame.shape[:2]
    
    # если центр не задан используем центр кадра
    if center is None:
        center = (width // 2, height // 2)
        # print(center)
    
    cx, cy = center  
    
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # диапазоны желтого
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    
    # создаем маску желтых пикселей
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # углы на которых лежат желтые точки
    found_angles = []
    
    # проверяем пиксели на углах на окружности 
    for angle_deg in range(0, 360):  
        # переводим угол в радианы
        angle_rad = math.radians(angle_deg)
        
        # вычисляем координаты точки на окружности
        x_abs = int(cx + circle_radius * math.cos(angle_rad))
        y_abs = int(cy + circle_radius * math.sin(angle_rad))
        
        # проверяем что координаты в пределах изображения
        if 0 <= x_abs < width and 0 <= y_abs < height:
            # проверяем является ли пиксель желтым
            if yellow_mask[y_abs, x_abs] > 0:
                found_angles.append(angle_deg)
    
    
    if not found_angles:
        return None
    
    # находим самую большую дугу желтых пикселей
    # сортируем углы
    found_angles.sort()
    
    # ищем самую длинную дугу
    max_length = 0
    max_start = 0
    max_end = 0
    
    i = 0
    while i < len(found_angles):
        start = i
        # идем вперед пока углы идут подряд (с учетом зацикления)
        while i < len(found_angles) - 1 and (
            found_angles[i+1] - found_angles[i] <= 2 or  # допуск 2 градуса
            (found_angles[i] > 350 and found_angles[i+1] < 10)  # переход через 360 или 0
        ):
            i += 1
        end = i
        
        # длина текущей дуги
        length = end - start + 1
        
        if length > max_length:
            max_length = length
            max_start = start
            max_end = end
        
        i += 1
    
    # если зацикливается, объединяем начало и конец
    if found_angles[-1] > 350 and found_angles[0] < 10:
        # проверяем объединенную дугу
        start_idx = len(found_angles) - 1
        while start_idx > 0 and found_angles[start_idx] - found_angles[start_idx-1] <= 2:
            start_idx -= 1
        
        end_idx = 0
        while end_idx < len(found_angles) - 1 and found_angles[end_idx+1] - found_angles[end_idx] <= 2:
            end_idx += 1
        
        combined_length = (len(found_angles) - start_idx) + (end_idx + 1)
        
        if combined_length > max_length:
            max_length = combined_length
            # для объединенной дуги берем среднее значение
            angles_for_combined = []
            angles_for_combined.extend(found_angles[start_idx:])  # конец списка
            angles_for_combined.extend(found_angles[:end_idx+1])  # начало списка
            
            # преобразуем углы для расчета среднего
            adjusted_angles = []
            for angle in angles_for_combined:
                if angle > 180:  # если угол больше 180, вычитаем 360 для правильного среднего
                    adjusted_angles.append(angle - 360)
                else:
                    adjusted_angles.append(angle)
            
            avg_angle_deg = np.mean(adjusted_angles)
            if avg_angle_deg < 0:
                avg_angle_deg += 360
        else:
            # берем углы из самой длинной непрерывной дуги
            avg_angle_deg = np.mean(found_angles[max_start:max_end+1])
    else:

        avg_angle_deg = np.mean(found_angles[max_start:max_end+1])
    
    # вычисляем координаты средней точки самой большой дуги
    avg_angle_rad = math.radians(avg_angle_deg)
    avg_x_abs = int(cx + circle_radius * math.cos(avg_angle_rad))
    avg_y_abs = int(cy + circle_radius * math.sin(avg_angle_rad))
    
    
    
    # возвращаем абсолютные координаты средней точки самой большой дуги
    # print('tchk na okr', avg_x_abs, avg_y_abs)
    return [avg_x_abs, avg_y_abs]


scan_flag = True

navigate_wait(z=1, frame_id='body', auto_arm=True) #взлет

navigate_wait(x=0, y=0)

navigate_wait(x=1, y=1, z=4) #подлет на точку сканирования 



img_path = img_actual.copy()
result_new = [Width_C, Hight_C] #записываем результат в центр
list_points = []
for i in range(40, 240, 40): #в радиусе 240 пикс с шагом 40 пикс ищем точки
    
    circle_radius = i
    result_old = result_new
    result_new = find_average_yellow_pixels_on_circle(img_path, circle_radius, center=None, debug=False)
    if result_new is not None:
        x_new, y_new = pixel_to_map_transform_using_existing(img_xy_to_point_func=img_xy_to_point, pixel_x=result_new[0], pixel_y=result_new[1], tf_buffer=tf_buffer, header_frame_id='main_camera_optical')
        draw_line(img_path, result_old, result_new) #рисуем линию полета
        list_points.append((x_new, y_new))
print('Path is: ', *list_points)

scan_flag = False

cv2.imshow("Fligh path", img_path)
cv2.waitKey(10)

for n in range(0, len(list_points)): #пролет по точкам
    x_trg = list_points[n][0]
    y_trg = list_points[n][1]
    navigate_wait(x_trg, y_trg, tolerance=0.1)
    rospy.sleep(1)

navigate_wait(x=0, y=0)

land() #посадка

# чистим лишний первый
if len(map_points) == 6:
    first_element = map_points.pop(0)


message = '' 
# создаем сообщение для публикации в топик
for i, (x, y) in enumerate(map_points, 1):
    message += str(x) +  ' ' + str(y) + '; '

# публикация
tubes.publish(data=message)
print('Published: ',message)
rospy.sleep(3)
