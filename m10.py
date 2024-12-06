import time
import numpy as np
from PyQt5.QtCore import pyqtSignal, QThread
from math import radians, sin, cos, sqrt, atan2, degrees, acos


def find_closest_obstacle(angle_range, lidar_data):
    closest_distance = float('inf')
    closest_angle = None
    # TODO: mke sure which is start angle of lms data
    angle = 0
    for _, reading in enumerate(lidar_data):
        if angle_range[0] <= angle <= angle_range[1]:
            if reading < closest_distance:
                closest_distance = reading
                closest_angle = angle
        angle += 0.5

    return closest_distance, closest_angle


def decide_turning_side(lidar_data, start=0, end=350):
    right_sum = sum(8000 if num > 8000 else num for num in lidar_data[0:180])
    left_sum = sum(8000 if num > 8000 else num for num in lidar_data[180:])
    if left_sum > right_sum:
        return "Turn left"
    elif right_sum > left_sum:
        return "Turn right"

class Autonomy:
    def __init__(self):
        pass

    def calculate_angle(self, ugv_lat=0.0, ugv_long=0.0, lat2=0.0, long2=0.0, facing_direction=0):
        """

        :param ugv_lat:
        :param ugv_long:
        :param lat2:
        :param long2:
        :param facing_direction:
        :return: angle in deg
        """
        # Convert decimal degrees to radians
        ugv_lat, ugv_long, lat2, long2 = map(radians, [ugv_lat, ugv_long, lat2, long2])

        # Calculate the bearing angle between the two points
        y = sin(long2 - ugv_long) * cos(lat2)
        x = cos(ugv_lat) * sin(lat2) - sin(ugv_lat) * cos(lat2) * cos(long2 - ugv_long)
        bearing = degrees(atan2(y, x))

        # Calculate the angle between the bearing angle and the facing direction
        angle = (bearing - facing_direction) % 360
        #print(f"calculate angle = {angle}")

        return angle

    def calculate_distance(self, ugv_lat=0.0, ugv_long=0.0, lat2=0.0, lon2=0.0):
        """

        :param ugv_lat:
        :param ugv_long:
        :param lat2:
        :param lon2:
        :return: next point distance from UGV in meters
        """
        radius = 6371000000  # radius of the earth in meters
        dlat = radians(lat2 - ugv_lat)
        dlon = radians(lon2 - ugv_long)
        a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(ugv_lat)) \
            * cos(radians(lat2)) * sin(dlon / 2) * sin(dlon / 2)
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        dist = radius * c
        #print(f"calculate distance = {dist}")
        return dist


    def drive_ugv_at_const_speed(self, max_speed=5):
        """

        :param max_speed:
        :return:
        """
        pass

    def match_ugv_direction_with_next_point(self):
        pass


class AutonomyThread(QThread):
    autonomy_error = pyqtSignal(bool)

    def __init__(self, parent=None, mainclass=None):
        super(AutonomyThread, self).__init__(parent)
        self.main_obj = mainclass
        self.cal_obj = Autonomy()
        self.dist_th_flag = 0

        self.lidar_max_range = 6000  # mm
        self.rover_width = 1800  # mm
        self.rover_width_th = 200  # mm
        self.rover_width = self.rover_width + (self.rover_width_th * 2)  # mm

        side = (self.rover_width // 2) + self.rover_width_th
        self.side_c = sqrt(2) * side

        self.lms_angle_correction = 0

    def run(self):
        waypoints = self.main_obj.waypoints
        num_wp = len(waypoints)
        wp_counter = 1
        while not self.isInterruptionRequested():
            try:
                if self.main_obj.gps_th.get_gps_status() and self.main_obj.lms_th.get_lms_status() \
                        and self.main_obj.bms_th.get_bms_status() and self.main_obj.is_cords_avail \
                        and self.main_obj.is_heading_avail:
                # if self.main_obj.gps_th.get_gps_status() and self.main_obj.lms_th.get_lms_status() \
                #         and self.main_obj.bms_th.get_bms_status() and self.main_obj.number_of_satellite >= 6:
                    ugv_lat = self.main_obj.ugv_lat
                    ugv_long = self.main_obj.ugv_long
                    heading = self.main_obj.ugv_heading
                    print(f"UGV heading = {heading}")
                    # heading = 180
                    # ugv_lat = 73.95
                    # ugv_long = 18.54

                    if num_wp != 0 and wp_counter <= num_wp:
                        wp_lat = waypoints[f"wp{wp_counter}"][1]
                        wp_long = waypoints[f"wp{wp_counter}"][0]

                        angle = int(self.cal_obj.calculate_angle(ugv_lat=ugv_lat, ugv_long=ugv_long,
                                                             facing_direction=heading,
                                                             lat2=wp_lat, long2=wp_long))

                        dist = int(self.cal_obj.calculate_distance(ugv_lat=ugv_lat, ugv_long=ugv_long, lat2=wp_lat,
                                                               lon2=wp_long))
                        try:
                            lms = self.main_obj.lms_readings
                            no_of_obstacles, obstacle_data = self._find_obstacles_in_fov(lidar_data=lms)
                            distance, ob_angle = find_closest_obstacle((0, 269), lms)

                            if 90 <= angle <= 270 and dist <= 5000:
                                print("waypoint is behind ugv skip it")
                                wp_counter += 1

                            elif distance <= 2000:
                                if 90 <= angle <= 270:
                                    self.move_ugv(angle)
                                else:
                                    print("obstacle less than 2m")
                                    self.dist_th_flag = 1
                                    lms = self.main_obj.lms_readings
                                    distance, ob_angle = find_closest_obstacle((0, 269), lms)
                                    print(distance, ob_angle)
                                    if ob_angle < 135 and distance > 650:
                                        distance, ob_angle = find_closest_obstacle((135, 225), lms)
                                        self.main_obj.send_curtis_control(left_throttle=0, right_throttle=-7000)
                                        time.sleep(0.1)
                                        if distance > 1500:
                                            self.main_obj.send_curtis_control(left_throttle=-5000, right_throttle=0)
                                        elif distance > 1000:
                                            self.main_obj.send_curtis_control(left_throttle=0, right_throttle=-8000)
                                        elif distance > 700 and ob_angle > 190:
                                            self.main_obj.send_curtis_control(left_throttle=0, right_throttle=-8000)
                                        elif distance > 600 and ob_angle > 195:
                                            self.main_obj.send_curtis_control(left_throttle=0, right_throttle=-8000)
                                        else:
                                            self.main_obj.motor_control.stop_ugv()

                                    elif ob_angle >= 135 and distance > 650:
                                        distance, ob_angle = find_closest_obstacle((45, 135), lms)
                                        self.main_obj.send_curtis_control(left_throttle=0, right_throttle=7000)
                                        time.sleep(0.1)
                                        if distance > 1500:
                                            self.main_obj.send_curtis_control(left_throttle=-5000, right_throttle=0)
                                        elif distance > 1000:
                                            self.main_obj.send_curtis_control(left_throttle=0, right_throttle=8000)
                                        elif distance > 700 and ob_angle < 80:
                                            self.main_obj.send_curtis_control(left_throttle=0, right_throttle=8000)
                                        elif distance > 600 and ob_angle < 75:
                                            self.main_obj.send_curtis_control(left_throttle=0, right_throttle=8000)
                                        else:
                                            self.main_obj.motor_control.stop_ugv()

                                    else:
                                        self.main_obj.motor_control.stop_ugv()
                                    time.sleep(0.1)

                            else:
                                if dist >= 2000:  # 1-meter radius we consider UGV reached at waypoint
                                    # TODO: confirm lms write data start angle and end angle
                                    distance, ob_angle = find_closest_obstacle((30, 240), lms)
                                    turning_side = decide_turning_side(lms)

                                    # print(f"distance = {distance}, obstacle angle = {ob_angle}, turning side = {turning_side}")
                                    if distance < 8000:
                                        if obstacle_data:
                                            _obstacle_path, obstacle_side, start_dist, end_dist = self._is_obstacle_in_path(
                                                obstacle_data=obstacle_data)
                                            print(
                                                f"{obstacle_side}, {_obstacle_path}, {distance}, {dist}, {start_dist}, {end_dist}")
                                        else:
                                            _obstacle_path = False
                                        if _obstacle_path:
                                            if distance <= 1000:
                                                print("obstacle less than 1m stop")
                                                self.dist_th_flag = 1
                                                self.main_obj.motor_control.stop_ugv()

                                            elif distance <= 2000 and dist > 2000:
                                                distance, ob_angle = find_closest_obstacle((30, 240), lms)
                                                turning_side = decide_turning_side(lms, start=60, end=480)
                                                if self.dist_th_flag == 3 and distance > 1900:
                                                    pass
                                                else:
                                                    self.dist_th_flag = 2
                                                    if turning_side == "Turn left":
                                                        self.main_obj.send_curtis_control(left_throttle=0,
                                                                                          right_throttle=-7000)
                                                    elif turning_side == "Turn right":
                                                        self.main_obj.send_curtis_control(left_throttle=0,
                                                                                          right_throttle=7000)
                                                    else:
                                                        self.main_obj.motor_control.stop_ugv()

                                            elif distance <= 3000 and dist > 3000:
                                                distance, ob_angle = find_closest_obstacle((30, 240), lms)
                                                turning_side = decide_turning_side(lms, start=60, end=480)
                                                if self.dist_th_flag == 2 and distance < 2100:
                                                    pass
                                                else:
                                                    self.dist_th_flag = 3
                                                    if turning_side == "Turn left":
                                                        self.main_obj.send_curtis_control(left_throttle=-7000,
                                                                                          right_throttle=-7000)
                                                    elif turning_side == "Turn right":
                                                        self.main_obj.send_curtis_control(left_throttle=-7000,
                                                                                          right_throttle=7000)
                                                    else:
                                                        self.main_obj.motor_control.stop_ugv()

                                            elif distance <= 6000 and dist > 6000:
                                                distance, ob_angle = find_closest_obstacle((30, 240), lms)
                                                turning_side = decide_turning_side(lms, start=60, end=480)
                                                if (self.dist_th_flag == 3 and distance > 3100) \
                                                        or (self.dist_th_flag == 5 and distance > 5900):
                                                    pass
                                                else:
                                                    self.dist_th_flag = 4
                                                    if turning_side == "Turn left":
                                                        self.main_obj.send_curtis_control(left_throttle=-7000,
                                                                                          right_throttle=-5000)
                                                    elif turning_side == "Turn right":
                                                        self.main_obj.send_curtis_control(left_throttle=-7000,
                                                                                          right_throttle=5000)
                                                    else:
                                                        self.main_obj.motor_control.stop_ugv()

                                            elif distance > 6000:
                                                if self.dist_th_flag == 4 and distance < 6100:
                                                    pass
                                                else:
                                                    self.dist_th_flag = 5
                                                    self.move_ugv(angle)
                                                    # self.main_obj.send_curtis_control(left_throttle=-5000, right_throttle=0)
                                            else:
                                                self.move_ugv(angle)
                                                print("\n else \n")
                                            time.sleep(0.05)
                                        else:
                                            self.move_ugv(angle)
                                    else:
                                        # self.main_obj.send_curtis_control(left_throttle=-7000, right_throttle=0)
                                        self.move_ugv(angle)
                                else:
                                    print("wapoint reached")
                                    wp_counter += 1
                        except:
                            self.autonomy_error.emit(True)
                            print("LMS data read problem")
                            break
                    else:
                        print("mission complete")
                        self.main_obj.motor_control.stop_ugv()
                else:
                    print("HW error")
                    self.autonomy_error.emit(True)
                    break
            except Exception as e:
                print(f"error in autonomy fun: {e}")
                self.main_obj.motor_control.stop_ugv()
        print("autonomy stop")
        try:
            self.main_obj.motor_control.stop_ugv()
        except:
            pass

    def _find_obstacles_in_fov(self, lidar_data):
        """
        return angle in degree and distance in mm
        :param lidar_data: list of lidar data
        :return: number of obstacels in fov, obstacle details (start_angle, end_angle, start_dist, end_dist)
        """
        processed_data = [min(self.lidar_max_range,
                              value) if value > self.lidar_max_range else self.lidar_max_range if value == 0 else value
                          for
                          value in lidar_data]

        angles = np.linspace(0, len(processed_data) / 2, len(processed_data), endpoint=False)

        objects = []
        current_object = []
        obj_angles = []
        current_obj_angle = []

        for i, distance in enumerate(processed_data):
            if distance < self.lidar_max_range:  # Start of a new object
                current_object.append(distance)  # Add readings to the current object
                current_obj_angle.append(angles[i])
            elif self.lidar_max_range <= distance < self.lidar_max_range:  # Continuation of the current object
                current_object.append(distance)  # Add readings to the current object
                current_obj_angle.append(angles[i])
            elif current_object:  # End of the current object
                objects.append(current_object)  # Store the current object
                current_object = []  # Reset for the next object
                obj_angles.append(current_obj_angle)
                current_obj_angle = []

        # Check for the last object
        if current_object:
            objects.append(current_object)  # Store the last object
            obj_angles.append(current_obj_angle)

        obstacle_details = []
        for i, ang in enumerate(obj_angles):
            start_angle = ang[0]
            end_angle = ang[-1]
            start_dist = objects[i][0]
            end_dist = objects[i][-1]
            obstacle_details.append((start_angle, end_angle, start_dist, end_dist))

        return len(obstacle_details), obstacle_details


    def _is_obstacle_in_path(self, obstacle_data):
        """

        :param obstacle_data:
        :return:
        """
        side = 0
        for obstacle in obstacle_data:
            start_angle = obstacle[0]
            end_angle = obstacle[1]
            start_dist = obstacle[2]
            end_dist = obstacle[3]

            if start_angle <= 135 and end_angle <= 135:
                print("obstacle in right plane")
                angle_a_deg = end_angle
                side_b = end_dist
                side = 1
            elif start_angle > 135 and end_angle > 135:
                print("obstacle in left plane")
                angle_a_deg = 270 - start_angle
                side_b = start_dist
                side = 2
            else:
                print("obstacle in center")
                return True, 3, start_dist, end_dist

            angle_a_rad = radians(angle_a_deg)
            side_a = sqrt(side_b ** 2 + self.side_c ** 2 - 2 * side_b * self.side_c * cos(angle_a_rad))
            angle_b_rad = acos((side_a ** 2 + self.side_c ** 2 - side_b ** 2) / (2 * side_a * self.side_c))
            angle_b_deg = degrees(angle_b_rad)
            # print(side_a, side_b, self.side_c, angle_a_deg, angle_b_deg)
            # if angle_b_deg > 45:
            #     return False, side, start_dist, end_dist
            # else:
            #     return True, side, start_dist, end_dist

            # if lidar not actually center
            if side == 1:
                a = 45 - self.lms_angle_correction
                if angle_b_deg > a:
                    return False, side, start_dist, end_dist
                else:
                    return True, side, start_dist, end_dist

            elif side == 2:
                a = 45 + self.lms_angle_correction
                if angle_b_deg > a:
                    return False, side, start_dist, end_dist
                else:
                    return True, side, start_dist, end_dist


    def move_ugv(self, angle):
        print(f"normal opration, {angle}")
        if angle >= 358 or angle <= 2:
            print("move f/w")
            self.main_obj.send_curtis_control(left_throttle=-4000, right_throttle=0)
        elif 2 < angle <= 40:
            print("long right turn")
            self.main_obj.send_curtis_control(left_throttle=-7000, right_throttle=6000)
        elif 40 < angle <= 70:
            print("sharp right turn")
            self.main_obj.send_curtis_control(left_throttle=-6000, right_throttle=7000)
        elif 70 < angle <= 180:
            print("right skid")
            self.main_obj.send_curtis_control(left_throttle=0, right_throttle=7000)
            time.sleep(0.5)
        elif 320 <= angle < 358:
            print("long left turn")
            self.main_obj.send_curtis_control(left_throttle=-7000, right_throttle=-6000)
        elif 290 <= angle < 320:
            print("sharp left turn")
            self.main_obj.send_curtis_control(left_throttle=-7000, right_throttle=-7000)
        elif 180 < angle < 290:
            print("left skid")
            self.main_obj.send_curtis_control(left_throttle=-0, right_throttle=-7000)
        else:
            print(f"else stop angle={angle}")
            self.main_obj.motor_control.stop_ugv()
        time.sleep(0.05)

    def stop(self):
        self.requestInterruption()
        self.wait()
