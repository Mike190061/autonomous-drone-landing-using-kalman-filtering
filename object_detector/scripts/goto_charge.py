#!/usr/bin/env python3

import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState
import csv
import math

class GoToCharge:
    def __init__(self, csv_file, battery_threshold=0.20):
        self.battery_threshold = battery_threshold
        self.landing_pads = self.read_landing_pads(csv_file)
        
        rospy.init_node('battery_monitor')

        rospy.wait_for_service('get_telemetry')
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

        rospy.wait_for_service('navigate_global')
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)

        rospy.Subscriber('/mavros/battery', BatteryState, self.battery_callback)

    def read_landing_pads(self, csv_file):
        landing_pads = []
        with open(csv_file, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                landing_pads.append({
                    'Platform ID': int(row['Platform ID']),
                    'Marker ID': int(row['Marker ID']),
                    'Latitude': float(row['Latitude']),
                    'Longitude': float(row['Longitude'])
                })
        return landing_pads

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        meters = R * c
        meters = round(meters, 3)
        return meters

    def find_nearest_landing_pad(self, current_lat, current_lon):
        min_distance = float('inf')
        nearest_pad = None
        for pad in self.landing_pads:
            distance = self.haversine(current_lat, current_lon, pad['Latitude'], pad['Longitude'])
            if distance < min_distance:
                min_distance = distance
                nearest_pad = pad
        return nearest_pad

    def battery_callback(self, data):
        if data.percentage < self.battery_threshold:
            telemetry = self.get_telemetry(frame_id='map')
            nearest_pad = self.find_nearest_landing_pad(telemetry.lat, telemetry.lon)

            if nearest_pad:
                rospy.loginfo("Navigating to nearest landing pad: %s", nearest_pad)
                self.navigate_global(lat=nearest_pad['Latitude'], lon=nearest_pad['Longitude'], z=2, frame_id='map', auto_arm=True)

if __name__ == '__main__':
    csv_file = '/path/to/your/landing_pads.csv'
    go_to_charge = GoToCharge(csv_file)
    rospy.spin()
