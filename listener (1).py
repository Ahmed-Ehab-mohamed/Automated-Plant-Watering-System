# subscriber.py

import rospy
from std_msgs.msg import Float32, String
import os
from datetime import datetime

soil_moisture = 0.0
rainfall = 0.0
soil_status = ""

# File to store readings
data_file = 'readings.txt'

def append_to_file(file_path, content):
    with open(file_path, 'a') as f:
        f.write(content)

def read_previous_readings(file_path):
    if not os.path.exists(file_path):
        return "No previous readings available"
    with open(file_path, 'r') as f:
        return f.read()

def soil_moisture_callback(data):
    global soil_moisture
    soil_moisture = data.data
    now = datetime.now()
    current_time = now.strftime("%Y-%m-%d %H:%M:%S")
    entry = f"{current_time} - Soil Moisture: {soil_moisture}\n"
    append_to_file(data_file, entry)

def rainfall_callback(data):
    global rainfall
    rainfall = data.data
    now = datetime.now()
    current_time = now.strftime("%Y-%m-%d %H:%M:%S")
    entry = f"{current_time} - Rainfall: {rainfall}\n"
    append_to_file(data_file, entry)

def soil_status_callback(data):
    global soil_status
    soil_status = data.data
    now = datetime.now()
    current_time = now.strftime("%Y-%m-%d %H:%M:%S")
    entry = f"{current_time} - Soil Status: {soil_status}\n"
    append_to_file(data_file, entry)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("soilMoisture", Float32, soil_moisture_callback)
    rospy.Subscriber("rainfall", Float32, rainfall_callback)
    rospy.Subscriber("soilStatus", String, soil_status_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

