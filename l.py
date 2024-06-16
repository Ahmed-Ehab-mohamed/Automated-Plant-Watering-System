import rospy
from std_msgs.msg import Float32, String
from datetime import datetime
import joblib

# Initialize variables
soil_moisture = 0.0
temperature = 0.0
humidity = 0.0
rainfall = 0.0

# File to store readings
data_file = 'schedueling.txt'

# Load the trained model
model = joblib.load('Watering.joblib')

def append_to_file(file_path, content):
    with open(file_path, 'a') as f:
        f.write(content)

def predict_watering(model, soil_moisture, temperature, humidity, rainfall):
    # Make predictions using the model
    data = [[soil_moisture, temperature, humidity, rainfall]]
    prediction = model.predict(data)
    return prediction[0]

def soil_moisture_callback(data):
    global soil_moisture
    soil_moisture = data.data
    now = datetime.now()
    current_time = now.strftime("%Y-%m-%d %H:%M:%S")
    entry = f"{current_time} - Soil Moisture: {soil_moisture}\n"
    append_to_file(data_file, entry)
    # Make prediction and publish it
    publish_prediction()

def temperature_callback(data):
    global temperature
    temperature = data.data
    now = datetime.now()
    current_time = now.strftime("%Y-%m-%d %H:%M:%S")
    entry = f"{current_time} - Temperature: {temperature}\n"
    append_to_file(data_file, entry)
    # Make prediction and publish it
    publish_prediction()

def humidity_callback(data):
    global humidity
    humidity = data.data
    now = datetime.now()
    current_time = now.strftime("%Y-%m-%d %H:%M:%S")
    entry = f"{current_time} - Humidity: {humidity}\n"
    append_to_file(data_file, entry)
    # Make prediction and publish it
    publish_prediction()

def rainfall_callback(data):
    global rainfall
    rainfall = data.data
    now = datetime.now()
    current_time = now.strftime("%Y-%m-%d %H:%M:%S")
    entry = f"{current_time} - Rainfall: {rainfall}\n"
    append_to_file(data_file, entry)
    # Make prediction and publish it
    publish_prediction()

def publish_prediction():
    # Make prediction using the current data
    watering_required = predict_watering(model, soil_moisture, temperature, humidity, rainfall)
    # Publish the prediction
    pub_prediction.publish(watering_required)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("soilMoisture", Float32, soil_moisture_callback)
    rospy.Subscriber("temperatureC", Float32, temperature_callback)
    rospy.Subscriber("humidity", Float32, humidity_callback)
    rospy.Subscriber("rainfall", Float32, rainfall_callback)
    rospy.spin()

if __name__ == '__main__':
    # Initialize publisher for prediction
    pub_prediction = rospy.Publisher('prediction', Float32, queue_size=10)
    listener()

