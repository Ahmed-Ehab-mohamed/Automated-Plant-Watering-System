# Flask app

from flask import Flask, render_template
import os

app = Flask(__name__)

def read_latest_readings(file_path):
    latest_readings = {'Soil Moisture': 'N/A', 'Temperature': 'N/A', 'Humidity': 'N/A', 'Rainfall': 'N/A'}
    if os.path.exists(file_path):
        with open(file_path, 'r') as f:
            lines = f.readlines()
            lines = [line.strip() for line in lines if line.strip()]  # Remove empty lines
            lines = lines[-4:]  # Get the last 4 non-empty lines
            for line in lines:
                parts = line.split(' - ')
                if len(parts) == 2:
                    key, value = parts[1].split(': ')
                    if key in latest_readings:
                        latest_readings[key] = value
    return latest_readings




@app.route('/')
def index():
    data_file = 'schedueling.txt'
    previous_readings = ""
    if os.path.exists(data_file):
        with open(data_file, 'r') as f:
            previous_readings = f.read()
    latest_readings = read_latest_readings(data_file)
    return render_template('index.html', latest_readings=latest_readings, previous_readings=previous_readings)

if __name__ == '__main__':
    app.run(debug=True, port=5000)

