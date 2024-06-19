import serial
from flask import Flask, jsonify, render_template
import threading

app = Flask(__name__)

# Global variables to store the latest GPS coordinates
latest_latitude = 0.0
latest_longitude = 0.0

def parse_gps_data(gps_data):
    # Split the GPS data string and extract latitude and longitude
    lat_str, lon_str = gps_data.split(',')
    latitude = float(lat_str)
    longitude = float(lon_str)
    return latitude, longitude

# Define the serial port
serial_port = 'COM12'

# Function to read GPS data from the serial port
def read_gps_data():
    global latest_latitude, latest_longitude
    try:
        ser = serial.Serial(serial_port, 9600, timeout=1)
        print(f"Serial connection established on {serial_port}")
        while True:
            if ser.in_waiting > 0:
                gps_data = ser.readline().decode().strip()
                if gps_data.startswith("GPS: "):
                    gps_data = gps_data.replace("GPS: ", "")
                    latitude, longitude = parse_gps_data(gps_data)
                    latest_latitude = latitude
                    latest_longitude = longitude
    except serial.SerialException as e:
        print(f"Error: Failed to open serial port {serial_port}: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    finally:
        ser.close()
        print("Serial connection closed.")

# API endpoint to serve GPS data
@app.route('/gps-data', methods=['GET'])
def get_gps_data():
    return jsonify(latitude=latest_latitude, longitude=latest_longitude)

# Route to serve the HTML page
@app.route('/')
def index():
    return render_template('map.html')

if __name__ == '__main__':
    # Start the GPS data reading in a separate thread
    gps_thread = threading.Thread(target=read_gps_data)
    gps_thread.daemon = True
    gps_thread.start()
    
    # Start the Flask server
    app.run(debug=True, port=8080)
