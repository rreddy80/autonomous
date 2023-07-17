""" 
Planning a complete autonomous flight in a grid pattern without GPS on PX4 or ArduPilot can be challenging because these autopilot systems heavily rely on GPS for position estimation and control. However, it is still possible to simulate a grid pattern flight using the available capabilities like IMU and distance sensors. Here's an example code that demonstrates the concept:
In this example, we use the dronekit library to communicate with the PX4 or ArduPilot-based autopilot. After connecting to the vehicle, we set the vehicle mode to "GUIDED" and arm the vehicle. Then, we initiate a simple takeoff to the desired altitude.

Next, we define the grid pattern parameters such as the grid size and spacing between grid points. We iterate through each grid point and calculate the target position. We set the target location using vehicle.simple_goto() and specify the desired groundspeed.

We wait until the vehicle reaches the target location by continuously checking the distance to the target location. Once the vehicle reaches the target location, we move to the next grid point. This process continues until all grid points are covered.

After completing the grid pattern flight, we return to the original takeoff location using the same approach. Finally, we set the vehicle mode to "LAND" and wait until the vehicle lands. Then, we close the connection to the vehicle.

Please note that without GPS, the accuracy of position estimation and control will be limited. The example code provided here is a simplified demonstration and may require modifications and additional safety measures depending on your specific use case and requirements.
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time

# Connect to the vehicle
# connection_string = '/dev/ttyACM0'  # Replace with the actual connection string, ex: 'udp:127.0.0.1:14550'
connection_string = '127.0.0.1:14450'
vehicle = connect(connection_string, wait_ready=True)

# Set the vehicle to GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
time.sleep(1)  # Wait for the mode to take effect

# Arm the vehicle
vehicle.armed = True
while not vehicle.armed:
    time.sleep(1)
vehicle.mode = VehicleMode("GUIDED")
time.sleep(1)  # Wait for the arming to complete

# Takeoff to a desired altitude
target_altitude = 10  # Replace with the desired altitude in meters
# Define grid pattern parameters
grid_size = 10  # Number of rows and columns in the grid
grid_spacing = 5  # Spacing between grid points in meters

vehicle.simple_takeoff(target_altitude)

# Wait until the vehicle reaches the target altitude
while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    if current_altitude >= target_altitude * 0.95:
        print("Vehicle reached the target altitude")
        break
    time.sleep(1)

# Start the grid pattern flight
for row in range(grid_size):
    for col in range(grid_size):
        # Calculate target position for each grid point
        target_x = col * grid_spacing
        target_y = row * grid_spacing
        target_altitude = 10  # Maintain a constant altitude

        # Set the target position
        target_location = LocationGlobalRelative(target_x, target_y, target_altitude)
        vehicle.simple_goto(target_location, groundspeed=5)  # Set the desired groundspeed in m/s

        # Wait until the vehicle reaches the target location
        while True:
            current_location = vehicle.location.global_relative_frame
            distance_to_target = current_location.distance_to(target_location)
            if distance_to_target <= 1:  # Replace with an appropriate threshold
                print("Vehicle reached the target location")
                break
            time.sleep(1)

# Return to the original takeoff location
return_location = LocationGlobal(vehicle.home_location.lat, vehicle.home_location.lon, target_altitude)
vehicle.simple_goto(return_location, groundspeed=5)

# Wait until the vehicle returns to the takeoff location
while True:
    current_location = vehicle.location.global_relative_frame
    distance_to_takeoff = current_location.distance_to(return_location)
    if distance_to_takeoff <= 1:  # Replace with an appropriate threshold
        print("Vehicle returned to the takeoff location")
        break
    time.sleep(1)

# Land the vehicle
vehicle.mode = VehicleMode("LAND")

# Wait until the vehicle lands
while vehicle.armed:
    time.sleep(1)

# Close the connection to the vehicle
vehicle.close()
