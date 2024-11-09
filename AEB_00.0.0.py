import carla
import time
import math

def aeb_system_with_radar(vehicle, threshold_distance=10.0, threshold_velocity=5.0):
    """
    Implements an AEB (Automatic Emergency Braking) system for a CARLA vehicle.
    Stops the vehicle if an obstacle is detected within a specified threshold distance
    and relative velocity threshold.
    """
    world = vehicle.get_world()
    blueprint_library = world.get_blueprint_library()
    
    # Set up a front-facing radar sensor
    radar_bp = blueprint_library.find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', '30')  # 30 degrees horizontal field of view
    radar_bp.set_attribute('vertical_fov', '10')    # 10 degrees vertical field of view
    radar_bp.set_attribute('range', '50')           # Radar range of 50 meters
    
    # Spawn the radar sensor at the front of the vehicle
    radar_transform = carla.Transform(carla.Location(x=2.5, z=1.0))  # Offset the radar to the front of the car
    radar = world.spawn_actor(radar_bp, radar_transform, attach_to=vehicle)

    # Define a callback function to process radar data
    def on_radar_data(data):
        for detection in data:
            # Calculate the distance to the detected object
            distance = detection.depth

            # Calculate relative velocity (positive values indicate the object is approaching)
            relative_velocity = detection.velocity
            
            # Check if the detected object is within the AEB activation threshold
            if distance < threshold_distance and relative_velocity > threshold_velocity:
                print("Obstacle detected by radar! Applying brakes.")
                vehicle.apply_control(carla.VehicleControl(brake=1.0))
                return

        # If no obstacle is within threshold, release the brakes
        vehicle.apply_control(carla.VehicleControl(brake=0.0))
    
    # Attach the callback function to the radar sensor
    radar.listen(on_radar_data)
    
    # Run the simulation for a while
    try:
        time.sleep(20)  # Run for 20 seconds as a test duration
    finally:
        # Clean up the radar sensor and destroy it
        radar.stop()
        radar.destroy()

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Spawn a vehicle
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('model3')[0]
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle.set_autopilot(True)  # Enable autopilot to simulate driving

    # Implement the AEB system using radar
    try:
        aeb_system_with_radar(vehicle, threshold_distance=10.0, threshold_velocity=5.0)
    finally:
        vehicle.destroy()
        print("AEB test with radar complete.")

if __name__ == '__main__':
    main()
