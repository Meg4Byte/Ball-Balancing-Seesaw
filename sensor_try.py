import VL53L0X
import time

# Create VL53L1X sensor objects for Sensor 1 and Sensor 2
tof_sensor1 = VL53L0X.VL53L0X(i2c_bus=1, i2c_address=0x2A)  # Set the I2C address
tof_sensor2 = VL53L0X.VL53L0X(i2c_bus=1, i2c_address=0x29)  # Set the I2C address of the second sensor

# Configure both sensors (adjust settings as needed)

tof_sensor1.start_ranging()
tof_sensor2.start_ranging()

try:
    while True:

        # Wait for measurements to be ready for both sensors
        while not tof_sensor1.is_ready() or not tof_sensor2.is_ready():
            time.sleep(0.01)

        # Get distance measurements for both sensors
        distance_mm1 = tof_sensor1.get_distance()
        distance_cm1 = distance_mm1 / 10.0

        distance_mm2 = tof_sensor2.get_distance()
        distance_cm2 = distance_mm2 / 10.0

        print(f"Sensor 1 Distance: {distance_cm1} cm")
        #print(f"Sensor 2 Distance: {distance_cm2} cm")

        time.sleep(0.1)  # Adjust the sampling rate as needed

except KeyboardInterrupt:
    pass

finally:
    tof_sensor1.stop_ranging()
    tof_sensor2.stop_ranging()

