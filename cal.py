from MPU6050 import MPU6050
from SimplePID import SimplePID


def avg_from_array(a_array):
    for index in range(0, len(a_array)):
        sum = a_array[index]

    return sum/len(a_array)


i2c_bus = 1
device_address = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = 0
y_accel_offset = 0
z_accel_offset =0
x_gyro_offset = 0
y_gyro_offset = 0
z_gyro_offset = 0
enable_debug_output = True

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

pid = SimplePID(0, -15000, 15000, 0.001, 0.0001, 0.0001, 100, True)

pid.set_delta_time_ms(100)

accel_reading = mpu.get_acceleration()


x_accel_reading = accel_reading[0]
y_accel_reading = accel_reading[1]
z_accel_reading = accel_reading[2]

x_accel_avg = [0]*100
y_accel_avg = [0]*100
z_accel_avg = [0]*100

x_accel_offset_avg = [0]*100
y_accel_offset_avg = [0]*100
z_accel_offset_avg = [0]*100

index = 0

try:
    while True:
        if pid.check_time():
            accel_reading = mpu.get_acceleration()
            x_accel_reading = accel_reading[0]
            y_accel_reading = accel_reading[1]
            z_accel_reading = accel_reading[2]
            x_accel_offset = pid.get_output_value(x_accel_reading)
            print('read: ' + str(x_accel_reading) + ' off: ' + str(x_accel_offset))
            mpu.set_x_accel_offset(int(x_accel_offset))
            x_accel_avg[index] = x_accel_reading
            x_accel_offset_avg[index] = x_accel_offset
            index += 1
            if index == len(x_accel_avg):
                index = 0
                print('x_avg_read: ' +
                      str(avg_from_array(x_accel_avg)) +
                      ' x_avg_offset: ' +
                      str(avg_from_array(x_accel_offset_avg)))
except KeyboardInterrupt:
    pass
