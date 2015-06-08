from MPU6050 import MPU6050
from SimplePID import SimplePID


def avg_from_array(a_array):
    sum = 0.0
    for index in range(0, len(a_array)):
        sum += a_array[index]

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

kp = 0.03125
ki = 0.25
kd = 0

pidax = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
piday = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
pidaz = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
pidgx = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
pidgy = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
pidgz = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)

#pid.set_delta_time_ms(100)

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

axindex = 0
ayindex = 0
azindex = 0

try:
    while True:
        accel_reading = mpu.get_acceleration()
        x_accel_reading = accel_reading[0]
        y_accel_reading = accel_reading[1]
        z_accel_reading = accel_reading[2]
        if pidax.check_time():
            x_accel_offset = pidax.get_output_value(x_accel_reading)

            mpu.set_x_accel_offset(int(x_accel_offset))

            x_accel_avg[axindex] = x_accel_reading
            x_accel_offset_avg[axindex] = x_accel_offset

            axindex += 1
            if axindex == len(x_accel_avg):
                axindex = 0
                print('x_avg_read: ' +
                      str(avg_from_array(x_accel_avg)) +
                      ' x_avg_offset: ' +
                      str(avg_from_array(x_accel_offset_avg)))
                print('y_avg_read: ' +
                      str(avg_from_array(y_accel_avg)) +
                      ' y_avg_offset: ' +
                      str(avg_from_array(y_accel_offset_avg)))
                print('z_avg_read: ' +
                      str(avg_from_array(z_accel_avg)) +
                      ' z_avg_offset: ' +
                      str(avg_from_array(z_accel_offset_avg)))

        if piday.check_time():
            y_accel_offset = piday.get_output_value(y_accel_reading)

            mpu.set_y_accel_offset(int(y_accel_offset))

            y_accel_avg[ayindex] = y_accel_reading
            y_accel_offset_avg[ayindex] = y_accel_offset

            ayindex += 1
            if ayindex == len(y_accel_avg):
                ayindex = 0

        if pidaz.check_time():
            z_accel_offset = pidaz.get_output_value(z_accel_reading)

            mpu.set_z_accel_offset(int(z_accel_offset))

            z_accel_avg[azindex] = z_accel_reading
            z_accel_offset_avg[azindex] = z_accel_offset

            azindex += 1
            if azindex == len(z_accel_avg):
                azindex = 0


except KeyboardInterrupt:
    pass
