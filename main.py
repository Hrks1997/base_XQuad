import uLoRa
import machine
from machine import I2C
import time
import utime
import _thread
import math
from mpu6050 import init_mpu6050, get_mpu6050_data

#utime.sleep(5)
# Flags and shared variables
pr=0.1
hold_alt_flag = False
arm_flag = False
yaw_flag = 0
prev_val = 0
values = [0] * 14
lock = _thread.allocate_lock()  # Lock for shared resources
min_pwm=1000
max_pwm=2200
initialization_pwm=900
disarm_pwm=900
tilt_stable_x=0
tilt_stable_y=0
fcv=0
fccv=0
bcv=0
bccv=0

# Motor PWM setup
fcc = machine.PWM(machine.Pin(0))
fc = machine.PWM(machine.Pin(1))
bc = machine.PWM(machine.Pin(2))
bcc = machine.PWM(machine.Pin(3))



y_mean=0.0



bcc.freq(15)#bcc
fcc.freq(15)#fcc1
bc.freq(15)#bc
fc.freq(15)#fc


# LoRa module setup
lora1 = uLoRa.LoRa()
led_receive = machine.Pin(25, machine.Pin.OUT)

cspin1 = machine.Pin(13, machine.Pin.OUT)
dio01 = machine.Pin(21, machine.Pin.IN)
rst1 = machine.Pin(28, machine.Pin.OUT)
spi1 = machine.SPI(1,
                  baudrate=1000000,
                  polarity=1,
                  phase=1,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=machine.Pin(10),
                  mosi=machine.Pin(11),
                  miso=machine.Pin(12))

while True:
    
    if lora1.begin(spi1, cspin1, rst1, dio01, 433):
        #print("LoRa second  OK")
        break
    utime.sleep(1)


def calibrate_gyroscope(i2c, samples=100):
    gyro_offsets = {'x': 0, 'y': 0, 'z': 0}

    print("Calibrating gyroscope... Keep the device steady!")
    for _ in range(samples):
        try:
            data = get_mpu6050_data(i2c)
        except:
            samples+=1
            continue
        gyro_offsets['x'] += data['gyro']['x']
        gyro_offsets['y'] += data['gyro']['y']
        gyro_offsets['z'] += data['gyro']['z']
        utime.sleep(0.01)  # Small delay between readings

    # Average the readings
    gyro_offsets['x'] /= samples
    gyro_offsets['y'] /= samples
    gyro_offsets['z'] /= samples

    print("Gyroscope calibration complete.")
    print("Offsets -> X: {:.2f}, Y: {:.2f}, Z: {:.2f}".format(
        gyro_offsets['x'], gyro_offsets['y'], gyro_offsets['z']
    ))
    
    if gyro_offsets['z'] > 0:
        gyro_offsets['z']= float(math.ceil(gyro_offsets['z']))  # Round up for positive values
    elif gyro_offsets['z'] < 0:
        gyro_offsets['z']= float(math.floor(gyro_offsets['z']))  # Round down for negative values
    else:
        return 0  # Return 0 if the value is 0
    
    return gyro_offsets


# Default PWM values for safety
bcc.duty_u16(initialization_pwm)
bc.duty_u16(initialization_pwm)
fcc.duty_u16(initialization_pwm)
fc.duty_u16(initialization_pwm)
#initialize gyro
i2c = I2C(0, scl=machine.Pin(9), sda=machine.Pin(8), freq=400000)
init_mpu6050(i2c)
gyro_offsets = calibrate_gyroscope(i2c)
yaw=0
prev_time = utime.ticks_ms()


class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, measured_value):
        error = self.setpoint - measured_value
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        # PID formula
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output




pitch_pid = PID(kp=1.0, ki=0.0, kd=0.0, setpoint=-2.5)
roll_pid = PID(kp=0.8, ki=0.0, kd=0.0, setpoint=0.5)#last val was 0.5, left side drift
yaw_pid= PID(kp=3.0, ki=0.0, kd=0.0, setpoint=0)



def calculate_tilt_angles(accel_data):
    x, y, z = accel_data['x'], accel_data['y'], accel_data['z']
 
    tilt_x = math.atan2(y, math.sqrt(x * x + z * z)) * 180 / math.pi
    tilt_y = math.atan2(-x, math.sqrt(y * y + z * z)) * 180 / math.pi 
    return tilt_x, tilt_y

def arm():
    """Arms the motors by setting initial PWM values."""
    global arm_flag,min_pwm,y_mean
    if not arm_flag:
        bcc.duty_u16(min_pwm)
        bc.duty_u16(min_pwm)
        fcc.duty_u16(min_pwm)
        fc.duty_u16(min_pwm)
        arm_flag = True
        y_mean=int(values[11])/4905.0*100

def disarm():
    """Disarms the motors by resetting PWM values."""
    global arm_flag,disarm_pwm
    if arm_flag:
        bcc.duty_u16(disarm_pwm)
        bc.duty_u16(disarm_pwm)
        fcc.duty_u16(disarm_pwm)
        fc.duty_u16(disarm_pwm)
        arm_flag = False
        
        



def thrust():
    
    global values, fcv, fccv, bcv, bccv,pr,gyro_offsets,prev_time,yaw,y_mean,pitch_pid,roll_pid,yaw_pid
    global tilt_stable_x,tilt_stable_y
    # Example: Initialize PID controllers for pitch and roll
    increment=0.5
    if arm_flag:
        with lock:  # Acquire lock before reading values
            try:
                data = get_mpu6050_data(i2c)
                throttle_duty = (int(values[10])/4095.0*1000)+min_pwm
                yaw_duty = int(values[11])/4905.0*100
                pitch_duty = int(values[12])/4095.0*100
                roll_duty = int(values[13])/4095.0*100
                curr_time = utime.ticks_ms()
                dt = (curr_time - prev_time) / 1000  # Time delta in seconds
                gyro_data_corrected = {'z': data['gyro']['z'] - 1,}#gyro_offsets['z'],}
                yaw += gyro_data_corrected['z'] * dt
                yaw = yaw % 360 if yaw >= 0 else (yaw + 360)
                prev_time = curr_time
                error = 0- yaw
                if error > 180:
                    error -= 360  # Shortest path is to go backward
                elif error < -180:
                    error += 360
                tilt_x, tilt_y = calculate_tilt_angles(data['accel'])
                
                
                if  0.3 < tilt_x or tilt_x < -0.3:
                    pitch_correction = pitch_pid.compute(tilt_x)  # Forward/Backward tilt
                else:
                    pitch_correction=0
                    
                
                if  0.3< tilt_y or tilt_y <-0.3:
                    roll_correction = roll_pid.compute(tilt_y)   # Left/Right tilt
                else:
                    roll_correction=0
                #yaw_correction =yaw_pid.compute(error)
                if 0.3 < error or error < -0.3:
                    yaw_correction = yaw_pid.compute(error)  # Apply yaw correction if out of margin
                    #print(yaw_correction,"    ",error)
                else:
                    yaw_correction = 0  # No correction within the margin
                #print((yaw_duty-y_mean) * increment,(pitch_duty-49) * increment,(roll_duty-49) * increment)
                #print(tilt_x,tilt_y,yaw,roll_correction)

            except (ValueError, IndexError) as e:
                #print("Error in thrust input values:", e)
                return  # Abort thrust update if values are invalid
            
        #print(int(throttle_duty))
        # Adjust motor values based on control inputs

        if 100 > pitch_duty > 55:
            fcv  += (pitch_duty-49) * increment
            fccv += (pitch_duty-49) * increment
            bcv  -= (pitch_duty-49) * increment
            bccv -= (pitch_duty-49) * increment
            #print("Backward movement")
        elif 0 <= pitch_duty < 45:
            fcv  += (pitch_duty-49) * increment
            fccv += (pitch_duty-49) * increment
            bcv  -= (pitch_duty-49) * increment
            bccv -= (pitch_duty-49) * increment
            #print("Forward movement")
        '''
        if 100 > yaw_duty > 45:
            fcv  += (yaw_duty-y_mean) * increment
            bcv  += (yaw_duty-y_mean) * increment
            fccv -= (yaw_duty-y_mean) * increment
            bccv -= (yaw_duty-y_mean) * increment
            #print("Left turn")
        elif 0 <= yaw_duty < 40:
            fccv -= (yaw_duty-y_mean) * increment
            bccv -= (yaw_duty-y_mean) * increment
            fcv  += (yaw_duty-y_mean) * increment
            bcv  += (yaw_duty-y_mean) * increment
            #print("Right turn")
        '''
        if 100 > roll_duty > 50:
            fcv += (roll_duty-49) * increment
            bccv += (roll_duty-49) * increment
            fccv -=(roll_duty-49) * increment
            bcv -= (roll_duty-49) * increment
            #print("Right roll")
        elif 0 <= roll_duty < 48:
            fccv -= (roll_duty-49) * increment
            bcv -= (roll_duty-49) * increment
            fcv += (roll_duty-49) * increment
            bccv += (roll_duty-49) * increment
            #print("Left roll")

        fc_output  = max(980, min(max_pwm, throttle_duty - pitch_correction - roll_correction-yaw_correction+fcv))
        fcc_output = max(980, min(max_pwm, throttle_duty - pitch_correction + roll_correction+yaw_correction+fccv))
        bc_output  = max(980, min(max_pwm, throttle_duty + pitch_correction + roll_correction-yaw_correction+bcv))
        bcc_output = max(980, min(max_pwm, throttle_duty + pitch_correction - roll_correction+yaw_correction+bccv))
        #print(fc_output,fcc_output,bc_output,bcc_output)
        try:
            
            fc.duty_u16(int(fc_output))
            fcc.duty_u16(int(fcc_output))
            bc.duty_u16(int(bc_output))
            bcc.duty_u16(int(bcc_output))
            
        except Exception as e:
            #print("Error updating motor PWM values:", e)
            pass
        
        # Reset corrections for next cycle
        fcv = fccv = bcv = bccv = 0



def hold_alt():
    global values, arm_flag, hold_alt_flag, prev_val
    if values[9] == 1:
        hold_alt_flag = True
        prev_val = values[10]  # Capture throttle for altitude hold
        #print("Hold Altitude Activated")
    elif values[4] == 1:
        hold_alt_flag = False
        #print("Altitude Hold Deactivated")
    if values[10] < 1600 and values[10] > 1400 and hold_alt_flag:
        bcc.duty_u16(int(prev_val))
        fcc.duty_u16(int(prev_val))
        bc.duty_u16(int(prev_val)+50)
        fc.duty_u16(int(prev_val))
    ##print(prev_val,arm_flag,capture_alt)





def update_drone():
    """Updates drone state based on received commands."""
    global values
    
    while True:
        try:
            with lock:
                if values[1] == 1:
                    #print("Arming Drone")
                    arm()
                if values[8] == 1:
                    #print("Disarming Drone")
                    disarm()
            thrust()
        except Exception as e:
            #print("Error in update_drone:", e)
            pass
        utime.sleep(0.01)



def receive():
    """Receives data from the LoRa module and updates values."""
    global values
    while True:
        try:
            packet_size = lora1.parsePacket()
            if packet_size > 0:
                payload = lora1.readBuffer()
                # Ensure payload is a bytes object
                if not isinstance(payload, (bytes, bytearray)):
                    payload = bytes(payload)  # Convert to bytes if necessary
                
                # Decode and parse values
                with lock:
                    try:
                        new_values = [int(x) for x in payload.decode('utf-8').strip().split(',')]
                        if len(new_values) == len(values):
                            values = new_values  # Update shared variable
                            #print(values)
                        else:
                            #print("Invalid payload length")
                            continue
                        led_receive.toggle()
                    except (ValueError, TypeError) as e:
                        #print("Parsing Error:", e)  # Log detailed error for debugging
                        pass
        except Exception as e:
            #print("LoRa Communication Error:", e)  # General exception for LoRa communication
            pass
        utime.sleep(0.01)



# Start threads
_thread.start_new_thread(update_drone, ())
receive()  # Main loop for receiving data

