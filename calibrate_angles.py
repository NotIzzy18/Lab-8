import math
from machine import Pin, PWM, ADC

def translate(angle: float) -> int:
	"""
	Converts an angle in degrees to the corresponding input
	for the duty_u16 method of the servo class

	See https://docs.micropython.org/en/latest/library/machine.PWM.html for more
	details on the duty_u16 method
	"""

	MIN = 1638 # 0 degrees
	MAX = 8192 # 180 degrees
	DEG = (MAX - MIN) / 180 # value per degree

	# clamp angle to be between 0 and 180
	angle = max(0, min(180, angle))

	return int(angle * DEG + MIN)

def inverse_kinematics(end_position_x, end_position_y):
    # Reminder: 1 cm = 10 mm
    # Servo Domain Restrictions (when y = 0): x >= 226.5mm --> 22.65 cm
    # Servo Range Restriction (when x = 0): y >= 445.4mm --> 44.54 cm
    
    La = 155 # length from shoulder to elblow = 155 mm
    Lb = 155 # length from elbow to wrist = 155 mm
    
    end_position_x = float(end_position_x)
    end_position_y = float(end_position_y)
    
    shoulder_position = [-50.0, 139.5] # coordinates of shoulder position if origin is bottom left corner of page
    
    end_position = math.sqrt(((shoulder_position[0] - end_position_x)**2.0) + (shoulder_position[1] - end_position_y)**2.0)
    xaxis_end_position = math.sqrt(((shoulder_position[0] - end_position_x)**2.0) + (end_position_y)**2.0)
    
    angle_bac = math.acos(((La)**2 + (end_position)**2 - (Lb)**2)/(2*La*end_position)) # b = elbow; a = shoulder; c = wrist
    angle_acb = math.asin(((La)*(math.sin(angle_bac)))/(Lb)) # a = shoulder; c = wrist; b = elbow
    angle_yac = math.acos(((shoulder_position[1])**2 + (end_position)**2 - (xaxis_end_position)**2)/(2*(shoulder_position[1])*(end_position))) # y = y-axis; a = shoulder; c = wrist;
    
    alpha = math.degrees(angle_bac) + math.degrees(angle_yac) # alpha = angle for shoulder
    beta = math.degrees(angle_bac) + math.degrees(angle_acb) # beta = angle for elbow
    
    resulting_angles = [alpha, beta]
    
    return (resulting_angles)

def callibration_setup(txt):
    with open(txt, "r+") as file:
        data = file.read()
        data = data.split()
        
        x = 0 # initalize variable for index
        elbow_angles = {} # initalize dict for elbow where key is desired angle and value is actual angle
        shoulder_angles = {} # initalize dict for elbow where key is desired angle and value is actual angle
        
        for i in data:
            if data[x] == "E": # if set of angles is from elbow calibration...
                elbow_angles[int(data[x+1])] = int(data[x+2]) # add following angles to dict where key is desired angle and value is actual angle
                x+=1
            elif data[x] == "S": # if set of angles is from shoulder calibration...
                shoulder_angles[int(data[x+1])] = int(data[x+2]) # add following angles to dict where key is desired angle and value is actual angle
                x+=1
            else: # ignores actual angles 
                x+=1
                pass
    return (elbow_angles, shoulder_angles)

def send_compensated_angle(joint, desired_angle):
    elbow_angles = callibration_setup("calibration_data_2.txt")[0] # ensures that dict of angles is in proper variable (index 0 = elbow angles); reads calibration data
    shoulder_angles = callibration_setup("calibration_data_2.txt")[1] # ensures that dict of angles is in proper variable (index 1 = shoulder angles); reads calibration data
    
    lower_key = (desired_angle // 10)*10 # division by 10 as a whole number will give lower key, since table is in 10 degree increments
    upper_key = lower_key + 10 # adding 10 to lower key will result in the next key, since 10 degree increments
    
    if joint == "e": # if joint used is elbow...
        keys = elbow_angles.keys() # initializes list to make seeing whether or not desired angle is a multitple of 10 easier
        if desired_angle in keys: # if desired angle is a multiple of 10, less math needed since error is just difference of desired and actual angles from dict
            error = desired_angle - elbow_angles[desired_angle]
            calibrated_angle = desired_angle + error
        else: # interpolation formula
            calibrated_angle = float(elbow_angles[lower_key]) + ((desired_angle - lower_key)*(float(elbow_angles[upper_key]) - float(elbow_angles[lower_key]))/(upper_key - lower_key))
    elif joint == "s": # if joint used is shoulder...
        keys = shoulder_angles.keys() # initializes list to make seeing whether or not desired angle is a multitple of 10 easier
        if desired_angle in keys: # if desired angle is a multiple of 10, less math needed since error is just difference of desired and actual angles from dict
            error = desired_angle - shoulder_angles[desired_angle]
            calibrated_angle = desired_angle + error
        else: # interpolation formula          
            calibrated_angle = float(shoulder_angles[lower_key]) + ((desired_angle - lower_key)*(float(shoulder_angles[upper_key]) - float(shoulder_angles[lower_key]))/(upper_key - lower_key))
    return calibrated_angle

# shoulder = GP0; elbow = GP1; wrist = GP2
# test jig ID: #2

print(send_compensated_angle("s", inverse_kinematics(50, 50)[0]))
print(send_compensated_angle("e", inverse_kinematics(50, 50)[1]))

shoulder = PWM(Pin(0), freq = 50)
shoulder.duty_u16(translate(send_compensated_angle("s", inverse_kinematics(50, 50)[0])))

elbow = PWM(Pin(1), freq = 50)
elbow.duty_u16(translate(send_compensated_angle("e", inverse_kinematics(50, 50)[1])))