# MACHINE VISION ROBOT (MVR)
# AUTHORS: TREVOR McCLEEREY, JANSEN QUIROS
# PHYS 124 - FALL 2017
# DESCRIPTION: This program controls a robot that uses an OpenMV camera to track a green object.
#              In the demonstration,  we have chosen a green ball to be its focus. Two servo motors
#              are used for pan and tilt adjustment to center the camera's focus onto the ball. If
#              the ball cannot be focused because it is out of the camera's range, whether it is too
#              far to the right/left or too close/far, it will signal the DC motors to turn or move
#              forward or backwards.


import sensor, image, time
from pyb import Servo   # to initialize and control servos
from pyb import delay   # to use timing delay separate from time module
from pyb import Pin     # to set pin control
from pyb import LED

red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in the green ball in different settings.
# You may pass up to 16 thresholds below. However, it's not really possible to segment any
# Scene with 16 thresholds before color thresholds start to overlap heavily.
thresholds = [(75, 93, -44, -24, -3, 7),   # Green facing desks
              (54, 88, -52, -26, 0, 20),   # Green facing whitboard
              (71, 91, -51, -29, -6, 8)]   # Green facing wall


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)     # Must be turned off for color tracking
sensor.set_auto_whitebal(False) # Must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. Don't set "merge=True" becuase that will merge blobs which we don't want here.


####################################################################
# Servo initialization and calibration here

pan = Servo(1)  # use PWM on P7, s2 on shield
pan.calibration(540, 2400, 540, 1400, 1500)
pan.angle(90)

tilt = Servo(2) # use PWM on P8, s1 on shield
tilt.calibration(540, 2400, 1200, 1800, 1500)
tilt.angle(90)

delay(1000)     # delay to give servos time to get to 90 & 90
#####################################################################

#####################################################################
# DC Motor pin setup

LEFT_MOTOR1 = Pin('P0', Pin.OUT_PP, Pin.PULL_NONE)
LEFT_MOTOR2 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)
LEFT_ENABLE = Pin('P2', Pin.OUT_PP, Pin.PULL_NONE) # HI -> ON;  LOW -> OFF

RIGHT_MOTOR1 = Pin('P3', Pin.OUT_PP, Pin.PULL_NONE)
RIGHT_MOTOR2 = Pin('P4', Pin.OUT_PP, Pin.PULL_NONE)
RIGHT_ENABLE = Pin('P5', Pin.OUT_PP, Pin.PULL_NONE) # HI -> ON; LOW -> OFF
#####################################################################

#####################################################################
# Constants initialization

# Blob sensing functions and constants begin here
# Area related constants and values:
LOW_AREA      = 8000  # likely needs adjustment
HIGH_AREA     = 11000 # likely needs adjustment
MIN_DELTA_A   = 200   # minimum change in area, knock out noise
last_area     = 0     # initialize to zero to start, updates with new data
new_area      = 0     # update in loop by taking measurement of blob

# Translational related constants and values
CENTER_CX             = 150
CENTER_CY             = 115
CENTERED_CX_THRESHOLD = 20
CENTERED_CY_THRESHOLD = 20

last_cx = 0
new_cx  = 0
last_cy = 0
new_cy  = 0

# Servo motor control constants
LOW_ANGLE     = 60
HIGH_ANGLE    = 120
PARALLEL_LOW  = 75
PARALLEL_HIGH = 105
pan_angle     = 90
tilt_angle    = 90
#####################################################################


#####################################################################
# This function checks if the pan servo is in the range near 90 degrees. This must be checked in
# order to move forward/reverse accurately. We only want forward motion if the MVR motion is parallel
# to the motion of the object
# PARAMETER - none
# RETURN - True: when the pan_angle is in the specified range around 90
#          False: when it's not in that range
# SIDE EFFECTS - none
def is_parallel():
    if pan_angle in range( PARALLEL_LOW, PARALLEL_HIGH ):
        blue_led.on()
        return True
    else:
        blue_led.off()
        return False


#####################################################################
# Area sensing function intended to detect a blob's change in area
# A change in area, delta_a, will be positive if the blob is getting further from the camera, and
# negative if the blob is getting closer to the camera. The former will require the DC motors
# to drive forward, the latter in reverse. This will maintain the ball in an acceptable area/distance
# PARAMETERS   - the_blob - a blob object used to measure the blob's area
# RETURN       - returns to caller only if delta_a is negligible; effectively
#                doing nothing. This is in order to eliminate noise in regard to the area
# SIDE EFFECTS - calls functions that drive the DC motors.
def sense_area( the_blob ):
    global diff_area, last_area      # Global allows you to modify these variables in this scope
    new_area  = the_blob.area()       # Get the blob's new area
    delta_a   = last_area - new_area   # Take difference
    last_area = new_area             # Set last area after difference has been taken
    print("area %d" % new_area)

    if is_parallel() and new_area not in range(LOW_AREA, HIGH_AREA):
        if new_area > HIGH_AREA:
            print("        REVERSE")
            reverse()
        elif new_area < LOW_AREA:
            print("FORWARD")
            forward()
    elif is_parallel() and new_area in range(LOW_AREA, HIGH_AREA):
            stop()


#####################################################################
# This function uses the central cx position of 150 (approximately) and takes a measurement of the
# current cx, will update servos depending on how big the difference is from center
#
# PARAMETER - the_blob - a blob object
# RETURN - nothing when the blob object is not far off center
# SIDE EFFECTS - calls pan_adjust when the blob is far off center, in order to bring it back to center
def center_cx( the_blob ):
    off_center = CENTER_CX - the_blob.cx()
    if abs( off_center ) < CENTERED_CX_THRESHOLD:
        return # Close enough to center
    else:
        check_pan_extreme()
        pan_adjust( off_center )


#####################################################################
# This function uses the central cy position of 150 and takes a measurement of the current cy. Then
# it will send the amount that the blob is off center to the servo tilt control to bring it back to
# center.
#
# PARAMETER: the_blob - a blob object
# RETURN - nothing when the blob is near the center of image
# SIDE EFFECTS - calls tiltAdjust when the blob is vertically off center
def center_cy( the_blob ):
    off_center = CENTER_CY - the_blob.cy()  # error value
    if abs( off_center ) < CENTERED_CY_THRESHOLD:
        return # close enough to center
    else:
        check_tilt_extreme()
        tiltAdjust( off_center )


#####################################################################
# this function will check if the pan servo is getting near its maximum range, either high or low
# if the pan is approaching the low extreme ( zero degrees on pan_angle ) then the vehicle needs to
# pivot right. This will aid the servos in keeping a blob centered in the image without reaching
# the max and not being able to turn any more
def check_pan_extreme():
    # Check low extreme
    if pan_angle < LOW_ANGLE:
        green_led.on()
        pivot_right()
    # Check high extreme
    elif pan_angle > HIGH_ANGLE:
        red_led.on()
        pivot_left()
    else:
        green_led.off()
        red_led.off()
        stop() # The pan is not near the extremes


#####################################################################
# The main thing to consider for tilt being at an extreme is probably going to be for an object
# directly under ( or nearly under ) the front of the vehicle. When this is the case, we can bring
# the object into better view by rolling directly backward away from the object. Probably no need
# to bother with the object being near the high extreme, as the vehicle's purpose is to track objects
# on the ground
def check_tilt_extreme():
    # Only check low extreme
    if tilt_angle not in range(LOW_ANGLE, HIGH_ANGLE):
        reverse()
    else:
        stop()

#####################################################################
# This function adjusts the pan servo which will correspond to changes in a blob's x position
# It also ensures that the angle being set does not exceed the maximum range of the servo
# PARAMETER - deltaX - some value that indicates how much the blob has changed its X pos.
# RETURN - none
# SIDE EFFECTS - updates the value of pan_angle and also adjusts the value on pan.angle()
def pan_adjust( deltaX ):
    global pan_angle        # need 'global' to modifiy in this scope
    pan_angle += ( deltaX // 12 )
    # maintain the value of pan_angle between 0 and 180.
    if pan_angle < 0:
        pan_angle = 0
    elif pan_angle > 179:
        pan_angle = 179
    pan.angle(pan_angle) # write the new angle


#####################################################################
# This function adjusts the tilt servo which will correspond to changes in a blob's y position
# It also ensures that the angle being set does not exceed the maximum range of the servo
# PARAMETER - deltaY - some value that indicates how much the blob has changed its Y pos.
# RETURN - none
# SIDE EFFECTS - updates the value of tilt_angle and also adjusts the value on tilt.angle()
def tiltAdjust( deltaY ):
    global tilt_angle      # need 'global' to modifiy in this scope
    tilt_angle -= (deltaY // 12)
        # These checks keep our angle within the allowable range, so it doesn't get messed at the
        # limits. Basically do not allow angles less than 0 or greater than 180
    if tilt_angle < 0:
        tilt_angle = 0
    elif tilt_angle > 149:
        tilt_angle = 149
    tilt.angle(tilt_angle)


#####################################################################
# This function sets both of the enable pins to low, which will stop both motors from doing
# whatever they are doing
# PARAMETER    - none
# RETURN       - none
# SIDE EFFECTS - sets pin2/pin5 to low state
def stop():
    RIGHT_ENABLE.low()
    LEFT_ENABLE.low()


#####################################################################
# This function will pivot the vehicle clockwise, or right from the perspective of the vehicle
# to achieve a pivot motion, the right DC motor will be rolled backward, and the left forward
#
# PARAMETERS   - none
# RETURN       - none
# SIDE EFFECTS - changes direction of wheels in order to pivot vehicle right
def pivot_right():
    left_motor( -1 )
    right_motor( 1 )


#####################################################################
# This function will pivot the vehicle counter clockwise, or left from the perspective of the vehicle
# to achieve a pivot motion, the left DC motor will be rolled backward, and the right forward
#
# PARAMETERS   - none
# RETURN       - none
# SIDE EFFECTS - changes direction of wheels in order to pivot vehicle left
def pivot_left():
    left_motor( 1 )
    right_motor( -1 )


#####################################################################
# This function will turn both DC motors in reverse at the same rate, in order to move the vehicle
# backward.
#
# PARAMETERS   - none
# RETURN       - none
# SIDE EFFECTS - changes direction of wheels in order to pivot vehicle left
def reverse():
    left_motor( -1 )
    right_motor( -1 )


#####################################################################
# This function will turn both DC motors in reverse at the same rate, in order to move the vehicle
# backward.
#
# PARAMETERS   - none
# RETURN       - none
# SIDE EFFECTS - changes direction of wheels in order to move vehicle forward
def forward():
    left_motor( 1 )
    right_motor( 1 )


#####################################################################
# This function switches the hbridge logic levels in order to change the direction
# of the left DC motor on the vehicle. A parameter is passed to this function that
# sets the logic level on the hbridge in order to drive the motor forward or reverse
#
# PARAMETERS   - direction: 1 to turn the wheel backward
#                           -1 to turn the wheel forward
# RETURN       - none
# SIDE EFFECTS - changes the logic level to the pins on the left motor 
def left_motor( direction ):
    if direction < 0: # turn wheel forward
        LEFT_MOTOR1.high()
        LEFT_MOTOR2.low()
    else: # turn wheel backward
        LEFT_MOTOR1.low()
        LEFT_MOTOR2.high()
    LEFT_ENABLE.high()


#####################################################################
# This function switches the hbridge logic levels in order to change the direction
# of the right DC motor on the vehicle. A parameter is passed to this function that
# sets the logic level on the hbridge in order to drive the motor forward or reverse
#
# PARAMETERS   - direction: 1 to turn the wheel backward
#                           -1 to turn the wheel forward
# RETURN       - none
# SIDE EFFECTS - changes the logic level to the pins on the right motor 
def right_motor( direction ):
    if direction < 0:
        RIGHT_MOTOR1.low()
        RIGHT_MOTOR2.high()
    else:
        RIGHT_MOTOR1.high()
        RIGHT_MOTOR2.low()
    RIGHT_ENABLE.high()

while(True):
    clock.tick()
    img = sensor.snapshot()

    # curate a list of blobs that match threshold tuples. merged=True merges blobs that overlap
    # into one blob
    blob_list = img.find_blobs(thresholds, pixels_threshold=100, area_threshold=100, merged=True)
    if len(blob_list) == 0:
        # Only want 1 blob in our image. If more than 1 continue to next loop
        stop()
        continue

    # Set big blob to first element
    bigBlob = blob_list[0]
    # then loop over all blobs to find the largest in the list
    for blob in blob_list:
        if bigBlob.area() < blob.area():
            bigBlob = blob

    # based on the biggest blob in bloblist, center it in the image
    center_cx( bigBlob ) # Keep x coord of blob near 150 degrees
    center_cy( bigBlob ) # Keep y coord of blob near 150 degrees
    sense_area( bigBlob ) # Monitor the distance of the blob
    print( pan_angle )
    
    # Draw box with cross on each blob
    img.draw_rectangle(bigBlob.rect())
    img.draw_cross(bigBlob.cx(), bigBlob.cy())
