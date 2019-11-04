import drive
import time

drive.init()
drive.full_reset_and_calibrate_all()

print("Starting test loop")
while True:
    drive.axis_states()
    drive.flipper_position(8192, 0)
    time.sleep(0.25)
    drive.flipper_position(0, 0)
    time.sleep(0.25)
    drive.flipper_position(-8192, 0)
    time.sleep(0.25)
    drive.flipper_position(0, 0)
    time.sleep(0.25)
