import sys
sys.path.append('./kinematic')
from kinematic.arm5dof_uservo import Arm5DoFUServo

SERVO_PORT = 'COM8'

def main():
    servo_manager = Arm5DoFUServo(device=SERVO_PORT, is_init_pose= False)
    servo_manager.home()
    # servo_manager.set_damping(1000)


if __name__ == "__main__":
    main()