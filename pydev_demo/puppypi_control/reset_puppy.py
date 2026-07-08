import time
import numpy as np

from servo_controller import setServoPulse
from HiwonderPuppy import HiwonderPuppy, PWMServoParams

print('Hiwonder')
print('🔄 Puppy reset start...')

puppy = HiwonderPuppy(
    setServoPulse=setServoPulse,
    servoParams=PWMServoParams(),
    dof='8'
)


def get_stance(x=0, y=0, z=-10, x_shift=-0.5):
    return np.array([
        [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
        [y, y, y, y],
        [z, z, z, z],
    ])


try:
    puppy.servo_force_run()
except Exception as e:
    print(f'servo_force_run failed: {e}')

try:
    puppy.move_stop(servo_run_time=300)
except Exception as e:
    print(f'move_stop failed: {e}')

time.sleep(0.5)

try:
    puppy.stance_config(
        get_stance(x=0, y=0, z=-10, x_shift=-0.5),
        pitch=0,
        roll=0
    )
except Exception as e:
    print(f'stance_config failed: {e}')

try:
    puppy.gait_config(
        overlap_time=0.2,
        swing_time=0.3,
        clearance_time=0.0,
        z_clearance=5
    )
except Exception as e:
    print(f'gait_config failed: {e}')

try:
    puppy.start()
except Exception as e:
    print(f'start failed: {e}')

time.sleep(0.5)

try:
    puppy.move_stop(servo_run_time=500)
except Exception as e:
    print(f'final move_stop failed: {e}')

print('✅ Puppy reset done. Robot should be back to controllable standing state.')
