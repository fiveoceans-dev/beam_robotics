import mujoco
import mujoco.viewer
import numpy as np
import time

# Load MuJoCo car model
model = mujoco.MjModel.from_xml_path("./model/hero/car.xml")
data = mujoco.MjData(model)

# Get actuator indices
move = model.actuator("forward").id
steer = model.actuator("steer").id

def apply_controls(throttle=1.0, steering=0.2):
    """ Moves the car forward and turns the front wheels. """
    data.ctrl[move] = throttle  # Forward movement
    data.ctrl[steer] = steering  # Turning

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation started. Car is moving forward and steering.")

    while viewer.is_running():
        apply_controls(throttle=1.0, steering=0.5)  # Move forward & steer slightly

        mujoco.mj_step(model, data)

        car_pos = data.qpos[:3]  # Get car's position

        # Update camera to follow car
        viewer.cam.lookat[:] = car_pos
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 0
        viewer.cam.elevation = -20

        viewer.sync()
        # time.sleep(0.01)

print("Simulation ended.")
