import mujoco
import mujoco.viewer
import numpy as np
import time

# Load MuJoCo model
model = mujoco.MjModel.from_xml_path("./model/hero/hero.xml")
data = mujoco.MjData(model)

# Get actuator indices
move_x = model.actuator("move_x").id
move_y = model.actuator("move_y").id
move_z = model.actuator("move_z").id

def apply_controls(force_x=0, force_y=0, force_z=0):
    """ Moves the hero in X, Y, Z directions using force. """
    data.ctrl[move_x] = force_x
    data.ctrl[move_y] = force_y
    data.ctrl[move_z] = force_z

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation started. Move freely in X, Y, Z directions.")

    while viewer.is_running():
        apply_controls(force_x=0, force_y=1, force_z=0)

        mujoco.mj_step(model, data)

        hero_pos = data.qpos[:3]

        # Update camera to follow hero
        viewer.cam.lookat[:] = hero_pos
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 0
        viewer.cam.elevation = -20

        viewer.sync()
        time.sleep(0.01)  # Simulate real-time movement

print("Simulation ended.")
