# boat.py
import mujoco
import mujoco.viewer
import numpy as np
import time

# Load MuJoCo boat model
model = mujoco.MjModel.from_xml_path("./model/hero/boat.xml")
data = mujoco.MjData(model)

# Get actuator indices (thruster)
left_thruster = model.actuator("left_motor").id
right_thruster = model.actuator("right_motor").id

# Get body IDs
boat_body_id = model.body("boat").id

# Water properties
WATER_LEVEL = 0.3 
BUOYANCY_FORCE = 9.81
DRAG_COEFFICIENT = 0.05

def apply_buoyancy_and_drag():
    # Get boat position
    boat_pos = data.xpos[boat_body_id]
    velocity = data.qvel[:3]

    if boat_pos[2] < WATER_LEVEL:
        depth = WATER_LEVEL - boat_pos[2]
        data.xfrc_applied[boat_body_id, 2] += BUOYANCY_FORCE * depth

        drag_force = -DRAG_COEFFICIENT * np.sign(velocity) * (velocity**2)
        data.xfrc_applied[boat_body_id, :3] += drag_force

def apply_controls(left_power, right_power):
    data.ctrl[left_thruster] = left_power
    data.ctrl[right_thruster] = right_power

def control_boat(direction):
    if direction == "FORWARD":
        apply_controls(0.5, 0.5)
    elif direction == "BACKWARD":
        apply_controls(-0.5, -0.5)
    elif direction == "LEFT":
        apply_controls(-0.5, 0.5)
    elif direction == "RIGHT":
        apply_controls(0.5, -0.5)
    else:
        apply_controls(0, 0)

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Simulation started. Propellers are spinning.")

    while viewer.is_running():
        # Apply propeller thrust and buoyancy
        apply_controls(left_power=0.5, right_power=-0.5)
        apply_buoyancy_and_drag()

        mujoco.mj_step(model, data)
        boat_pos = data.xpos[boat_body_id]

        # Update camera to follow the boat
        viewer.cam.lookat[:] = boat_pos
        viewer.cam.distance = 10.0
        viewer.cam.azimuth = 0
        viewer.cam.elevation = -90
        viewer.cam.trackbodyid = boat_body_id

        viewer.sync()
        time.sleep(0.01)

print("Simulation ended.")
