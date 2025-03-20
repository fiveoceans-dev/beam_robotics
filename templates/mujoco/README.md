
## Project structure


- 'templates/docker/' - includes initial traiting and inference model
- 'templates/mujoco/' - includes boat.py simulation
- 'templates/unity' - includes pre build unity apps for sim2real over websocket
- 'web/' - includes web UI for sim2real


1. virtualenv mujoco_env

2. source mujoco_env/bin/activate

3. pip install mujoco

4. brew install glfw

5. mjpython template.py