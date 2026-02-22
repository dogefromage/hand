

# cool commands

# build
```bash
cd ros2_ws
colcon build --symlink
source install/local_setup.bash
```

### publish state (no visualization)

```bash
ros2 launch hand_description publish_state.launch.py # start state publishing and sliders
```
