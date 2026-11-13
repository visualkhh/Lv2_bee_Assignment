# Jetson low-memory operation profile

A saved 2D map can replace nvblox during localization, but **no current real-robot map in this workspace has been validated for this mode**. The `kk` map is old; the user selected `scene_sim` as the working map. `scene_sim` is generated from the simulation scene, and physical alignment has not yet been verified. A map-server smoke test only proved that `kk/map.yaml` loads; it did not prove coordinate alignment or safe robot travel. Never select the most recent map automatically for a physical run.

`low_memory.launch.py` defaults to `map:=none`, `map_backend:=nvblox`, OpenCV one thread, and YOLO/tag inference at 1 Hz while `/skills/navigate` is active. The ordinary launch defaults remain unchanged. Specify a map operation explicitly.

To make a new **real** map with reduced scheduled nvblox work, after rebuilding while the robot is stopped:

```sh
cd /home/ghj/Lv2_bee_Assignment/lv2_module6/ros2_ws
LAUNCH_FILE=low_memory.launch.py ./start.sh map:=slam map_backend:=nvblox
```

The lightweight nvblox config integrates depth at 5 Hz and ESDF at 2 Hz, and disables mesh updates. It keeps full-map retention; map memory can still grow. Check `/map` coverage and obstacle latency on the robot before relying on the profile.

For static 2D localization, use the selected `scene_sim` map explicitly. Its `start_pose.png` defines the map origin at the robot start, facing +X toward the signal light. With the robot stopped, check the actual start pose and wall alignment against that image before autonomous travel:

```sh
LAUNCH_FILE=low_memory.launch.py ./start.sh map:=saved map_backend:=static_2d map_name:=scene_sim
```

Static mode keeps cuVSLAM and the filtered camera point-cloud obstacle layers, and skips nvblox, its depth gate, and the map saver. The launch rejects missing/invalid map files and omitted map names. Loading `scene_sim` proves neither real wall alignment nor safe travel; check robot pose and obstacles in RViz before autonomous motion.

Run `resource_watch.py` from pa27 for a before/after log that survives Jetson resets (see `README-resource-watch.md`). Compare available RAM, swap traffic, CPU, map availability, obstacle updates, and control timing. Python launch/config and the C++ perception node need a workspace rebuild before they affect runtime. Do not install only launch files on top of an old perception binary. Source and isolated tests are complete; moving-robot validation remains outstanding.
