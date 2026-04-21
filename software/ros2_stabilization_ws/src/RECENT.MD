What was built
rolling_shutter_node.py
New publisher /camera/raw/compressed — publishes the raw frame after the lag buffer pop, so it's temporally aligned with the RS-corrected output. Used by the comparison node.
yaw_stabilizer.py — full redesign
Replaced 30-frame moving average with a proper control loop:

Parameter	Default	Effect
reference_alpha	0.02	EMA drift rate — lower = slower reference, more jitter correction
p_gain	1.0	Scale the fisheye-derived correction (try 0.8 if overcorrecting)
d_gain	0.0	Damping on correction rate-of-change; increase if you see oscillation
max_margin_px	80	Crop headroom — tanh soft clamping saturates smoothly at edges
out_w	960	Output width (4:3 at 720h)
Start with reference_alpha:=0.01 if you want a tighter hold, 0.05 if you want more pan pass-through.

demo_comparison_node.py — new node
Subscribes to all three pipeline stages, publishes:

/demo_comparison/compressed — 2-panel: raw | yaw stabilized (both 4:3, same size)
/demo_full_pipeline/compressed — 3-panel: raw | RS corrected | yaw stabilized (half-size to fit monitor)
Launch files
File	Viewers	Annotations
demo_rs_yaw_launch.py	/rs_comparison + /demo_full_pipeline	On (per-node control: rs_annotations, yaw_annotations, cmp_annotations)
demo_rs_yaw_clean_launch.py	/demo_comparison only	All off
Demo commands:


# Annotated (diagnostics + 3-panel view)
ros2 launch stabilization_pkg demo_rs_yaw_launch.py

# Clean recording
ros2 launch stabilization_pkg demo_rs_yaw_clean_launch.py

# Tune stabilizer live
ros2 launch stabilization_pkg demo_rs_yaw_launch.py reference_alpha:=0.01 p_gain:=0.9 d_gain:=0.15