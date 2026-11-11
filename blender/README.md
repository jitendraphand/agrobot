# Blender scenes

`build_scene.py` builds every scene from `cad/export/` (the same STL files
you print), so the renders always match the CAD.

| Scene file | What it is | Render |
|---|---|---|
| `scenes/agrobot_studio.blend` | Robot on a studio backdrop, three area lights | `renders/agrobot_studio.png` |
| `scenes/agrobot_field.blend` | Robot probing soil in a crop row, physical sky | `renders/agrobot_field.png` |
| `scenes/agrobot_exploded.blend` | Printed parts pulled apart | `renders/agrobot_exploded.png` |
| `scenes/agrobot_mission_anim.blend` | The simulated row mission as keyframes (279 frames, 7x time-lapse) | `renders/agrobot_mission_anim.mp4` |

In every scene the robot is a hierarchy of empties named `link_<name>`, one
per moving body (`link_chassis`, `link_turntable`, `link_upper_arm`,
`link_forearm`, `link_tool`, the fingers and the wheels), with the part
meshes parented to them. Pose the robot by rotating those empties about
their joint axes; materials are per part.

The files were saved by Blender 5.0 and render with Cycles on the CPU.

```bash
# inside Blender
blender --background --python blender/build_scene.py -- studio field exploded --render
blender --background --python blender/build_scene.py -- anim --render --speed=7 --fps=20

# or with the bpy Python module
pip install bpy
python blender/build_scene.py studio --render
```

The animation reads `simulation/output/row_mission_traj.npz`, which
`simulation/run_sim.py row` writes: body positions and orientations at 30
frames per second straight from the physics. Nothing in it is hand-animated.
