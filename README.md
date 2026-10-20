# Moonwalk 2.0 · Report 04

From the plan to the robot. Scrolling research site for Moonwalk 2.0, towards a permanent human presence on the Moon.
Space Robotics Lab, Tohoku University.

**Contents**
- Site: zooming tour from the whole Moon to the south pole with elevation, illumination and cold-trap layers
- Landing zone: ispace Mission 4 latitude (≥ 87°S) drawn against the four candidate sites
- Assembly methodology: from the block list to the robot's job list
- Pad: 283 pavers set centre-out by one MoonBot Dragon; technical drawing, 3D player, video, job list
- Dome: ShellA at 0.6× built from inside through its own doorway; every drive checked for collisions with set blocks
- 4D BIM and the robot operations framework (IFC mapping, five layers)
- Simulation methodology: Grasshopper → plan files → ROS 2 Motion Stack → Isaac Sim
- Literature review (BIM ↔ robotics, dry-stacked assembly, lunar construction) and conference deadlines

**Data** (`data/`): `pad_assembly_plan.csv`, `dome_assembly_plan.csv` (one row per block: step, block, ring/course, stock stack, pick and place station, drive, start/end time) and the full plan + timeline JSON used by the 3D players.

Live site: https://arhimatrix.github.io/arhimatrix-moonwalk-2-rep_04/
Report 03: https://arhimatrix.github.io/arhimatrix-moonwalk-2-rep_03/
Report 02: https://arhimatrix.github.io/arhimatrix-moonwalk-2-rep_02/

Robot model: MoonBot Dragon (dragon_v2), Space Robotics Lab. Moon surface map: Celestia (Clementine/NASA data). 3D: three.js r128.

© Nina Velimirović
