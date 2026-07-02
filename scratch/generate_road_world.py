import numpy as np
import os

# Define file paths
dest_dir = "/home/aosm/Mono Camera Depth Autonomuos Car/src/autonomous_car/worlds"
os.makedirs(dest_dir, exist_ok=True)
dest_path = os.path.join(dest_dir, "road_world.world")

# Generate road points (oval track)
# Center at (0, 0).
# Bottom straight: y = -10, x from -15 to 15
# Right curve: semi-circle centered at (15, 0) of radius 10, theta from -pi/2 to pi/2
# Top straight: y = 10, x from 15 to -15
# Left curve: semi-circle centered at (-15, 0) of radius 10, theta from pi/2 to 3*pi/2

points = []

# 1. Bottom straight
for x in np.linspace(-15, 15, 20, endpoint=False):
    points.append((x, -10.0))

# 2. Right curve
for theta in np.linspace(-np.pi/2, np.pi/2, 15, endpoint=False):
    x = 15.0 + 10.0 * np.cos(theta)
    y = 10.0 * np.sin(theta)
    points.append((x, y))

# 3. Top straight
for x in np.linspace(15, -15, 20, endpoint=False):
    points.append((x, 10.0))

# 4. Left curve
for theta in np.linspace(np.pi/2, 3*np.pi/2, 15, endpoint=False):
    x = -15.0 + 10.0 * np.cos(theta)
    y = 10.0 * np.sin(theta)
    points.append((x, y))

n_points = len(points)
road_width = 4.0
barrier_length = 2.4

# Calculate left and right curbs along the road normal vector
left_curbs = []
right_curbs = []

for i in range(n_points):
    p_prev = np.array(points[(i - 1) % n_points])
    p_curr = np.array(points[i])
    p_next = np.array(points[(i + 1) % n_points])
    
    # Tangent vector
    tangent = p_next - p_prev
    norm_t = np.linalg.norm(tangent)
    if norm_t > 0:
        tangent = tangent / norm_t
    else:
        tangent = np.array([1.0, 0.0])
        
    # Normal vector (90 deg CCW)
    normal = np.array([-tangent[1], tangent[0]])
    
    # Curb positions
    p_left = p_curr + (road_width / 2.0 + 0.1) * normal
    p_right = p_curr - (road_width / 2.0 + 0.1) * normal
    
    # Yaw angle
    yaw = np.arctan2(tangent[1], tangent[0])
    
    left_curbs.append((p_left[0], p_left[1], yaw))
    right_curbs.append((p_right[0], p_right[1], yaw))

# Build World XML
xml_lines = []
xml_lines.append('<?xml version="1.0" ?>')
xml_lines.append('<sdf version="1.6">')
xml_lines.append('  <world name="road_world">')

# 1. Scene settings with realistic weather (fog/clouds/ambient)
xml_lines.append('    <scene>')
xml_lines.append('      <sky>')
xml_lines.append('        <time>16.0</time>')
xml_lines.append('        <clouds>')
xml_lines.append('          <speed>8.0</speed>')
xml_lines.append('          <humidity>0.7</humidity>')
xml_lines.append('          <mean_size>0.5</mean_size>')
xml_lines.append('          <ambient>0.7 0.7 0.7 1.0</ambient>')
xml_lines.append('        </clouds>')
xml_lines.append('      </sky>')
xml_lines.append('      <fog>')
xml_lines.append('        <color>0.7 0.75 0.8 1.0</color>')
xml_lines.append('        <type>linear</type>')
xml_lines.append('        <start>2.0</start>')
xml_lines.append('        <end>25.0</end>')
xml_lines.append('        <density>0.08</density>')
xml_lines.append('      </fog>')
xml_lines.append('      <ambient>0.6 0.6 0.6 1.0</ambient>')
xml_lines.append('      <background>0.7 0.75 0.8 1.0</background>')
xml_lines.append('      <shadows>true</shadows>')
xml_lines.append('    </scene>')
xml_lines.append('')

# 2. Lighting
xml_lines.append('    <include>')
xml_lines.append('      <uri>model://sun</uri>')
xml_lines.append('    </include>')
xml_lines.append('')

# 3. Grass ground plane (Collision & Background visual)
xml_lines.append('    <model name="grass_ground_plane">')
xml_lines.append('      <static>true</static>')
xml_lines.append('      <link name="link">')
xml_lines.append('        <collision name="collision">')
xml_lines.append('          <geometry>')
xml_lines.append('            <plane>')
xml_lines.append('              <normal>0 0 1</normal>')
xml_lines.append('              <size>150 150</size>')
xml_lines.append('            </plane>')
xml_lines.append('          </geometry>')
xml_lines.append('          <surface>')
xml_lines.append('            <contact>')
xml_lines.append('              <collide_bitmask>0xffff</collide_bitmask>')
xml_lines.append('            </contact>')
xml_lines.append('            <friction>')
xml_lines.append('              <ode>')
xml_lines.append('                <mu>100</mu>')
xml_lines.append('                <mu2>50</mu2>')
xml_lines.append('              </ode>')
xml_lines.append('            </friction>')
xml_lines.append('          </surface>')
xml_lines.append('        </collision>')
xml_lines.append('        <visual name="visual">')
xml_lines.append('          <cast_shadows>false</cast_shadows>')
xml_lines.append('          <geometry>')
xml_lines.append('            <plane>')
xml_lines.append('              <normal>0 0 1</normal>')
xml_lines.append('              <size>150 150</size>')
xml_lines.append('            </plane>')
xml_lines.append('          </geometry>')
xml_lines.append('          <material>')
xml_lines.append('            <script>')
xml_lines.append('              <uri>file://media/materials/scripts/gazebo.material</uri>')
xml_lines.append('              <name>Gazebo/Grass</name>')
xml_lines.append('            </script>')
xml_lines.append('          </material>')
xml_lines.append('        </visual>')
xml_lines.append('      </link>')
xml_lines.append('    </model>')
xml_lines.append('')

# 4. The road visual model with road markings
xml_lines.append('    <road name="racetrack">')
xml_lines.append(f'      <width>{road_width}</width>')
xml_lines.append('      <material>')
xml_lines.append('        <script>')
xml_lines.append('          <uri>model://autonomous_car/media/materials/scripts/road.material</uri>')
xml_lines.append('          <name>Road/Lanes</name>')
xml_lines.append('        </script>')
xml_lines.append('      </material>')
for x, y in points:
    xml_lines.append(f'      <point>{x:.3f} {y:.3f} 0.01</point>')
# Close the loop by repeating the first point
xml_lines.append(f'      <point>{points[0][0]:.3f} {points[0][1]:.3f} 0.01</point>')
xml_lines.append('    </road>')
xml_lines.append('')

# 5. Left (inward) curbs with alternating red/white colors
for i, (x, y, yaw) in enumerate(left_curbs):
    color = "Gazebo/Red" if i % 2 == 0 else "Gazebo/White"
    xml_lines.append(f'    <model name="left_curb_{i}">')
    xml_lines.append('      <static>true</static>')
    xml_lines.append(f'      <pose>{x:.3f} {y:.3f} 0.15 0 0 {yaw:.3f}</pose>')
    xml_lines.append('      <link name="link">')
    xml_lines.append('        <collision name="collision">')
    xml_lines.append('          <geometry>')
    xml_lines.append(f'            <box><size>{barrier_length} 0.2 0.3</size></box>')
    xml_lines.append('          </geometry>')
    xml_lines.append('        </collision>')
    xml_lines.append('        <visual name="visual">')
    xml_lines.append('          <geometry>')
    xml_lines.append(f'            <box><size>{barrier_length} 0.2 0.3</size></box>')
    xml_lines.append('          </geometry>')
    xml_lines.append('          <material>')
    xml_lines.append('            <script>')
    xml_lines.append('              <uri>file://media/materials/scripts/gazebo.material</uri>')
    xml_lines.append(f'              <name>{color}</name>')
    xml_lines.append('            </script>')
    xml_lines.append('          </material>')
    xml_lines.append('        </visual>')
    xml_lines.append('      </link>')
    xml_lines.append('    </model>')

# 6. Right (outward) curbs with alternating red/white colors
for i, (x, y, yaw) in enumerate(right_curbs):
    color = "Gazebo/Red" if i % 2 == 0 else "Gazebo/White"
    xml_lines.append(f'    <model name="right_curb_{i}">')
    xml_lines.append('      <static>true</static>')
    xml_lines.append(f'      <pose>{x:.3f} {y:.3f} 0.15 0 0 {yaw:.3f}</pose>')
    xml_lines.append('      <link name="link">')
    xml_lines.append('        <collision name="collision">')
    xml_lines.append('          <geometry>')
    xml_lines.append(f'            <box><size>{barrier_length} 0.2 0.3</size></box>')
    xml_lines.append('          </geometry>')
    xml_lines.append('        </collision>')
    xml_lines.append('        <visual name="visual">')
    xml_lines.append('          <geometry>')
    xml_lines.append(f'            <box><size>{barrier_length} 0.2 0.3</size></box>')
    xml_lines.append('          </geometry>')
    xml_lines.append('          <material>')
    xml_lines.append('            <script>')
    xml_lines.append('              <uri>file://media/materials/scripts/gazebo.material</uri>')
    xml_lines.append(f'              <name>{color}</name>')
    xml_lines.append('            </script>')
    xml_lines.append('          </material>')
    xml_lines.append('        </visual>')
    xml_lines.append('      </link>')
    xml_lines.append('    </model>')

# 7. Obstacles on the road (Orange cones and grey barriers)
obstacles = [
    # Bottom straight (left lane, right lane)
    {"name": "cone_bottom_1", "type": "cone", "x": 0.0, "y": -9.0, "z": 0.25},
    {"name": "cone_bottom_2", "type": "cone", "x": 8.0, "y": -11.0, "z": 0.25},
    # Right curve
    {"name": "barrier_right", "type": "barrier", "x": 23.5, "y": 0.0, "z": 0.4},
    # Top straight (left lane, right lane)
    {"name": "cone_top_1", "type": "cone", "x": 0.0, "y": 11.0, "z": 0.25},
    {"name": "cone_top_2", "type": "cone", "x": -8.0, "y": 9.0, "z": 0.25},
    # Left curve
    {"name": "barrier_left", "type": "barrier", "x": -23.5, "y": 0.0, "z": 0.4},
]

for obs in obstacles:
    xml_lines.append(f'    <model name="{obs["name"]}">')
    xml_lines.append('      <static>true</static>')
    if obs["type"] == "cone":
        xml_lines.append(f'      <pose>{obs["x"]} {obs["y"]} {obs["z"]} 0 0 0</pose>')
        xml_lines.append('      <link name="link">')
        xml_lines.append('        <collision name="collision">')
        xml_lines.append('          <geometry><cylinder><radius>0.15</radius><length>0.5</length></cylinder></geometry>')
        xml_lines.append('        </collision>')
        xml_lines.append('        <visual name="visual">')
        xml_lines.append('          <geometry><cylinder><radius>0.15</radius><length>0.5</length></cylinder></geometry>')
        xml_lines.append('          <material>')
        xml_lines.append('            <script>')
        xml_lines.append('              <uri>file://media/materials/scripts/gazebo.material</uri>')
        xml_lines.append('              <name>Gazebo/Orange</name>')
        xml_lines.append('            </script>')
        xml_lines.append('          </material>')
        xml_lines.append('        </visual>')
        xml_lines.append('      </link>')
    else: # barrier
        xml_lines.append(f'      <pose>{obs["x"]} {obs["y"]} {obs["z"]} 0 0 0</pose>')
        xml_lines.append('      <link name="link">')
        xml_lines.append('        <collision name="collision">')
        xml_lines.append('          <geometry><box><size>0.6 0.6 0.8</size></box></geometry>')
        xml_lines.append('        </collision>')
        xml_lines.append('        <visual name="visual">')
        xml_lines.append('          <geometry><box><size>0.6 0.6 0.8</size></box></geometry>')
        xml_lines.append('          <material>')
        xml_lines.append('            <script>')
        xml_lines.append('              <uri>file://media/materials/scripts/gazebo.material</uri>')
        xml_lines.append('              <name>Gazebo/Grey</name>')
        xml_lines.append('            </script>')
        xml_lines.append('          </material>')
        xml_lines.append('        </visual>')
        xml_lines.append('      </link>')
    xml_lines.append('    </model>')

# Physics
xml_lines.append('    <physics type="ode">')
xml_lines.append('      <real_time_update_rate>1000.0</real_time_update_rate>')
xml_lines.append('      <max_step_size>0.001</max_step_size>')
xml_lines.append('      <real_time_factor>1.0</real_time_factor>')
xml_lines.append('    </physics>')
xml_lines.append('  </world>')
xml_lines.append('</sdf>')

# Write to file
with open(dest_path, "w") as f:
    f.write("\n".join(xml_lines))

print(f"Generated {dest_path} successfully with {n_points} road points and curbs.")
