import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image

# 1. Load Map and Parameters
map_path = '/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/robot/maps/INB-3floor/submap_0.pgm'
resolution = 0.025
origin = [-14.731968, -13.009910, 0.0]
# SCENARIO = "poster-baseline"
# SCENARIO = "poster-causal"
SCENARIO = "buffet-baseline"
# SCENARIO = "buffet-causal"

try:
    img = Image.open(map_path)
    img_arr = np.array(img)
    
    # Calculate Extent [xmin, xmax, ymin, ymax]
    # Width * Res + OriginX
    height, width = img_arr.shape
    xmin = origin[0]
    xmax = origin[0] + width * resolution
    ymin = origin[1]
    ymax = origin[1] + height * resolution
    extent = [xmin, xmax, ymin, ymax]
    
    # ROS map_server usually requires flipping the image vertically for display
    # because images are indexed from top-left, but map coordinates are bottom-left.
    img_arr = np.flipud(img_arr)
    
except Exception as e:
    print(f"Error loading map: {e}")
    img_arr = None

# 1. Setup Waypoints
if SCENARIO == "poster-baseline":
    waypoints = {
        "N": (25.55, 3.82),
        "inb3241": (21.53, 2.91),
        "poster1": (19.38, 2.53),
        "poster2": (16.73, 2.17),
        "tmp": (21.32, 1.98),
        "D": (21.9, 0.5)
    }
    path_sequence = ["N", "inb3241", "poster1", "poster2", "poster1", "tmp", "D"]
elif SCENARIO == "poster-causal":
    waypoints = {
        "N": (25.55, 3.82),
        "O": (26.5, -0.75),
        "E": (26.8, -3.96),
        "D": (20.9, -4.88),
        "C": (10.5, -6),
        "CR": (10.1, -3.12),
        "R": (9.7, 0.234),
        "S": (9.55, 1.17),
        "poster5": (6.10, 0.74),
        "inb3238": (4.37, 0.41),
    }
    path_sequence = ["N", "O", "E", "D", "C", "CR", "R", "S", "poster5", "inb3238"]
elif SCENARIO == "buffet-baseline":
    waypoints = {
        "inb3241": (21.53, 2.91),
        "N": (25.55, 3.82),
        "tmp": (26.07, 1.72),
    }
    path_sequence = ["inb3241", "N", "tmp"]
elif SCENARIO == "buffet-causal":
    waypoints = {
        "inb3241": (21.53, 2.91),
        "N": (25.55, 3.82),
        "M": (27.68, 4.94),
        "LM": (30.62, 5.54),
        "L": (34.23, 6.2),
        "I": (36.16, 4.33),
        "H": (37.75, -2.06),
        "F": (31.84, -3.24),
        "G": (31.55, -1.18),
    }
    path_sequence = ["inb3241", "N", "M", "LM", "L", "I", "H", "F", "G"]



# 2. Parameters
v_max = 0.5   # m/s
accel = 0.25  # m/s^2
dt = 0.1      # 10hz

def get_dist(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def generate_stop_go_leg(start_pos, end_pos, start_time, leg_name):
    dist_total = get_dist(start_pos, end_pos)
    
    # Required distance to reach max speed and stop: (v^2/2a) * 2
    d_ramp_theoretical = (v_max**2) / (2 * accel)
    
    if dist_total < (2 * d_ramp_theoretical):
        # Triangle Profile
        v_peak = np.sqrt(dist_total * accel) # derived from d = 2 * (0.5 * v/a * v/2) ?? No, d = v^2/a
        # actually d = 2 * (0.5 * a * t^2) where t = v/a.  => d = v^2/a => v = sqrt(d*a)
        # But wait, v_peak is reached at midpoint. d_half = 0.5 * dist_total. 
        # v_peak = sqrt(2 * a * d_half) = sqrt(a * dist_total). Correct.
        
        t_ramp = v_peak / accel
        t_flat = 0
        d_acc_actual = 0.5 * dist_total
        d_flat = 0
        t_total = 2 * t_ramp
    else:
        # Trapezoidal Profile
        v_peak = v_max
        t_ramp = v_max / accel
        d_acc_actual = d_ramp_theoretical
        d_flat = dist_total - 2 * d_acc_actual
        t_flat = d_flat / v_max
        t_total = (2 * t_ramp) + t_flat

    times = np.arange(0, t_total + dt, dt)
    data = []

    for t in times:
        if t <= t_ramp:
            # Accel
            v = accel * t
            s = 0.5 * accel * t**2
        elif t <= (t_ramp + t_flat):
            # Cruise
            v = v_peak
            s = d_acc_actual + v_peak * (t - t_ramp)
        elif t <= t_total:
            # Decel
            t_dec_start = t_ramp + t_flat
            t_local = t - t_dec_start
            v = v_peak - (accel * t_local)
            s_decel_start = d_acc_actual + d_flat
            s = s_decel_start + (v_peak * t_local - 0.5 * accel * t_local**2)
        else:
            v = 0
            s = dist_total

        if s > dist_total: s = dist_total
        if v < 0: v = 0
        
        ratio = s / dist_total if dist_total > 0 else 0
        x = start_pos[0] + ratio * (end_pos[0] - start_pos[0])
        y = start_pos[1] + ratio * (end_pos[1] - start_pos[1])

        data.append({
            "time": round(start_time + t, 2),
            "x": round(x, 4),
            "y": round(y, 4),
            "v": round(v, 4),
            "leg": leg_name
        })
        
    return data, start_time + t_total

# 3. Build Full Mission
full_data = []
curr_time = 0.0

for i in range(len(path_sequence) - 1):
    leg_res, end_t = generate_stop_go_leg(
        waypoints[path_sequence[i]], 
        waypoints[path_sequence[i+1]], 
        curr_time, 
        f"{path_sequence[i]}->{path_sequence[i+1]}"
    )
    if full_data:
        full_data.extend(leg_res[1:]) # stitch
    else:
        full_data.extend(leg_res)
    curr_time = end_t

# Save
df = pd.DataFrame(full_data)
df.to_csv(f'tiago_path-{SCENARIO}.csv', index=False)
print(f"CSV Saved as tiago_path-{SCENARIO}.csv")

# 3. Plotting
plt.figure(figsize=(12, 12))

if img_arr is not None:
    plt.imshow(img_arr, cmap='gray', extent=extent, origin='lower', alpha=0.6)

# Plot Trajectory
plt.plot(df['x'], df['y'], 'b-', linewidth=2, label='Tiago Path')

# Plot Waypoints for context
for pid, coord in waypoints.items():
    plt.scatter(coord[0], coord[1], c='red', s=40, zorder=5)
    plt.annotate(pid, (coord[0], coord[1]), xytext=(5,5), textcoords='offset points', color='darkred', weight='bold')

plt.title("Tiago Trajectory Overlay on Map")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.legend()
plt.grid(False) # Grid usually looks messy on top of a map
# Zoom into the relevant area to make it readable
# Get bounds of trajectory with some padding
x_min_t, x_max_t = df['x'].min(), df['x'].max()
y_min_t, y_max_t = df['y'].min(), df['y'].max()
pad = 5.0
plt.xlim(x_min_t - pad, x_max_t + pad)
plt.ylim(y_min_t - pad, y_max_t + pad)
plt.show()