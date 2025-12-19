import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math

def plot_boxplot(df, title, ylabel, fontsize=15, outdir = None):
    ylimit = math.ceil(max(np.concatenate([np.array(df[SCENARIO][k]["data"]) for SCENARIO in SCENARIOS for k in df[SCENARIO].keys()]))) + 1
    plt.figure(figsize=(10, 6))
    ax = plt.gca()

    plt.boxplot([df[SCENARIO][k]["data"] for SCENARIO in SCENARIOS for k in df[SCENARIO].keys()], labels=[df[SCENARIO][k]["label"] for SCENARIO in SCENARIOS for k in df[SCENARIO].keys()])
    plt.title(f"{title}", fontdict={"fontsize": fontsize})
    plt.ylabel(ylabel, fontdict={"fontsize": fontsize})
    plt.grid(axis='y', linestyle='--', alpha=0.6)
    plt.xticks(fontsize=fontsize)
    plt.yticks(fontsize=fontsize)
    plt.yticks(range(0, ylimit, 1), fontsize=fontsize)
    
    current_right_lim = len(SCENARIOS)*2 + 0.5
    ax.set_xlim(left=-0.5, right=current_right_lim)

    # Define bands and labels
    bands = [
        (0, 0.5, "Intimate", "tab:red"),
        (0.5, 1.2, "Personal", "tab:orange"),
        (1.2, 3.6, "Social", "tab:blue"),
        (3.6, 7.6, "Public", "tab:green"),
    ]

    # Add colored bands and labels
    for lower, upper, label, color in bands:
        ax.axhspan(lower, upper, color=color, alpha=0.3)
        # Place text at a fixed position on the far left (e.g., x = -0.4)
        ax.text(-0.4, (lower + upper) / 2, label, 
                va="center", ha="left", fontsize=fontsize-1, color="black") 
        
    plt.tight_layout()
    if outdir is not None:
        plt.savefig(os.path.join(outdir, f'{title.lower().replace(" ", "_")}_boxplot.png'))
    else:
        plt.show()
    plt.close()




# SCENARIO = 
SCENARIOS = ["poster", "buffet"]
df = {"poster": {"baseline": {"data": [], "label": ""}, "causal": {"data": [], "label": ""}},
      "buffet": {"baseline": {"data": [], "label": ""}, "causal": {"data": [], "label": ""}}}
df = {SCENARIO: {"baseline": {"data": [], "label": ""}, "causal": {"data": [], "label": ""}} for SCENARIO in SCENARIOS}

for SCENARIO in SCENARIOS:
    CSVPATH = "/home/lcastri/git/PeopleFlow/utilities_ws/src/RAL/robot_postprocess/csv/"
    df_baseline = pd.read_csv(CSVPATH + f'tiago_path-{SCENARIO}-baseline.csv')
    df_causal = pd.read_csv(CSVPATH + f'tiago_path-{SCENARIO}-causal.csv')

    centroids = {
        "poster2": (16.73, 2.17),
        "O": (26.5, -0.75),
    }
    if SCENARIO == "poster":
        centroid_x, centroid_y = centroids["poster2"]
    elif SCENARIO == "buffet":
        centroid_x, centroid_y = centroids["O"]

    # 3. Calculate Euclidean Distance for every timestep
    df[SCENARIO]['baseline']["data"] = np.sqrt((df_baseline['x'] - centroid_x)**2 + (df_baseline['y'] - centroid_y)**2)
    df[SCENARIO]['causal']["data"] = np.sqrt((df_causal['x'] - centroid_x)**2 + (df_causal['y'] - centroid_y)**2)
    df[SCENARIO]['baseline']["label"] = f"baseline-{SCENARIO}"
    df[SCENARIO]['causal']["label"] = f"causal-{SCENARIO}"

plot_boxplot(df, 
             title="Human-Robot Proxemic Compliance",
             ylabel="m",
             outdir=None
            )