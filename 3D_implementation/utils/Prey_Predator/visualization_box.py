import pandas as pd
import matplotlib.pyplot as plt

# Parameters
RADIUS = 2.5   # fixed for now
THRESH_ENGULF = (2 * RADIUS) * 0.9   # prey+pred radius -10%
THRESH_SEP = 3 * RADIUS

# Load data
df = pd.read_csv("experiment_results/results_baseline_03-10-2025_14-16/distance_data_baseline.txt", sep="\t")

# Classify behavior
def classify_behavior(d):
    if d <= THRESH_ENGULF:
        return "engulfing"
    elif d >= THRESH_SEP:
        return "separating"
    else:
        return "following"

df["behavior"] = df["distance"].apply(classify_behavior)

# Count per behavior
counts = df["behavior"].value_counts()

# --- Bar Plot ---
plt.figure(figsize=(6,4))
counts.plot(kind="bar", color=["red", "blue", "green"])
plt.ylabel("Count")
plt.title("Behavior Classification (all trials)")
plt.xticks(rotation=0)
plt.tight_layout()
plt.show()

# --- Box Plot (distance distributions per behavior) ---
plt.figure(figsize=(6,4))
df.boxplot(column="distance", by="behavior", grid=False)
plt.ylabel("Distance")
plt.title("Distance Distributions by Behavior")
plt.suptitle("")
plt.tight_layout()
plt.show()
