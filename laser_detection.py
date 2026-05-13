import numpy as np
import matplotlib.pyplot as plt

WINDOW = 5
MIN_ON_CLUSTER = 3
MIN_SCORE = 0

def simulate_laser_row(row_length=150, laser_center=75):
    """
    Simulates a 1D array representing accumulated event polarities on a single Y-row.
    A moving laser typically generates ON events (+1) on its leading edge
    and OFF events (-1) on its trailing edge.
    """
    pol = np.zeros(row_length, dtype=int)

    # Simulate the laser strike at the center
    # Left side gets positive events, right side gets negative events
    # This satisfies your C++ logic where L_m needs to be positive
    pol[laser_center-5:laser_center] = [1, 2, 3, 2, 1]
    pol[laser_center+1:laser_center+6] = [-1, -2, -3, -2, -1]

    # Inject some random background noise (sensor chatter)
    noise_indices = np.random.choice(row_length, 40, replace=False)
    pol[noise_indices] += np.random.choice([-1, 1], 40)

    return pol

def run_detection_algorithm(pol, window=WINDOW, min_on_cluster=MIN_ON_CLUSTER, min_score=MIN_SCORE):
    """
    An exact Python port of your C++ Polarity Spatial Gradient logic.
    """
    row_length = len(pol)
    scores = np.zeros(row_length)

    best_x = -1
    max_score = 0

    for x in range(window, row_length - window):
        L = 0
        R = 0

        # Identical to: for (int i = 1; i <= WINDOW; ++i)
        for i in range(1, window + 1):
            L += pol[x - i]
            R += pol[x + i]

        score = L - R
        scores[x] = score

        # Your physical gate logic
        if L >= min_on_cluster and score > max_score:
            max_score = score
            best_x = x

    # Apply MIN_SCORE threshold at the very end, just like the C++ code
    if max_score < min_score:
        best_x = -1

    return scores, best_x, max_score

# ==========================================
# Run the Simulation
# ==========================================
row_length = 150


# 1. Generate the raw polarity data
pol_data = simulate_laser_row(row_length=row_length, laser_center=75)

# 2. Run your algorithm
scores, best_x, max_score = run_detection_algorithm(
    pol_data, window=WINDOW, min_on_cluster=MIN_ON_CLUSTER, min_score=MIN_SCORE
)

# ==========================================
# Plot the Results
# ==========================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# Top Plot: Raw Polarity Accumulation
x_axis = np.arange(row_length)
ax1.bar(x_axis, pol_data, color=['green' if val > 0 else 'red' if val < 0 else 'gray' for val in pol_data])
ax1.set_title("Raw Polarity Accumulation (1 Row)", fontsize=14)
ax1.set_ylabel("Accumulated Polarity")
ax1.axhline(0, color='black', linewidth=1)
ax1.grid(True, linestyle='--', alpha=0.6)

# Bottom Plot: The Calculated Score (L_m - R_m)
ax2.plot(x_axis, scores, color='blue', linewidth=2, label="Spatial Gradient Score (L - R)")
ax2.axhline(MIN_SCORE, color='orange', linestyle='--', linewidth=2, label=f"Detection Threshold ({MIN_SCORE})")

# Highlight the detected point
if best_x != -1:
    ax2.plot(best_x, max_score, 'ko', markersize=10, label=f"Detected Laser Center (x={best_x})")
    ax2.vlines(x=best_x, ymin=0, ymax=max_score, color='black', linestyle=':')
    ax1.vlines(x=best_x, ymin=min(pol_data), ymax=max(pol_data), color='black', linestyle=':', label="Detected Center")
    print(f"SUCCESS: Laser detected at pixel {best_x} with a score of {max_score}.")
else:
    print("FAILED: No laser detected above threshold.")

ax2.set_title("Algorithm Output: Gradient Score", fontsize=14)
ax2.set_xlabel("Pixel Column (X coordinate)", fontsize=12)
ax2.set_ylabel("Score")
ax2.legend()
ax2.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()
plt.show()