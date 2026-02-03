import matplotlib.pyplot as plt
import numpy as np

# Configuration
N_RANGE = 1024
N_DOPPLER = 128
FILENAME = "radar_heatmap_data.txt"

def parse_radar_data():
    print(f"Reading {FILENAME}...")
    
    # We will accumulate frames. 
    # Since the simulation runs 4 frames, we can sum them or just show the last one.
    # Let's show the 'Max Hold' map (accumulated detections).
    rd_map = np.zeros((N_RANGE, N_DOPPLER))
    
    try:
        with open(FILENAME, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) < 3: continue
                
                r_idx = int(parts[0])
                d_idx = int(parts[1])
                mag   = int(parts[2])
                
                # Update map
                if r_idx < N_RANGE and d_idx < N_DOPPLER:
                    # Because the file contains multiple frames, we just overwrite 
                    # or max-hold. Let's use max-hold to see all targets from all frames.
                    rd_map[r_idx, d_idx] = max(rd_map[r_idx, d_idx], mag)
                    
    except FileNotFoundError:
        print(f"Error: Could not find {FILENAME}. Run the Vivado simulation first!")
        return None

    return rd_map

def plot_map(rd_map):
    plt.figure(figsize=(10, 8))
    
    # Use 'hot' colormap for radar look. Origin='lower' flips it to normal graph coordinates
    plt.imshow(rd_map, aspect='auto', cmap='inferno', origin='lower')
    
    plt.title(f"OS-CFAR Range-Doppler Map (Max Hold)\nSize: {N_RANGE}x{N_DOPPLER}")
    plt.xlabel("Doppler Bin")
    plt.ylabel("Range Bin")
    plt.colorbar(label="Detection Magnitude (CFAR Output)")
    
    print("Displaying plot...")
    plt.show()

if __name__ == "__main__":
    data = parse_radar_data()
    if data is not None:
        plot_map(data)