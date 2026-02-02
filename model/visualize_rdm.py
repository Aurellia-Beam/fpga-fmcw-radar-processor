#!/usr/bin/env python3
"""Visualize FMCW Radar Range-Doppler Map from simulation output."""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Parameters from testbench
N_RANGE = 1024
N_DOPPLER = 128
TARGET_1 = (100, 5)    # (range_bin, doppler_bin)
TARGET_2 = (500, -10)  # Note: -10 wraps to 118 in unsigned (128-10)

def load_rdm(filepath):
    """Load radar output: range doppler magnitude"""
    data = np.loadtxt(filepath, dtype=int)
    rdm = np.zeros((N_RANGE, N_DOPPLER))
    for row in data:
        r, d, mag = row[0], row[1], row[2]
        if 0 <= r < N_RANGE and 0 <= d < N_DOPPLER:
            rdm[r, d] = mag
    return rdm

def plot_rdm(rdm):
    """Plot Range-Doppler Map with expected target markers."""
    plt.figure(figsize=(12, 8))
    
    # Convert to dB (avoid log(0))
    rdm_db = 20 * np.log10(rdm + 1)
    
    # Shift Doppler axis so 0 is centered
    rdm_shifted = np.fft.fftshift(rdm_db, axes=1)
    doppler_axis = np.arange(N_DOPPLER) - N_DOPPLER // 2
    
    plt.imshow(rdm_shifted.T, aspect='auto', origin='lower',
               extent=[0, N_RANGE, -N_DOPPLER//2, N_DOPPLER//2],
               cmap='viridis')
    plt.colorbar(label='Magnitude (dB)')
    
    
    plt.xlabel('Range Bin')
    plt.ylabel('Doppler Bin')
    plt.title('FMCW Radar Range-Doppler Map')
    plt.tight_layout()
    plt.savefig('rdm_plot.png', dpi=150)
    plt.show()
    print("Saved: rdm_plot.png")

def analyze_peaks(rdm, n_peaks=5):
    """Find and report top magnitude peaks."""
    flat_idx = np.argsort(rdm.flatten())[-n_peaks:][::-1]
    print(f"\nTop {n_peaks} detections:")
    print(f"{'Range':>8} {'Doppler':>8} {'Magnitude':>12}")
    print("-" * 30)
    for idx in flat_idx:
        r, d = np.unravel_index(idx, rdm.shape)
        # Convert Doppler to signed
        d_signed = d if d < N_DOPPLER // 2 else d - N_DOPPLER
        print(f"{r:>8} {d_signed:>8} {rdm[r, d]:>12.0f}")

if __name__ == "__main__":
    # Try common locations for output file
    paths = ["radar_output.txt", "C:/radar_output.txt", "./radar_output.txt"]
    
    rdm = None
    for p in paths:
        if Path(p).exists():
            print(f"Loading: {p}")
            rdm = load_rdm(p)
            break
    
    if rdm is None:
        print("ERROR: radar_output.txt not found. Run simulation first.")
        print("Expected format: range_idx doppler_idx magnitude (space-separated)")
        exit(1)
    
    print(f"Loaded RDM: {rdm.shape}, max={rdm.max():.0f}, nonzero={np.count_nonzero(rdm)}")
    analyze_peaks(rdm)
    plot_rdm(rdm)
