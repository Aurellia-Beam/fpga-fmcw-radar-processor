#!/usr/bin/env python3
"""
ADR_visualize.py
Visualizer for Air Defense Radar tactical scenario
Displays detections, tracks, and notch maneuver effects
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional

# Radar parameters (match VHDL)
N_RANGE = 1024
N_DOPPLER = 128
MAX_RANGE_KM = 120.0
WAVELENGTH_M = 0.1
PRF_HZ = [8000, 9000, 10000]
SCAN_RATE = 2.0  # Hz
NOTCH_TIME = 30.0  # seconds

@dataclass
class Track:
    id: int
    range_bins: list = field(default_factory=list)
    doppler_bins: list = field(default_factory=list)
    velocities: list = field(default_factory=list)
    qualities: list = field(default_factory=list)
    scans: list = field(default_factory=list)

def load_detections(filepath: str = "ADR_detections.txt"):
    """Load detection data from simulation output."""
    detections = []
    if not Path(filepath).exists():
        return np.array([])
    
    with open(filepath) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 3:
                r, d, mag = int(parts[0]), int(parts[1]), int(parts[2])
                detections.append([r, d, mag])
    
    return np.array(detections) if detections else np.array([])

def load_tracks(filepath: str = "ADR_tracks.txt"):
    """Load track data from simulation output."""
    tracks = {}
    scan_counts = []
    current_scan = 0
    
    if not Path(filepath).exists():
        return {}, []
    
    with open(filepath) as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            
            if parts[0] == "TRK":
                trk_id = int(parts[1])
                if trk_id not in tracks:
                    tracks[trk_id] = Track(id=trk_id)
                
                # Parse: TRK id R=range D=doppler VR=vel Q=quality S=status
                r = int(parts[2].split('=')[1])
                d = int(parts[3].split('=')[1])
                vr = int(parts[4].split('=')[1])
                q = int(parts[5].split('=')[1])
                
                tracks[trk_id].range_bins.append(r / 4.0)  # Q2 to bins
                tracks[trk_id].doppler_bins.append(d / 4.0)
                tracks[trk_id].velocities.append(vr)
                tracks[trk_id].qualities.append(q)
                tracks[trk_id].scans.append(current_scan)
                
            elif parts[0] == "SCAN_END":
                active = int(parts[1].split('=')[1])
                scan_counts.append(active)
                current_scan += 1
    
    return tracks, scan_counts

def bin_to_range_km(bin_idx):
    """Convert range bin to km."""
    return (bin_idx / N_RANGE) * MAX_RANGE_KM

def bin_to_velocity_mps(doppler_bin, prf_idx=0):
    """Convert Doppler bin to velocity (m/s)."""
    # Centered at N_DOPPLER/2
    centered = doppler_bin - N_DOPPLER/2
    prf = PRF_HZ[prf_idx % 3]
    # fd = centered * prf / N_DOPPLER
    # v = fd * wavelength / 2
    fd = centered * prf / N_DOPPLER
    return fd * WAVELENGTH_M / 2.0

def plot_rdm_with_tracks(detections, tracks, scan_idx=None, title=""):
    """Plot Range-Doppler Map with track overlays."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Build RDM from detections
    rdm = np.zeros((N_DOPPLER, N_RANGE))
    if len(detections) > 0:
        # If scan_idx specified, filter to approximate scan
        samples_per_scan = N_RANGE * N_DOPPLER
        if scan_idx is not None:
            start = scan_idx * samples_per_scan
            end = min(start + samples_per_scan, len(detections))
            scan_dets = detections[start:end]
        else:
            scan_dets = detections
        
        for r, d, mag in scan_dets:
            if 0 <= r < N_RANGE and 0 <= d < N_DOPPLER:
                rdm[int(d), int(r)] = max(rdm[int(d), int(r)], mag)
    
    # RDM plot
    ax1 = axes[0]
    rdm_db = 20 * np.log10(rdm + 1)
    
    # Convert axes to physical units
    range_km = np.linspace(0, MAX_RANGE_KM, N_RANGE)
    vel_mps = np.array([bin_to_velocity_mps(d) for d in range(N_DOPPLER)])
    
    im = ax1.imshow(rdm_db, aspect='auto', origin='lower',
                    extent=[0, MAX_RANGE_KM, vel_mps[0], vel_mps[-1]],
                    cmap='viridis')
    ax1.set_xlabel('Range (km)')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title(f'Range-Doppler Map {title}')
    plt.colorbar(im, ax=ax1, label='dB')
    
    # Overlay tracks
    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    for trk_id, trk in tracks.items():
        if len(trk.range_bins) > 0:
            r_km = [bin_to_range_km(r) for r in trk.range_bins]
            v_mps = [bin_to_velocity_mps(d) for d in trk.doppler_bins]
            
            color = colors[trk_id % 10]
            ax1.plot(r_km, v_mps, 'o-', color=color, markersize=4, 
                    linewidth=1, alpha=0.7, label=f'Track {trk_id}')
    
    if tracks:
        ax1.legend(loc='upper right', fontsize=8)
    
    # Zero-Doppler notch region
    ax1.axhspan(-20, 20, alpha=0.2, color='red', label='MTI Notch')
    
    # Track quality over time
    ax2 = axes[1]
    for trk_id, trk in tracks.items():
        if len(trk.scans) > 0:
            t_sec = np.array(trk.scans) / SCAN_RATE
            color = colors[trk_id % 10]
            ax2.plot(t_sec, trk.qualities, 'o-', color=color, 
                    markersize=3, label=f'Track {trk_id}')
    
    ax2.axvline(NOTCH_TIME, color='red', linestyle='--', label='Notch Start')
    ax2.axvline(NOTCH_TIME + 10, color='green', linestyle='--', label='Notch End')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Track Quality')
    ax2.set_title('Track Quality vs Time')
    ax2.set_ylim(0, 16)
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_track_history(tracks):
    """Plot track position history in Range-Time and Doppler-Time."""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    
    # Range vs Time
    ax1 = axes[0]
    for trk_id, trk in tracks.items():
        if len(trk.scans) > 0:
            t_sec = np.array(trk.scans) / SCAN_RATE
            r_km = [bin_to_range_km(r) for r in trk.range_bins]
            color = colors[trk_id % 10]
            ax1.plot(t_sec, r_km, 'o-', color=color, markersize=2, 
                    label=f'Track {trk_id}')
    
    ax1.axvline(NOTCH_TIME, color='red', linestyle='--', alpha=0.5)
    ax1.axvline(NOTCH_TIME + 10, color='green', linestyle='--', alpha=0.5)
    ax1.set_ylabel('Range (km)')
    ax1.set_title('Track Range History')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, alpha=0.3)
    
    # Velocity vs Time
    ax2 = axes[1]
    for trk_id, trk in tracks.items():
        if len(trk.scans) > 0:
            t_sec = np.array(trk.scans) / SCAN_RATE
            v_mps = [bin_to_velocity_mps(d) for d in trk.doppler_bins]
            color = colors[trk_id % 10]
            ax2.plot(t_sec, v_mps, 'o-', color=color, markersize=2,
                    label=f'Track {trk_id}')
    
    ax2.axvline(NOTCH_TIME, color='red', linestyle='--', alpha=0.5, label='Notch Start')
    ax2.axvline(NOTCH_TIME + 10, color='green', linestyle='--', alpha=0.5, label='Notch End')
    ax2.axhspan(-20, 20, alpha=0.2, color='red', label='MTI Notch')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Track Velocity History (Notch Maneuver Visible)')
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_active_tracks(scan_counts):
    """Plot number of active tracks over time."""
    fig, ax = plt.subplots(figsize=(10, 4))
    
    t_sec = np.arange(len(scan_counts)) / SCAN_RATE
    ax.plot(t_sec, scan_counts, 'b-', linewidth=2)
    ax.axhline(10, color='gray', linestyle='--', alpha=0.5, label='Expected (6+4)')
    ax.axvline(NOTCH_TIME, color='red', linestyle='--', label='Notch Start')
    ax.axvline(NOTCH_TIME + 10, color='green', linestyle='--', label='Notch End')
    
    ax.fill_between(t_sec, scan_counts, alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Active Tracks')
    ax.set_title('Track Count Over Time')
    ax.set_ylim(0, max(scan_counts) + 2 if scan_counts else 15)
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def analyze_notch_performance(tracks):
    """Analyze track maintenance during notch maneuver."""
    print("\n=== NOTCH MANEUVER ANALYSIS ===\n")
    
    notch_start_scan = int(NOTCH_TIME * SCAN_RATE)
    notch_end_scan = int((NOTCH_TIME + 10) * SCAN_RATE)
    
    for trk_id, trk in tracks.items():
        if len(trk.scans) < 5:
            continue
        
        scans = np.array(trk.scans)
        quals = np.array(trk.qualities)
        vels = np.array([bin_to_velocity_mps(d) for d in trk.doppler_bins])
        
        # Check if track existed before notch
        pre_notch = scans < notch_start_scan
        during_notch = (scans >= notch_start_scan) & (scans <= notch_end_scan)
        post_notch = scans > notch_end_scan
        
        if not any(pre_notch):
            continue
        
        print(f"Track {trk_id}:")
        print(f"  Pre-notch quality:  {np.mean(quals[pre_notch]):.1f}")
        
        if any(during_notch):
            print(f"  During notch quality: {np.mean(quals[during_notch]):.1f}")
            print(f"  During notch velocity: {np.mean(vels[during_notch]):.1f} m/s")
            
            # Did track enter notch region?
            in_notch = np.abs(vels[during_notch]) < 20
            if any(in_notch):
                print(f"  ⚠️  Track entered MTI notch region")
        else:
            print(f"  ❌ Track LOST during notch")
        
        if any(post_notch):
            print(f"  Post-notch quality: {np.mean(quals[post_notch]):.1f}")
        else:
            print(f"  ❌ Track NOT RECOVERED after notch")
        
        print()

def main():
    print("Loading tactical scenario data...")
    
    detections = load_detections("ADR_detections.txt")
    tracks, scan_counts = load_tracks("ADR_tracks.txt")
    
    print(f"Loaded {len(detections)} detections")
    print(f"Loaded {len(tracks)} tracks over {len(scan_counts)} scans")
    
    if len(detections) == 0 and len(tracks) == 0:
        print("\nNo data found. Run ADR_tb_tactical simulation first.")
        print("Expected files: ADR_detections.txt, ADR_tracks.txt")
        return
    
    # Analysis
    analyze_notch_performance(tracks)
    
    # Plots
    fig1 = plot_rdm_with_tracks(detections, tracks, title="(All Scans Combined)")
    fig1.savefig('ADR_rdm_tracks.png', dpi=150, bbox_inches='tight')
    print("Saved: ADR_rdm_tracks.png")
    
    fig2 = plot_track_history(tracks)
    fig2.savefig('ADR_track_history.png', dpi=150, bbox_inches='tight')
    print("Saved: ADR_track_history.png")
    
    if scan_counts:
        fig3 = plot_active_tracks(scan_counts)
        fig3.savefig('ADR_track_count.png', dpi=150, bbox_inches='tight')
        print("Saved: ADR_track_count.png")
    
    plt.show()

if __name__ == "__main__":
    main()
