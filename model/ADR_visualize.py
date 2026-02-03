#!/usr/bin/env python3
"""
ADR_visualize.py
Visualizer for Air Defense Radar tactical scenario
Displays detections, tracks, and notch maneuver effects
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional
import glob

# Radar parameters (match VHDL)
N_RANGE = 1024
N_DOPPLER = 128
MAX_RANGE_KM = 120.0
WAVELENGTH_M = 0.1
PRF_HZ = [8000, 9000, 10000]
SCAN_RATE = 2.0  # Hz
NOTCH_TIME = 30.0  # seconds

# Quick test parameters
N_RANGE_QUICK = 128
N_DOPPLER_QUICK = 32

@dataclass
class Track:
    id: int
    range_bins: list = field(default_factory=list)
    doppler_bins: list = field(default_factory=list)
    velocities: list = field(default_factory=list)
    qualities: list = field(default_factory=list)
    scans: list = field(default_factory=list)

def find_sim_files(det_name="ADR_quick_det.txt", trk_name="ADR_quick_trk.txt"):
    """Auto-find simulation output files in common Vivado locations."""
    search_paths = [
        ".",  # Current directory
        "..",
        # Vivado sim directories
        "**/sim_1/behav/xsim",
        "**/*.sim/sim_1/behav/xsim",
        "../**/*.sim/sim_1/behav/xsim",
        "../../**/*.sim/sim_1/behav/xsim",
        # Common project structures
        "**/xsim",
        "../**/xsim",
    ]
    
    det_file = None
    trk_file = None
    
    # Try direct paths first
    for pattern in search_paths:
        if '*' in pattern:
            matches = glob.glob(f"{pattern}/{det_name}", recursive=True)
            if matches:
                det_file = matches[0]
                break
        else:
            path = Path(pattern) / det_name
            if path.exists():
                det_file = str(path)
                break
    
    for pattern in search_paths:
        if '*' in pattern:
            matches = glob.glob(f"{pattern}/{trk_name}", recursive=True)
            if matches:
                trk_file = matches[0]
                break
        else:
            path = Path(pattern) / trk_name
            if path.exists():
                trk_file = str(path)
                break
    
    # Also check for full tactical sim files
    if det_file is None:
        for pattern in search_paths:
            if '*' in pattern:
                matches = glob.glob(f"{pattern}/ADR_detections.txt", recursive=True)
                if matches:
                    det_file = matches[0]
                    break
            else:
                path = Path(pattern) / "ADR_detections.txt"
                if path.exists():
                    det_file = str(path)
                    break
    
    if trk_file is None:
        for pattern in search_paths:
            if '*' in pattern:
                matches = glob.glob(f"{pattern}/ADR_tracks.txt", recursive=True)
                if matches:
                    trk_file = matches[0]
                    break
            else:
                path = Path(pattern) / "ADR_tracks.txt"
                if path.exists():
                    trk_file = str(path)
                    break
                    
    return det_file, trk_file

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
    
    if not filepath or not Path(filepath).exists():
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
                
                # Parse: TRK id R=range D=doppler Q=quality
                # or:    TRK id R=range D=doppler VR=vel Q=quality S=status
                r = int(parts[2].split('=')[1])
                d = int(parts[3].split('=')[1])
                
                # Handle both quick format (5 parts) and full format (7 parts)
                if len(parts) >= 5:
                    # Find Q= field
                    q = 0
                    for p in parts[4:]:
                        if p.startswith('Q='):
                            q = int(p.split('=')[1])
                            break
                
                tracks[trk_id].range_bins.append(r)
                tracks[trk_id].doppler_bins.append(d)
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
    print("Searching for simulation output files...")
    
    det_file, trk_file = find_sim_files()
    
    if det_file:
        print(f"Found detections: {det_file}")
    else:
        print("No detection file found")
        
    if trk_file:
        print(f"Found tracks: {trk_file}")
    else:
        print("No track file found")
    
    if not det_file and not trk_file:
        print("\nNo data found. Run simulation first.")
        print("Searched for: ADR_quick_det.txt, ADR_quick_trk.txt")
        print("             ADR_detections.txt, ADR_tracks.txt")
        print("\nOr specify path manually:")
        print("  python ADR_visualize.py /path/to/sim/folder")
        return
    
    # Detect if quick or full sim based on filename
    is_quick = "quick" in (det_file or "") or "quick" in (trk_file or "")
    n_range = N_RANGE_QUICK if is_quick else N_RANGE
    n_doppler = N_DOPPLER_QUICK if is_quick else N_DOPPLER
    
    print(f"Mode: {'Quick' if is_quick else 'Full'} ({n_range}x{n_doppler})")
    
    detections = load_detections(det_file) if det_file else np.array([])
    tracks, scan_counts = load_tracks(trk_file) if trk_file else ({}, [])
    
    print(f"Loaded {len(detections)} detections")
    print(f"Loaded {len(tracks)} tracks over {len(scan_counts)} scans")
    
    if len(tracks) == 0 and len(detections) == 0:
        print("Files found but empty.")
        return
    
    # Quick track plot
    if tracks:
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        
        # Conversion factors
        km_per_bin = MAX_RANGE_KM / n_range
        nm_per_km = 0.539957
        nm_per_bin = km_per_bin * nm_per_km
        
        # Doppler to velocity: fd = 2*v/lambda, bin = fd/PRF * N_DOPPLER + N/2
        # v = (bin - N/2) * PRF * lambda / (2 * N_DOPPLER)
        prf = PRF_HZ[0]  # Use first PRF
        mps_per_bin = prf * WAVELENGTH_M / (2 * n_doppler)
        kts_per_mps = 1.94384
        kts_per_bin = mps_per_bin * kts_per_mps
        zero_doppler_q2 = n_doppler / 2 * 4  # Q2 format
        
        # Range vs scan
        ax1 = axes[0]
        colors = plt.cm.tab10(np.linspace(0, 1, 10))
        for trk_id, trk in tracks.items():
            if len(trk.scans) > 0:
                color = colors[trk_id % 10]
                range_nm = [r / 4 * nm_per_bin for r in trk.range_bins]  # Q2 to bins to nm
                ax1.plot(trk.scans, range_nm, 'o-', color=color, 
                        markersize=4, label=f'Track {trk_id}')
        ax1.set_xlabel('Scan')
        ax1.set_ylabel('Range (nm)')
        ax1.set_title('Track Range vs Scan')
        ax1.legend(loc='best', fontsize=8)
        ax1.grid(True, alpha=0.3)
        
        # Doppler vs scan (in knots)
        ax2 = axes[1]
        for trk_id, trk in tracks.items():
            if len(trk.scans) > 0:
                color = colors[trk_id % 10]
                # Q2 doppler bins to velocity: (d/4 - N/2) * kts_per_bin
                vel_kts = [(d / 4 - n_doppler / 2) * kts_per_bin for d in trk.doppler_bins]
                ax2.plot(trk.scans, vel_kts, 'o-', color=color,
                        markersize=4, label=f'Track {trk_id}')
        ax2.axhline(0, color='red', linestyle='--', alpha=0.5, label='Zero Doppler (Notch)')
        ax2.set_xlabel('Scan')
        ax2.set_ylabel('Velocity (kts)')
        ax2.set_title('Track Velocity vs Scan')
        ax2.legend(loc='best', fontsize=8)
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('ADR_tracks_result.png', dpi=150)
        print("Saved: ADR_tracks_result.png")
        
    # Detection heatmap
    if len(detections) > 0:
        fig2, ax = plt.subplots(figsize=(10, 6))
        rdm = np.zeros((n_doppler, n_range))
        for r, d, mag in detections:
            if 0 <= int(r) < n_range and 0 <= int(d) < n_doppler:
                rdm[int(d), int(r)] = max(rdm[int(d), int(r)], mag)
        
        rdm_db = 20 * np.log10(rdm + 1)
        
        # Axis labels in nm and kts
        km_per_bin = MAX_RANGE_KM / n_range
        nm_per_km = 0.539957
        range_nm = np.arange(n_range) * km_per_bin * nm_per_km
        
        prf = PRF_HZ[0]
        mps_per_bin = prf * WAVELENGTH_M / (2 * n_doppler)
        kts_per_mps = 1.94384
        vel_kts = (np.arange(n_doppler) - n_doppler/2) * mps_per_bin * kts_per_mps
        
        ax.imshow(rdm_db, aspect='auto', origin='lower', cmap='viridis',
                  extent=[range_nm[0], range_nm[-1], vel_kts[0], vel_kts[-1]])
        ax.set_xlabel('Range (nm)')
        ax.set_ylabel('Velocity (kts)')
        ax.set_title('Detection Heatmap (All Scans)')
        plt.colorbar(ax.images[0], ax=ax, label='dB')
        plt.tight_layout()
        plt.savefig('ADR_detections_result.png', dpi=150)
        print("Saved: ADR_detections_result.png")
    
    plt.show()
    
    # Print summary
    print("\n=== TRACK SUMMARY ===")
    km_per_bin = MAX_RANGE_KM / n_range
    nm_per_km = 0.539957
    nm_per_bin = km_per_bin * nm_per_km
    for trk_id, trk in tracks.items():
        r_start = trk.range_bins[0] / 4 * nm_per_bin
        r_end = trk.range_bins[-1] / 4 * nm_per_bin
        print(f"Track {trk_id}: {len(trk.scans)} updates, "
              f"R={r_start:.1f}->{r_end:.1f} nm, "
              f"Q={trk.qualities[-1] if trk.qualities else 0}")

if __name__ == "__main__":
    import sys
    
    # Allow manual path override
    if len(sys.argv) > 1:
        folder = Path(sys.argv[1])
        print(f"Searching in: {folder}")
        
        det_file = None
        trk_file = None
        
        for name in ["ADR_quick_det.txt", "ADR_detections.txt"]:
            if (folder / name).exists():
                det_file = str(folder / name)
                break
        for name in ["ADR_quick_trk.txt", "ADR_tracks.txt"]:
            if (folder / name).exists():
                trk_file = str(folder / name)
                break
        
        # Override the find function
        _original_find = find_sim_files
        find_sim_files = lambda *a, **k: (det_file, trk_file)
    
    main()
