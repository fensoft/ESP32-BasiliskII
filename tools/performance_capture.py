#!/usr/bin/env python3
"""
Performance Capture Tool for BasiliskII ESP32.

Captures serial output for a specified duration, extracts performance metrics,
and logs them to the performance tracking document.

Usage:
    python3 tools/performance_capture.py                    # 60-second capture
    python3 tools/performance_capture.py --duration 30      # 30-second capture
    python3 tools/performance_capture.py --label "baseline" # Add label to results
    python3 tools/performance_capture.py --port /dev/ttyUSB0  # Custom port
"""

import sys
import os
import re
import argparse
import serial
import time
from datetime import datetime
from statistics import mean, stdev

# Configuration
DEFAULT_SERIAL_PORT = '/dev/cu.usbmodem11401'
BACKUP_SERIAL_PORTS = [
    '/dev/cu.usbmodem211401',
    '/dev/cu.usbmodem11301',
    '/dev/ttyUSB0',
]
BAUD_RATE = 115200
DEFAULT_DURATION = 60  # seconds


def get_project_root():
    """Get the project root directory."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(script_dir)


def find_serial_port(preferred_port):
    """Find an available serial port."""
    ports_to_try = [preferred_port] + BACKUP_SERIAL_PORTS
    
    for port in ports_to_try:
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
            ser.close()
            return port
        except (serial.SerialException, OSError):
            continue
    
    return None


def parse_ips_line(line):
    """Parse IPS line and return MIPS value."""
    # Example: [IPS] 2074192 instructions/sec (2.07 MIPS), total: 132120000
    match = re.search(r'\((\d+\.?\d*)\s*MIPS\)', line)
    if match:
        return float(match.group(1))
    return None


def parse_profiler_block(lines):
    """Parse CPU profiler block and return metrics dict."""
    metrics = {}
    
    for line in lines:
        # Cycles/instr: fetch=14 (11%) exec=109 (89%) total=123
        cycles_match = re.search(
            r'Cycles/instr:\s*fetch=(\d+)\s*\((\d+)%\)\s*exec=(\d+)\s*\((\d+)%\)\s*total=(\d+)',
            line
        )
        if cycles_match:
            metrics['fetch_cycles'] = int(cycles_match.group(1))
            metrics['fetch_pct'] = int(cycles_match.group(2))
            metrics['exec_cycles'] = int(cycles_match.group(3))
            metrics['exec_pct'] = int(cycles_match.group(4))
            metrics['total_cycles'] = int(cycles_match.group(5))
        
        # PSRAM bandwidth: read=3204KB/s write=2122KB/s total=5327KB/s
        bw_match = re.search(
            r'PSRAM bandwidth:\s*read=(\d+)KB/s\s*write=(\d+)KB/s\s*total=(\d+)KB/s',
            line
        )
        if bw_match:
            metrics['psram_read_kbps'] = int(bw_match.group(1))
            metrics['psram_write_kbps'] = int(bw_match.group(2))
            metrics['psram_total_kbps'] = int(bw_match.group(3))
        
        # BOTTLENECK detection
        if 'BOTTLENECK: CPU' in line:
            metrics['bottleneck'] = 'CPU'
        elif 'BOTTLENECK: PSRAM' in line:
            metrics['bottleneck'] = 'PSRAM'
    
    return metrics


def capture_serial(port, duration):
    """Capture serial output for specified duration."""
    print(f"Connecting to {port} at {BAUD_RATE} baud...")
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Error: Could not open serial port: {e}")
        return None, []
    
    print(f"Capturing for {duration} seconds...")
    print("-" * 60)
    
    lines = []
    start_time = time.time()
    
    try:
        while (time.time() - start_time) < duration:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        print(line)
                        lines.append(line)
                except Exception as e:
                    print(f"[Read error: {e}]")
            else:
                time.sleep(0.01)
        
        elapsed = time.time() - start_time
        print("-" * 60)
        print(f"Capture complete. {len(lines)} lines in {elapsed:.1f}s")
        
    except KeyboardInterrupt:
        print("\n[Capture stopped by user]")
        elapsed = time.time() - start_time
    finally:
        ser.close()
    
    return elapsed, lines


def extract_metrics(lines):
    """Extract all performance metrics from captured lines."""
    mips_values = []
    profiler_blocks = []
    
    current_block = []
    in_profiler_block = False
    
    for line in lines:
        # Extract MIPS
        mips = parse_ips_line(line)
        if mips is not None:
            mips_values.append(mips)
        
        # Detect profiler block
        if '========== CPU PROFILER ==========' in line:
            in_profiler_block = True
            current_block = []
        elif '==================================' in line and in_profiler_block:
            in_profiler_block = False
            metrics = parse_profiler_block(current_block)
            if metrics:
                profiler_blocks.append(metrics)
        elif in_profiler_block:
            current_block.append(line)
    
    # Check for WiFi connectivity
    wifi_connected = any('WiFi connected' in line for line in lines)
    wifi_error = any('assert failed' in line.lower() for line in lines)
    
    return {
        'mips_values': mips_values,
        'profiler_blocks': profiler_blocks,
        'wifi_connected': wifi_connected,
        'wifi_error': wifi_error,
    }


def calculate_summary(metrics):
    """Calculate summary statistics from extracted metrics."""
    summary = {
        'wifi_status': 'ERROR' if metrics['wifi_error'] else ('Connected' if metrics['wifi_connected'] else 'Unknown'),
    }
    
    # MIPS statistics
    if metrics['mips_values']:
        summary['mips_avg'] = mean(metrics['mips_values'])
        summary['mips_min'] = min(metrics['mips_values'])
        summary['mips_max'] = max(metrics['mips_values'])
        if len(metrics['mips_values']) > 1:
            summary['mips_stdev'] = stdev(metrics['mips_values'])
        summary['mips_count'] = len(metrics['mips_values'])
    
    # Profiler statistics
    if metrics['profiler_blocks']:
        blocks = metrics['profiler_blocks']
        
        if all('total_cycles' in b for b in blocks):
            cycles = [b['total_cycles'] for b in blocks]
            summary['cycles_avg'] = mean(cycles)
        
        if all('exec_pct' in b for b in blocks):
            exec_pcts = [b['exec_pct'] for b in blocks]
            summary['exec_pct_avg'] = mean(exec_pcts)
        
        if all('psram_total_kbps' in b for b in blocks):
            bw = [b['psram_total_kbps'] for b in blocks]
            summary['psram_bw_avg'] = mean(bw)
        
        # Bottleneck (majority vote)
        bottlenecks = [b.get('bottleneck', 'Unknown') for b in blocks]
        summary['bottleneck'] = max(set(bottlenecks), key=bottlenecks.count)
        summary['profiler_count'] = len(blocks)
    
    return summary


def save_raw_capture(lines, label, timestamp):
    """Save raw capture to timestamped file."""
    project_root = get_project_root()
    captures_dir = os.path.join(project_root, 'docs', 'performance_captures')
    os.makedirs(captures_dir, exist_ok=True)
    
    filename = f"{timestamp}_{label.replace(' ', '_')}.txt"
    filepath = os.path.join(captures_dir, filename)
    
    with open(filepath, 'w') as f:
        f.write('\n'.join(lines))
    
    print(f"Raw capture saved to: {filepath}")
    return filepath


def append_to_tracking_doc(summary, label, timestamp, duration):
    """Append results to PERFORMANCE_TRACKING.md."""
    project_root = get_project_root()
    doc_path = os.path.join(project_root, 'docs', 'PERFORMANCE_TRACKING.md')
    
    # Create doc if it doesn't exist
    if not os.path.exists(doc_path):
        header = """# BasiliskII ESP32-P4 Performance Tracking

This document tracks performance measurements over time.

## Measurements

| Date | Label | MIPS (avg) | MIPS (range) | Cycles/instr | Exec % | PSRAM BW | Bottleneck | WiFi |
|------|-------|------------|--------------|--------------|--------|----------|------------|------|
"""
        with open(doc_path, 'w') as f:
            f.write(header)
    
    # Format the new row
    mips_avg = f"{summary.get('mips_avg', 0):.2f}" if 'mips_avg' in summary else "N/A"
    mips_range = f"{summary.get('mips_min', 0):.2f}-{summary.get('mips_max', 0):.2f}" if 'mips_min' in summary else "N/A"
    cycles = f"{summary.get('cycles_avg', 0):.0f}" if 'cycles_avg' in summary else "N/A"
    exec_pct = f"{summary.get('exec_pct_avg', 0):.0f}%" if 'exec_pct_avg' in summary else "N/A"
    psram_bw = f"{summary.get('psram_bw_avg', 0):.0f} KB/s" if 'psram_bw_avg' in summary else "N/A"
    bottleneck = summary.get('bottleneck', 'N/A')
    wifi = summary.get('wifi_status', 'Unknown')
    
    date_str = datetime.now().strftime('%Y-%m-%d %H:%M')
    
    row = f"| {date_str} | {label} | {mips_avg} | {mips_range} | {cycles} | {exec_pct} | {psram_bw} | {bottleneck} | {wifi} |\n"
    
    # Append to file
    with open(doc_path, 'a') as f:
        f.write(row)
    
    print(f"Results appended to: {doc_path}")


def print_summary(summary, duration):
    """Print a formatted summary of the capture."""
    print("\n" + "=" * 60)
    print("PERFORMANCE CAPTURE SUMMARY")
    print("=" * 60)
    
    print(f"\nWiFi Status: {summary.get('wifi_status', 'Unknown')}")
    
    if 'mips_avg' in summary:
        print(f"\nMIPS Performance ({summary.get('mips_count', 0)} samples):")
        print(f"  Average: {summary['mips_avg']:.2f} MIPS")
        print(f"  Range:   {summary.get('mips_min', 0):.2f} - {summary.get('mips_max', 0):.2f} MIPS")
        if 'mips_stdev' in summary:
            print(f"  StdDev:  {summary['mips_stdev']:.2f}")
    else:
        print("\nNo MIPS data captured (emulator may not have started)")
    
    if 'profiler_count' in summary:
        print(f"\nProfiler Data ({summary.get('profiler_count', 0)} samples):")
        if 'cycles_avg' in summary:
            print(f"  Avg cycles/instr: {summary['cycles_avg']:.0f}")
        if 'exec_pct_avg' in summary:
            print(f"  Avg exec %:       {summary['exec_pct_avg']:.0f}%")
        if 'psram_bw_avg' in summary:
            print(f"  Avg PSRAM BW:     {summary['psram_bw_avg']:.0f} KB/s")
        print(f"  Bottleneck:       {summary.get('bottleneck', 'N/A')}")
    else:
        print("\nNo profiler data captured (profiler may be disabled)")
    
    print("\n" + "=" * 60)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Capture and analyze emulator performance')
    parser.add_argument('--duration', '-d', type=int, default=DEFAULT_DURATION,
                        help=f'Capture duration in seconds (default: {DEFAULT_DURATION})')
    parser.add_argument('--label', '-l', type=str, default='capture',
                        help='Label for this capture (default: "capture")')
    parser.add_argument('--port', '-p', type=str, default=DEFAULT_SERIAL_PORT,
                        help=f'Serial port (default: {DEFAULT_SERIAL_PORT})')
    parser.add_argument('--no-save', action='store_true',
                        help='Do not save raw capture or update tracking doc')
    
    args = parser.parse_args()
    
    # Find serial port
    port = find_serial_port(args.port)
    if not port:
        print(f"Error: Could not find serial port. Tried: {args.port}")
        print("Make sure the device is connected and the port is correct.")
        sys.exit(1)
    
    if port != args.port:
        print(f"Note: Using {port} instead of {args.port}")
    
    # Capture
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    elapsed, lines = capture_serial(port, args.duration)
    
    if not lines:
        print("No data captured!")
        sys.exit(1)
    
    # Extract and analyze
    metrics = extract_metrics(lines)
    summary = calculate_summary(metrics)
    
    # Print summary
    print_summary(summary, elapsed)
    
    # Save results
    if not args.no_save:
        save_raw_capture(lines, args.label, timestamp)
        append_to_tracking_doc(summary, args.label, timestamp, elapsed)
    
    # Return exit code based on WiFi status
    if metrics['wifi_error']:
        print("\nWARNING: WiFi errors detected! Check the capture.")
        sys.exit(2)
    
    sys.exit(0)


if __name__ == '__main__':
    main()
