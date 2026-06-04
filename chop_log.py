#!/usr/bin/env python3
import argparse
import csv
import os
import sys
from pymavlink import mavutil

DOWNSAMPLE_TYPES = {
    'IMU', 'IMU2', 'IMU3',
    'GYR', 'GYR2', 'GYR3',
    'ACC', 'ACC2', 'ACC3',
    'MAG', 'MAG2', 'MAG3',
    'BARO', 'BAR2', 'BAR3',
    'RATE',
    'PIDR', 'PIDP', 'PIDY', 'PIDA', 'PIDS',
    'VIBE',
    'FTN', 'FTN2', 'FTS',
    'ISBH', 'ISBD',
    # State estimation / attitude / nav outputs
    'ATT', 'AHR2', 'POS', 'ANG', 'ORGN',
    'NKF1', 'NKF2', 'NKF3', 'NKF4', 'NKF5',
    'NKQ', 'NKQ1', 'NKQ2',
    'XKF1', 'XKF2', 'XKF3', 'XKF4', 'XKF5',
    'XKQ', 'XKQ1', 'XKQ2',
    'XKFS', 'XKFD', 'XKFM',
    'XKV1', 'XKV2',
    'XKY0', 'XKY1',
    'NKT', 'XKT',
    'ATSC',
    'ESC', 'ESC1', 'ESC2', 'ESC3', 'ESC4', 'ESC5', 'ESC6', 'ESC7', 'ESC8',
}

# Format/parameter messages — keep them, but as text sidecars rather than per-row CSV
SIDECAR_TYPES = {'FMT', 'FMTU', 'UNIT', 'MULT', 'PARM', 'MSG', 'CMD'}

# Firmware file-transfer dumps (raw binary Data field) — never flight data, and the
# binary content breaks csv.writer. Always skip, even at tier 3.
SKIP_TYPES = {'FILE'}

# Secondary/redundant sensor copies and the legacy EKF2 stack.
# Skipped by default; pass --all-sensors to keep them.
REDUNDANT_TYPES = {
    'IMU2', 'IMU3',
    'ACC2', 'ACC3',
    'GYR2', 'GYR3',
    'MAG2', 'MAG3',
    'BAR2', 'BAR3',
    'ISBH', 'ISBD',                      # batch-sampled IMU (FFT) — redundant with IMU
    'NKF1', 'NKF2', 'NKF3', 'NKF4', 'NKF5',
    'NKQ', 'NKQ1', 'NKQ2', 'NKT',        # EKF2 — redundant with EKF3 (XKF*)
}

# Tiered message-type whitelist. --tier 1 = core only, --tier 2 = +tuning/debug,
# --tier 3 = keep everything (no filtering).
TIER_1 = {
    'ATT', 'RATE', 'CTUN', 'QTUN', 'NTUN', 'TECS',
    'POS', 'GPS', 'BARO', 'MAG', 'IMU', 'VIBE',
    'ARSP', 'AOA', 'BAT', 'RCIN', 'RCOU', 'XKF1',
    'ESC', 'ESC1', 'ESC2', 'ESC3', 'ESC4', 'ESC5', 'ESC6', 'ESC7', 'ESC8',
}
TIER_2 = TIER_1 | {
    'PIDR', 'PIDP', 'PIDE', 'PIDN',
    'PIQR', 'PIQP', 'PIQY', 'PIQA',
    'PSCN', 'PSCE', 'PSCD',
    'FCNS', 'CTRL', 'AETR',
    'TSIT', 'QPOS',
    'MOTB', 'RPM', 'TERR', 'ATSC',
    'ANG', 'DCM', 'AHR2', 'EAHR',
}

# Sibling-message groups that can be merged into a single CSV with an `axis` column.
PID_FW_GROUP = {'PIDR': 'R', 'PIDP': 'P', 'PIDE': 'E', 'PIDN': 'N'}
PID_Q_GROUP  = {'PIQR': 'R', 'PIQP': 'P', 'PIQY': 'Y', 'PIQA': 'A'}
PSC_GROUP    = {'PSCN': 'N', 'PSCE': 'E', 'PSCD': 'D'}


def _format_value(v):
    if isinstance(v, bytes):
        try:
            s = v.decode('utf-8', errors='replace')
        except Exception:
            return v.hex()
        # Drop control chars (NUL, \r, \n, ...) that would break csv.writer.
        return ''.join(ch for ch in s if ord(ch) >= 32 or ch == '\t').strip()
    return v


def _tier_keep_set(tier):
    if tier == 1:
        return TIER_1
    if tier == 2:
        return TIER_2
    return None  # tier 3 = keep everything


def _group_for(msg_type, group_pids, group_psc):
    if group_pids and msg_type in PID_FW_GROUP:
        return 'PID_FW', PID_FW_GROUP[msg_type]
    if group_pids and msg_type in PID_Q_GROUP:
        return 'PID_Q', PID_Q_GROUP[msg_type]
    if group_psc and msg_type in PSC_GROUP:
        return 'PSC', PSC_GROUP[msg_type]
    return None, None


def chop_log(input_file, output_dir, start_time, end_time, downsample_hz=50.0,
             all_sensors=False, tier=2, group_pids=False, group_psc=False):
    print(f"Opening {input_file}...")
    mlog = mavutil.mavlink_connection(input_file)

    os.makedirs(output_dir, exist_ok=True)

    min_interval = 1.0 / downsample_hz
    last_kept = {}
    keep_set = _tier_keep_set(tier)

    t0 = None
    writers = {}        # msg_type -> csv.writer
    files = {}          # msg_type -> open file handle
    fieldnames = {}     # msg_type -> list of field names
    counts = {}         # msg_type -> rows written
    messages_dropped = 0

    params_csv_path = os.path.join(output_dir, 'params.csv')
    params_parm_path = os.path.join(output_dir, 'params.parm')
    messages_path = os.path.join(output_dir, 'messages.txt')
    params_csv_file = open(params_csv_path, 'w', newline='')
    params_parm_file = open(params_parm_path, 'w')
    params_writer = csv.writer(params_csv_file)
    params_writer.writerow(['name', 'value'])
    messages_file = open(messages_path, 'w')
    seen_params = {}

    try:
        while True:
            msg = mlog.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            # Sidecar handling for header/param/message types
            if msg_type == 'PARM':
                name = getattr(msg, 'Name', '')
                value = getattr(msg, 'Value', '')
                # ArduPilot can emit the same PARM multiple times; keep the last value
                if seen_params.get(name) != value:
                    seen_params[name] = value
                    params_writer.writerow([name, value])
                    params_parm_file.write(f"{name},{value}\n")
                continue
            if msg_type == 'MSG':
                ts = getattr(msg, '_timestamp', 0.0)
                text = _format_value(getattr(msg, 'Message', ''))
                messages_file.write(f"{ts:.6f}\t{text}\n")
                continue
            if msg_type in SIDECAR_TYPES:
                # FMT/FMTU/UNIT/MULT/CMD: structural metadata, not flight data — skip
                continue
            if msg_type in SKIP_TYPES:
                continue
            if not all_sensors and msg_type in REDUNDANT_TYPES:
                continue
            if keep_set is not None and msg_type not in keep_set:
                continue

            # Establish log zero-time
            if t0 is None and msg._timestamp > 0:
                t0 = msg._timestamp
            if t0 is None:
                continue

            rel_time = msg._timestamp - t0
            if rel_time < start_time:
                continue
            if rel_time > end_time:
                print(f"Reached end time ({end_time}s). Stopping.")
                break

            # Downsample high-rate types. Key by (type, instance) so per-motor
            # ESC streams and per-core EKF (XKF1.C) downsample independently.
            if msg_type in DOWNSAMPLE_TYPES:
                inst = getattr(msg, 'Instance', None)
                if inst is None:
                    inst = getattr(msg, 'I', None)
                if inst is None:
                    inst = getattr(msg, 'C', None)
                dedup_key = (msg_type, inst) if inst is not None else msg_type
                prev = last_kept.get(dedup_key)
                if prev is not None and (msg._timestamp - prev) < min_interval:
                    messages_dropped += 1
                    continue
                last_kept[dedup_key] = msg._timestamp

            # Resolve output destination (grouped sibling sets share one file).
            group_name, axis = _group_for(msg_type, group_pids, group_psc)
            out_key = group_name if group_name else msg_type
            extra_header = ['axis'] if group_name else []
            extra_row = [axis] if group_name else []

            # Lazily open a CSV per output key
            if out_key not in writers:
                fields = list(msg.get_fieldnames())
                fieldnames[out_key] = fields
                path = os.path.join(output_dir, f"{out_key}.csv")
                fh = open(path, 'w', newline='')
                w = csv.writer(fh)
                w.writerow(['timestamp', 'rel_time'] + extra_header + fields)
                files[out_key] = fh
                writers[out_key] = w
                counts[out_key] = 0

            row = [f"{msg._timestamp:.6f}", f"{rel_time:.6f}"] + extra_row
            for f in fieldnames[out_key]:
                row.append(_format_value(getattr(msg, f, '')))
            writers[out_key].writerow(row)
            counts[out_key] += 1

        print("\n--- Chopping Complete ---")
        print(f"Output directory: {output_dir}")
        print(f"Params (CSV):     {params_csv_path}")
        print(f"Params (.parm):   {params_parm_path}")
        print(f"Messages written: {messages_path}")
        print(f"High-rate rows dropped (downsampled to {downsample_hz} Hz): {messages_dropped}")
        print("Per-type row counts:")
        for t in sorted(counts):
            print(f"  {t:8s} {counts[t]}")

    except Exception as e:
        print(f"Error processing log: {e}")
        sys.exit(1)
    finally:
        params_csv_file.close()
        params_parm_file.close()
        messages_file.close()
        for fh in files.values():
            fh.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Chop an ArduPilot .bin log by time and export to per-type CSV files.'
    )
    parser.add_argument('input_log', help='Path to the original .bin file')
    parser.add_argument('output_dir', help='Directory to write per-type CSV files into')
    parser.add_argument('--start', type=float, required=True, help='Start time in seconds (relative to log start)')
    parser.add_argument('--end', type=float, required=True, help='End time in seconds (relative to log start)')
    parser.add_argument('--hz', type=float, default=50.0, help='Downsample rate for high-frequency types (default: 50)')
    parser.add_argument('--all-sensors', action='store_true',
                        help='Keep secondary/redundant sensor copies (IMU2/3, MAG2/3, BAR2/3, ISBH/ISBD) '
                             'and the legacy EKF2 stack (NKF*). Off by default to shrink upload size.')
    parser.add_argument('--tier', type=int, choices=[1, 2, 3], default=2,
                        help='Message-type breadth: 1 = core only (~20 types), '
                             '2 = core + tuning/debug (default), 3 = everything.')
    parser.add_argument('--group-pids', action='store_true',
                        help='Merge PIDR/P/E/N into PID_FW.csv and PIQR/P/Y/A into PID_Q.csv '
                             '(adds an "axis" column).')
    parser.add_argument('--group-psc', action='store_true',
                        help='Merge PSCN/E/D into PSC.csv (adds an "axis" column).')

    args = parser.parse_args()

    chop_log(args.input_log, args.output_dir, args.start, args.end,
             downsample_hz=args.hz, all_sensors=args.all_sensors,
             tier=args.tier, group_pids=args.group_pids, group_psc=args.group_psc)
