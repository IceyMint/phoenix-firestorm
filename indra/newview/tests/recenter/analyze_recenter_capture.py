#!/usr/bin/env python3
"""Extract recenter-only evidence; logged alignment is not physical-headset acceptance."""
import argparse
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path)
    parser.add_argument('--out', type=Path, default=Path('recenter-capture'))
    args = parser.parse_args()
    candidates = [Path.home() / folder / 'logs/Firestorm.log'
                  for folder in ('.firestorm_x64', '.firestormos_x64')]
    existing = [p for p in candidates if p.is_file()]
    log = args.log or (max(existing, key=lambda p: p.stat().st_mtime) if existing else None)
    if log is None:
        parser.error('No Firestorm.log found; specify --log with its path.')
    names = ('Reference space change pending', 'P2-04b runtime identity',
             'Recenter stable STAGE', 'xr_recenter_', 'recenter_interlock_rejected',
             'recenter_body_alignment_skipped', 'xr_local_calibration_continuity')
    selected = []
    commands = {}
    headings = {}
    for line in log.read_text(errors='replace').splitlines():
        if not any(name in line for name in names):
            continue
        selected.append(line)
        start = line.find('{')
        if start < 0:
            continue
        try:
            record = json.loads(line[start:])
        except json.JSONDecodeError:
            continue
        key = str(record.get('change_time_ns', ''))
        if 'xr_recenter_applied:' in line:
            commands[key] = record
        if 'xr_recenter_render_heading:' in line:
            headings.setdefault(key, []).append(record)
    results = []
    for key, command in commands.items():
        frames = headings.get(key, [])
        settled = frames[-3:]
        errors = [abs(r['render_root_error_rad']) * 180 / math.pi for r in settled]
        results.append(dict(change_time_ns=key, yaw_source=command.get('yaw_source'),
                            body_turn_rad=command.get('body_turn_rad'),
                            heading_samples=len(frames), final_root_errors_deg=errors,
                            logged_root_settled_within_5deg=len(errors) == 3 and max(errors) <= 5))
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'recenter-only.log').write_text('\n'.join(selected) + '\n')
    report = dict(log=str(log), commands=results,
                  verdict='No accepted body command' if not results else 'Inspect per-event logged heading results',
                  physical_headset_acceptance_verified=False, simulator_ack_verified=False)
    (args.out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
