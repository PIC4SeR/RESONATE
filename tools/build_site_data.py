#!/usr/bin/env python3
"""Build the gh-pages assets from the RESONATE Zenodo dataset.

Copies videos/<scenario>/<algorithm>/run_<k>.mp4 (remuxed with +faststart so
browsers can start playing before the whole file is downloaded) and writes
data/runs.json from metrics/runs.csv.

Usage: python3 tools/build_site_data.py /path/to/zenodo_resonate
"""
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

COLUMNS = {
    'time_to_reach_goal': 'ttg',
    'path_length': 'pl',
    'cumulative_heading_changes': 'chc',
    'minimum_distance_to_people': 'min_dist',
    'social_work_per_second': 'sw_s',
    'avg_robot_linear_speed': 'v',
    'Safety': 'SA',
    'Friendliness': 'FR',
    'Naturalness': 'NA',
    'Avoidance foresight': 'AF',
}


def number(text):
    try:
        value = float(text)
    except ValueError:
        return None
    return None if math.isnan(value) else round(value, 3)


def main(dataset):
    site = Path(__file__).resolve().parent.parent
    runs = []
    with open(dataset / 'metrics' / 'runs.csv', newline='') as f:
        for row in csv.DictReader(f):
            run = {'scenario': row['scenario'], 'algorithm': row['algorithm'],
                   'run': int(row['run']), 'result': row['result']}
            run.update({key: number(row[col]) for col, key in COLUMNS.items()})
            runs.append(run)

            rel = Path('videos', row['scenario'], row['algorithm'], f"run_{row['run']}.mp4")
            dst = site / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', str(dataset / rel),
                            '-c', 'copy', '-movflags', '+faststart', str(dst)], check=True)

    (site / 'data').mkdir(exist_ok=True)
    with open(site / 'data' / 'runs.json', 'w') as f:
        json.dump(runs, f, separators=(',', ':'))
    print(f'{len(runs)} runs written')


if __name__ == '__main__':
    main(Path(sys.argv[1]))
