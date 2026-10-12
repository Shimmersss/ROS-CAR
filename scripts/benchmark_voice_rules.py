#!/usr/bin/env python3
"""Replay authored ASR text through fixed rules, without ROS or hardware."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import types


ROOT = Path(__file__).resolve().parents[1]
PARSER_PATH = 'ros2_ws/src/voice_command_router/voice_command_router/validation.py'
CASES_PATH = 'ros2_ws/src/voice_command_router/test/fixtures/voice_rule_cases.jsonl'
MOTION = {'ARM', 'FOLLOW', 'FORWARD', 'REVERSE', 'LEFT', 'RIGHT'}
COMMANDS = MOTION | {'STOP', 'STATUS'}


def load_parser(source, name):
    module = types.ModuleType(name)
    sys.modules[name] = module  # dataclasses resolves annotations via this entry
    exec(compile(source, name, 'exec'), module.__dict__)
    return module


def classify(parsed):
    if parsed is None:
        return 'NONE'
    action = parsed['action']
    if action == 'SET_MODE':
        return 'FOLLOW' if parsed['mode'] == 'FOLLOW' else action
    if action == 'QUERY_STATUS':
        return 'UNSUPPORTED' if parsed['query'].startswith('unsupported_') else 'STATUS'
    if action == 'DRIVE':
        linear, angular = parsed['linear_mps'], parsed['angular_rps']
        if linear != 0 and angular != 0:
            return 'INVALID_COMBINED_DRIVE'
        if linear:
            return 'FORWARD' if linear > 0 else 'REVERSE'
        return 'LEFT' if angular > 0 else 'RIGHT'
    return action


def evaluate(module, cases):
    counts = Counter()
    categories = {}
    failures = []
    for case in cases:
        error = None
        try:
            parsed = module.parse_local_text(case['utterance'])
        except module.CommandValidationError as exc:
            parsed, error = None, str(exc)
        predicted = classify(parsed)
        expected = case['expected']
        correct = predicted == expected
        counts['total'] += 1
        counts['correct'] += correct
        counts['command_total'] += expected in COMMANDS
        counts['command_correct'] += expected in COMMANDS and correct
        counts['stop_total'] += expected == 'STOP'
        counts['stop_correct'] += expected == predicted == 'STOP'
        counts['unexpected_motion'] += predicted in MOTION and expected not in MOTION
        counts['wrong_executable_action'] += predicted in MOTION | {'STOP'} and not correct
        counts['non_command_total'] += expected in ('NONE', 'UNSUPPORTED')
        counts['non_command_executions'] += (expected in ('NONE', 'UNSUPPORTED')
                                             and predicted in MOTION | {'STOP'})
        category = categories.setdefault(case['category'], {'total': 0, 'correct': 0})
        category['total'] += 1
        category['correct'] += correct
        if not correct:
            failures.append({'id': case['id'], 'utterance': case['utterance'],
                             'expected': expected, 'predicted': predicted, 'error': error})
    return {'counts': dict(counts), 'categories': categories, 'failures': failures}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, default=ROOT / CASES_PATH)
    parser.add_argument('--baseline-ref', default='59847c0', help='Git revision of the old rules')
    parser.add_argument('--output', type=Path, help='Optional JSON report path')
    args = parser.parse_args()
    cases_text = args.cases.read_text(encoding='utf-8')
    cases = [json.loads(line) for line in cases_text.splitlines() if line]
    if not cases:
        parser.error('The case file is empty')
    revision = subprocess.check_output(
        ['git', '-c', f'safe.directory={ROOT}', 'rev-parse', '--verify', args.baseline_ref],
        cwd=ROOT, text=True).strip()
    source = subprocess.check_output(
        ['git', '-c', f'safe.directory={ROOT}', 'show', f'{revision}:{PARSER_PATH}'],
        cwd=ROOT, text=True)
    baseline = evaluate(load_parser(source, '_voice_rules_baseline'), cases)
    current_source = (ROOT / PARSER_PATH).read_text(encoding='utf-8')
    current = evaluate(load_parser(current_source, '_voice_rules_current'), cases)
    report = {'dataset': str(args.cases.resolve()), 'source': 'synthetic_authored',
              'dataset_sha256': hashlib.sha256(cases_text.encode()).hexdigest(),
              'current_parser_sha256': hashlib.sha256(current_source.encode()).hexdigest(),
              'baseline_revision': revision, 'baseline': baseline, 'current': current,
              'limitation': 'Visible text regression cases, not recorded speech or a blind test.'}
    for label, result in [('baseline', baseline), ('current', current)]:
        c = result['counts']
        print(f"{label}: labels {c['correct']}/{c['total']}; "
              f"commands {c['command_correct']}/{c['command_total']}; "
              f"stop {c['stop_correct']}/{c['stop_total']}; "
              f"non-command executions {c['non_command_executions']}/{c['non_command_total']}; "
              f"unexpected motion {c['unexpected_motion']}; "
              f"wrong executable action {c['wrong_executable_action']}")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print(f'Report: {args.output}')
    # Recognition gaps are reported; any wrong executable action fails the run.
    return int(current['counts']['wrong_executable_action'] > 0)


if __name__ == '__main__':
    raise SystemExit(main())
