#!/usr/bin/env python3
"""Verify repository model assets and prepare private, relocatable launch files.

No ROS, model loading, dependency installation or running-service modification.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import secrets
import shlex
import shutil
import stat
import tempfile
import zipfile


REPO = Path(__file__).resolve().parents[1]
ASSETS = REPO / 'models/inference'
TONGUE = 'tongue_cloud_server_20261015'


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def verify_assets():
    entries = json.loads((ASSETS / 'manifest.json').read_text())['files']
    for item in entries:
        path = ASSETS / item['path']
        if (not path.is_file() or path.stat().st_size != item['bytes']
                or digest(path) != item['sha256']):
            raise ValueError(f"模型文件缺失/未拉取LFS/哈希不符：{path}；请先 git lfs pull")
    return {item['kind']: item for item in entries}


def unpack_tongue(archive, runtime):
    destination = runtime / TONGUE
    if not destination.exists():
        temporary = Path(tempfile.mkdtemp(prefix='.tongue-', dir=runtime))
        try:
            with zipfile.ZipFile(archive) as bundle:
                for entry in bundle.infolist():
                    target = (temporary / entry.filename).resolve()
                    if (not target.is_relative_to(temporary / TONGUE)
                            or stat.S_ISLNK(entry.external_attr >> 16)):
                        raise ValueError('invalid_archive_member')
                bundle.extractall(temporary)
            (temporary / TONGUE).rename(destination)
        finally:
            shutil.rmtree(temporary)
    rows = json.loads((destination / 'FILES_SHA256.json').read_text())
    for row in rows:
        path = (destination / row['path']).resolve()
        if (not path.is_relative_to(destination) or not path.is_file()
                or path.stat().st_size != row['bytes'] or digest(path) != row['sha256']):
            raise ValueError(f"冻结舌诊文件校验失败：{row['path']}")
    return destination, len(rows)


def write_script(path, arguments, cwd):
    path.write_text('#!/bin/bash\nset -eu\ncd ' + shlex.quote(str(cwd))
                    + '\nexport OMP_NUM_THREADS=2 MKL_NUM_THREADS=2\nexec '
                    + shlex.join([str(value) for value in arguments]) + '\n')
    path.chmod(0o700)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, default=REPO / 'artifacts/inference-runtime')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8200)
    parser.add_argument('--tongue-port', type=int, default=8096)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    if not all(1 <= port <= 65535 for port in (args.port, args.tongue_port)) or args.port == args.tongue_port:
        parser.error('两个端口必须合法且不同')
    entries = verify_assets()
    if args.verify_only:
        print('PASS: 三个 Git LFS 资产的大小与SHA256一致')
        return
    runtime = args.runtime.expanduser().resolve()
    if runtime.is_relative_to(ASSETS):
        parser.error('运行目录不得写入模型资产目录')
    runtime.mkdir(mode=0o700, parents=True, exist_ok=True)
    tongue, count = unpack_tongue(ASSETS / entries['tongue']['path'], runtime)
    token = runtime / 'gateway.token'
    if not token.exists():
        fd = os.open(token, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(secrets.token_urlsafe(32) + '\n')
    elif not token.is_file() or token.is_symlink() or not token.read_text().strip():
        raise ValueError('invalid_existing_token_file')
    token.chmod(0o600)
    config = dict(host=args.host, port=args.port, token_file=str(token), max_queue=4,
                  timeout_sec=120, models={})
    for kind in ('pcb', 'helmet'):
        item = entries[kind]
        config['models'][kind] = dict(weights=str(ASSETS / item['path']), sha256=item['sha256'], device='cuda:0')
    config['models']['pcb']['classes'] = ['missing_hole', 'mouse_bite', 'open_circuit', 'short', 'spur', 'spurious_copper']
    config['models']['tongue'] = dict(port=args.tongue_port, device='cuda')
    config_path = runtime / 'gateway.private.yaml'
    # JSON is a YAML subset; use stdlib so preparation needs no pip environment.
    config_path.write_text(json.dumps(config, indent=2) + '\n')
    config_path.chmod(0o600)
    write_script(runtime / 'run-tongue.sh', [runtime / 'venv-tongue/bin/python',
        tongue / 'code/deploy/cloud/server.py', '--root', tongue / 'code', '--manifest',
        tongue / 'code/deploy/configs/cloud/pipeline_r50_384_cvAC10_passthrough_debug.json',
        '--device', 'cuda', '--host', '127.0.0.1', '--port', args.tongue_port,
        '--workers', '1', '--max-queue', '4', '--deadline-ms', '120000', '--uncertain-band', '0'], REPO)
    write_script(runtime / 'run-gateway.sh', [runtime / 'venv-yolo/bin/python', '-m',
        'services.inference_gateway.gateway', '--config', config_path], REPO)
    print(f'PASS: 三个资产与舌诊{count}项冻结文件校验通过；配置生成于 {runtime}')
    print('尚未安装环境或启动服务；按 docs/云端推理部署与迁移.md 继续。')


if __name__ == '__main__':
    main()
