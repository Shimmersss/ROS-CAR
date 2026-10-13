#!/usr/bin/env python3
"""Allowlisted one-way sync. Defaults to dry-run; no deletion or remote execution."""
import argparse
import ipaddress
from pathlib import Path
import platform
import re
import shlex
import socket
import subprocess
import sys

# Clash/Surge-style TUN proxies: fake-IP range and tunnel interfaces.
FAKE_IP = ipaddress.ip_network('198.18.0.0/15')


def route_of(address):
    """(interface, gateway) the system uses for address, or None when it cannot be determined."""
    try:
        if platform.system() == 'Darwin':
            text = subprocess.check_output(['route', '-n', 'get', address], text=True, timeout=3)
            fields = dict(line.strip().split(': ', 1) for line in text.splitlines() if ': ' in line)
            return fields.get('interface', ''), fields.get('gateway', '')
        words = subprocess.check_output(['ip', 'route', 'get', address], text=True, timeout=3).split()
        pick = lambda key: words[words.index(key)+1] if key in words[:-1] else ''
        return pick('dev'), pick('via')
    except (OSError, subprocess.SubprocessError):
        return None


def tunnel_reason(host):
    """Why a LAN target is not reached directly (proxy TUN), or '' when the route looks direct."""
    try:
        address = ipaddress.ip_address(socket.gethostbyname(host))
    except (OSError, ValueError):
        return ''  # e.g. an ssh_config alias; leave it to ssh
    if address in FAKE_IP:
        return f'{host} 解析到代理假 IP {address}'
    if not address.is_private or address.is_loopback:
        return ''
    route = route_of(str(address))
    if route is None:
        return ''
    interface, gateway = route
    try:
        fake_gateway = ipaddress.ip_address(gateway) in FAKE_IP
    except ValueError:  # empty or a link-local name such as link#14
        fake_gateway = False
    if interface.startswith(('utun', 'tun')) or fake_gateway:
        return f'去往 {address} 的流量走 {interface}（网关 {gateway or "无"}），像是代理 TUN 接管了局域网'
    return ''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True, help='user@hostname or user@IPv4')
    parser.add_argument('--dest', required=True, help='absolute dedicated project directory')
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--allow-tunnel', action='store_true',
                        help='目标确实经 VPN 隧道可达时跳过代理路由检查')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9.-]*', args.host):
        parser.error('host 必须为 user@hostname 或 user@IPv4')
    if not re.fullmatch(r'/[A-Za-z0-9_./-]+', args.dest):
        parser.error('dest 必须为不含空格的绝对项目路径')
    dest = Path(args.dest)
    if '..' in dest.parts or len(dest.parts) < 4 or dest.name in {'.', '..'}:
        parser.error('请指定如 /home/user/ROSCAR 的专用项目子目录')
    reason = '' if args.allow_tunnel else tunnel_reason(args.host.split('@', 1)[1])
    if reason:
        sys.exit(f'{reason}。SSH 会被代理接住而连不到小车：关闭代理 TUN/增强模式或设局域网直连后重试；'
                 '确属 VPN 可加 --allow-tunnel。')
    root = Path(__file__).resolve().parents[1]
    # ros2_ws/src includes roscar_interfaces/roscar_api; docs includes the public API manual.
    paths = ['README.md', 'AGENTS.md', 'WORKLOG.md', '人体跟随感知方案.md',
             'ros2_ws/src', 'scripts', 'tests', 'docs', 'foxglove',
             'deploy/README.md', 'deploy/humble-test.Dockerfile', 'deploy/radar-test.Dockerfile', 'deploy/navigation-test.Dockerfile', 'deploy/chassis-test.Dockerfile', 'deploy/systemd', 'deploy/patches', 'deploy/udev',
             'models/manifest.json', 'models/weights/README.md',
             'data/catalog', 'data/recordings/README.md']
    cmd = ['rsync', '-azR', '--itemize-changes',
           '--exclude=__pycache__', '--exclude=*.pyc', '--exclude=.DS_Store',
           '--exclude=local.*', '--exclude=.env', '--exclude=.env.*']
    if not args.apply:
        cmd.append('--dry-run')
    cmd.extend(paths + [f'{args.host}:{args.dest.rstrip("/")}/'])
    print(('实际同步' if args.apply else '同步预览') + ': ' + shlex.join(cmd), flush=True)
    try:
        subprocess.run(cmd, cwd=root, check=True)
    except subprocess.CalledProcessError as exc:
        sys.exit(f'rsync 失败（退出码 {exc.returncode}）；255 通常是 SSH 连不上：检查小车是否开机、地址和路由。')


if __name__ == '__main__':
    main()
