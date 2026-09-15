"""Dependency-free validation for untrusted model tool calls."""

import json


class CommandValidationError(ValueError):
    pass


def parse_buzz_command(raw):
    try:
        command = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise CommandValidationError('工具调用不是有效 JSON') from exc
    if not isinstance(command, dict):
        raise CommandValidationError('工具调用必须是 JSON 对象')
    if set(command) - {'request_id', 'name', 'arguments'}:
        raise CommandValidationError('工具调用包含未知字段')
    if command.get('name') != 'buzz':
        raise CommandValidationError('工具不在本地白名单中')
    request_id = command.get('request_id')
    if not isinstance(request_id, str) or not request_id or len(request_id) > 128:
        raise CommandValidationError('request_id 无效')
    arguments = command.get('arguments')
    if not isinstance(arguments, dict) or set(arguments) - {'duration_ms'}:
        raise CommandValidationError('buzz 参数无效')
    duration_ms = arguments.get('duration_ms', 300)
    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int):
        raise CommandValidationError('duration_ms 必须是整数')
    if not 100 <= duration_ms <= 2000:
        raise CommandValidationError('duration_ms 必须在 100 到 2000 之间')
    return request_id, duration_ms
