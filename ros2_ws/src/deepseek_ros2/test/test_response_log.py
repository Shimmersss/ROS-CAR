from datetime import datetime, timezone
import json

from deepseek_ros2.response_log import append_response


def test_append_response_writes_utf8_json_lines(tmp_path):
    destination = tmp_path / 'nested' / 'responses.jsonl'
    timestamp = datetime(2026, 9, 14, 14, 0, tzinfo=timezone.utc)

    append_response(destination, '中文回答', timestamp=timestamp)
    append_response(destination, '第二条', timestamp=timestamp)

    records = [
        json.loads(line)
        for line in destination.read_text(encoding='utf-8').splitlines()
    ]
    assert records == [
        {'timestamp': '2026-09-14T14:00:00+00:00', 'answer': '中文回答'},
        {'timestamp': '2026-09-14T14:00:00+00:00', 'answer': '第二条'},
    ]
