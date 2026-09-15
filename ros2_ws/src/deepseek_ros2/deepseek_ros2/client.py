"""Dependency-free DeepSeek Chat Completions client."""

import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class DeepSeekError(RuntimeError):
    """Raised for transport errors or malformed API responses."""


def extract_assistant_text(payload):
    message = extract_assistant_message(payload)
    text = message.get('content')
    if not isinstance(text, str) or not text.strip():
        raise DeepSeekError('DeepSeek 返回了空回答')
    return text.strip()


def extract_assistant_message(payload):
    try:
        message = payload['choices'][0]['message']
    except (KeyError, IndexError, TypeError) as exc:
        raise DeepSeekError('DeepSeek 响应缺少 choices[0].message') from exc
    if not isinstance(message, dict):
        raise DeepSeekError('DeepSeek 返回了无效 assistant message')
    return message


class DeepSeekClient:
    def __init__(self, api_key, *, base_url='https://api.deepseek.com',
                 model='deepseek-v4-flash', timeout_sec=30.0,
                 max_tokens=300, thinking_enabled=False):
        if not api_key:
            raise ValueError('api_key is required')
        self.api_key = api_key
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.timeout_sec = float(timeout_sec)
        self.max_tokens = int(max_tokens)
        self.thinking_enabled = bool(thinking_enabled)

    def complete(self, messages, *, tools=None, tool_choice=None):
        body = {
            'model': self.model,
            'messages': messages,
            'stream': False,
            'max_tokens': self.max_tokens,
            'thinking': {
                'type': 'enabled' if self.thinking_enabled else 'disabled',
            },
        }
        if tools:
            body['tools'] = tools
            body['tool_choice'] = tool_choice or 'auto'
        request = Request(
            self.base_url + '/chat/completions',
            data=json.dumps(body, ensure_ascii=False).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )
        try:
            with urlopen(request, timeout=self.timeout_sec) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except HTTPError as exc:
            detail = exc.read(2048).decode('utf-8', errors='replace')
            raise DeepSeekError(f'DeepSeek HTTP {exc.code}: {detail}') from exc
        except URLError as exc:
            raise DeepSeekError(f'DeepSeek 网络错误: {exc.reason}') from exc
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise DeepSeekError('DeepSeek 返回了无效 JSON') from exc
        return extract_assistant_message(payload)

    def chat(self, messages):
        message = self.complete(messages)
        return extract_assistant_text({'choices': [{'message': message}]})
