"""Small local-only Ollama client compatible with the existing tool router."""

import json
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, Request, build_opener


class LocalModelError(RuntimeError):
    pass


class OllamaClient:
    def __init__(self, *, base_url='http://127.0.0.1:11435', model='qwen3:1.7b',
                 timeout_sec=25.0, max_tokens=160, context_tokens=2048):
        parsed = urlsplit(base_url)
        if parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost') \
                or parsed.username or parsed.password or parsed.path not in ('', '/'):
            raise ValueError('离线模型地址只能是本机 HTTP 服务')
        if not model or ':cloud' in model or '/' in model:
            raise ValueError('只允许已下载到本机的模型名称')
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.timeout_sec = float(timeout_sec)
        self.max_tokens = int(max_tokens)
        self.context_tokens = int(context_tokens)

    def complete(self, messages, *, tools=None, tool_choice=None):
        del tool_choice  # Ollama chooses among the supplied local tools.
        body = {
            'model': self.model,
            'messages': messages,
            'stream': False,
            'think': False,
            'keep_alive': '5m',
            'options': {
                'num_ctx': self.context_tokens,
                'num_predict': self.max_tokens,
                'temperature': 0.1,
            },
        }
        if tools:
            body['tools'] = tools
        request = Request(
            self.base_url + '/api/chat',
            data=json.dumps(body, ensure_ascii=False).encode('utf-8'),
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            # Ignore process-wide proxy settings; the offline request must stay on loopback.
            with build_opener(ProxyHandler({})).open(
                    request, timeout=self.timeout_sec) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except HTTPError as exc:
            raise LocalModelError(f'本地模型 HTTP {exc.code}') from exc
        except URLError as exc:
            raise LocalModelError(f'本地模型连接失败: {exc.reason}') from exc
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise LocalModelError('本地模型返回了无效 JSON') from exc
        message = payload.get('message')
        if not isinstance(message, dict):
            raise LocalModelError('本地模型响应缺少 message')
        # The existing ROS tool handler accepts OpenAI-style JSON argument strings.
        for call in message.get('tool_calls') or []:
            function = call.get('function') or {}
            arguments = function.get('arguments')
            if isinstance(arguments, dict):
                function['arguments'] = json.dumps(arguments, ensure_ascii=False)
        return message
