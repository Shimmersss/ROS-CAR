import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from offline_voice.ollama_client import OllamaClient


class LocalHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        self.server.request_path = self.path
        self.server.request_body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        payload = {'message': {'role': 'assistant', 'tool_calls': [{
            'function': {'name': 'query_status', 'arguments': {'query': 'status'}}}]}}
        encoded = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def log_message(self, *_args):
        pass


class OllamaClientTest(unittest.TestCase):
    def test_rejects_nonlocal_endpoint_and_cloud_model(self):
        for url in ('https://127.0.0.1:11435', 'http://example.com:11435'):
            with self.assertRaises(ValueError):
                OllamaClient(base_url=url)
        with self.assertRaises(ValueError):
            OllamaClient(model='qwen3:cloud')

    def test_uses_local_chat_api_and_adapts_tool_arguments(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), LocalHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            client = OllamaClient(base_url=f'http://127.0.0.1:{server.server_port}')
            message = client.complete([{'role': 'user', 'content': '查询状态'}], tools=[{
                'type': 'function', 'function': {'name': 'query_status'}}])
            self.assertEqual(server.request_path, '/api/chat')
            self.assertEqual(server.request_body['model'], 'qwen3:1.7b')
            self.assertFalse(server.request_body['think'])
            self.assertEqual(json.loads(message['tool_calls'][0]['function']['arguments']),
                             {'query': 'status'})
        finally:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()
