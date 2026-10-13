from pathlib import Path
import threading
import pytest
from capture_task.catalog import Catalog
from capture_task.mock_cloud import MockCloud


@pytest.fixture
def catalog():
    return Catalog.load(Path(__file__).parents[1]/'config/tasks.yaml')


@pytest.fixture
def cloud(tmp_path):
    server = MockCloud(('127.0.0.1', 0), tmp_path/'cloud')
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server, 'http://127.0.0.1:'+str(server.server_port)
    server.stop(); thread.join(3)
