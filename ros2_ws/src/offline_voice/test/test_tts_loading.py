"""Reject incomplete Matcha deployments without leaking a speech queue worker."""
import threading
from unittest.mock import patch

import pytest

rclpy = pytest.importorskip('rclpy')
sherpa_onnx = pytest.importorskip('sherpa_onnx')
from offline_voice import tts_node


@pytest.mark.parametrize('missing', ['model-steps-3.onnx', 'lexicon.txt', 'tokens.txt',
                                    'date.fst', 'number.fst', 'vocos-22khz-univ.onnx'])
def test_missing_asset_cleans_up_queue(tmp_path, missing):
    model = tmp_path/'matcha-icefall-zh-baker'
    model.mkdir()
    for name in ('model-steps-3.onnx', 'lexicon.txt', 'tokens.txt', 'date.fst', 'number.fst'):
        if name != missing:
            (model/name).touch()
    if missing != 'vocos-22khz-univ.onnx':
        (tmp_path/'vocos-22khz-univ.onnx').touch()
    before = set(threading.enumerate())
    rclpy.init()
    try:
        with patch.object(tts_node, 'DEFAULT_MODEL_ROOT', tmp_path):
            with patch.object(sherpa_onnx, 'OfflineTts') as load:
                with pytest.raises(RuntimeError, match='缺失'):
                    tts_node.OfflineTtsNode()
                load.assert_not_called()
        assert set(threading.enumerate()) == before
    finally:
        rclpy.shutdown()


def test_native_model_error_cleans_up_queue(tmp_path):
    before = set(threading.enumerate())
    rclpy.init()
    try:
        with patch.object(tts_node.OfflineTtsNode, '_load_tts', side_effect=RuntimeError('invalid model')):
            with pytest.raises(RuntimeError, match='invalid model'):
                tts_node.OfflineTtsNode()
        assert set(threading.enumerate()) == before
    finally:
        rclpy.shutdown()
