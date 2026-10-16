"""Validated independent catalog; no voice/manager imports."""
from copy import deepcopy
import math
import re
import unicodedata
import yaml

# Copied from execution_mode_manager/catalog.py RESERVED_STOP (2026-10-12).
# Emergency KWS hears the speaker too. Keep this list in sync during reviews.
RESERVED_STOP = {'停止', '急停', '停下', '停车', '别动', '退出遥控', '取消遥控', '不要动',
                 '撤销授权', '不用了', '退出语音', '休眠', '解除紧急制动', '复位故障'}


def speech_text(text):
    if not isinstance(text, str) or not text.strip() or len(text) > 300:
        raise ValueError('invalid_text')
    normalized = ''.join(c for c in unicodedata.normalize('NFKC', text)
                         if not c.isspace() and unicodedata.category(c)[0] not in 'PCZ')
    if any(word in normalized for word in RESERVED_STOP):
        raise ValueError('reserved_speech')
    return text


def number(value, lo, hi):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError('invalid_number')
    return value


class Catalog:
    def __init__(self, data):
        if not isinstance(data, dict) or set(data) != {'tasks', 'defaults', 'params_whitelist'}:
            raise ValueError('invalid_catalog')
        self.data = deepcopy(data)
        self.defaults = self.data['defaults']
        self.tasks = self.data['tasks']
        self.whitelist = self.data['params_whitelist']
        if not isinstance(self.whitelist, list) or len(set(self.whitelist)) != len(self.whitelist) or set(self.whitelist) - {'announce', 'prepare_sec'}:
            raise ValueError('invalid_whitelist')
        if not isinstance(self.tasks, dict) or not 1 <= len(self.tasks) <= 32:
            raise ValueError('invalid_tasks')
        bounds = {'task_timeout_sec': (.1, 600), 'speech_timeout_sec': (.1, 120),
                  'frame_timeout_sec': (.05, 10), 'max_frame_age_sec': (.01, .5), 'jpeg_quality': (1, 100)}
        required_defaults = set(bounds) | {'quality_policy', 'done_announce', 'fail_announce'}
        if not required_defaults <= set(self.defaults) or set(self.defaults) - required_defaults - {'analysis_done_announce', 'retake_announce'}:
            raise ValueError('invalid_defaults')
        for key, (lo, hi) in bounds.items():
            number(self.defaults[key], lo, hi)
        if type(self.defaults['jpeg_quality']) is not int or self.defaults['quality_policy'] not in ('warn', 'fail'):
            raise ValueError('invalid_quality_policy')
        speech_text(self.defaults['done_announce']); speech_text(self.defaults['fail_announce'])
        for key in ('analysis_done_announce', 'retake_announce'):
            if key in self.defaults:
                speech_text(self.defaults[key])
        for kind, task in self.tasks.items():
            if not isinstance(kind, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,31}', kind):
                raise ValueError('invalid_task_type')
            required = {'name', 'announce', 'prepare_sec', 'burst', 'upload', 'min_width', 'quality', 'require_hold'}
            if not isinstance(task, dict) or not required <= set(task) or set(task) - required - {'inference'}:
                raise ValueError('invalid_task')
            if 'inference' in task:
                infer = task['inference']
                if not isinstance(infer, dict) or set(infer) != {'timeout_sec', 'speak_summary'} or type(infer['speak_summary']) is not bool:
                    raise ValueError('invalid_inference_options')
                number(infer['timeout_sec'], .1, 600)
            speech_text(task['announce'])
            if not isinstance(task['name'], str) or not 1 <= len(task['name']) <= 100:
                raise ValueError('invalid_name')
            number(task['prepare_sec'], 0, 30)
            number(task['burst'], 1, 10); number(task['min_width'], 1, 10000)
            if type(task['burst']) is not int or type(task['min_width']) is not int or type(task['require_hold']) is not bool or task['upload'] not in ('best', 'all'):
                raise ValueError('invalid_task_options')
            q = task['quality']
            if not isinstance(q, dict) or set(q) != {'brightness', 'min_sharpness', 'rb_ratio'}:
                raise ValueError('invalid_quality')
            for field, lo, hi in [('brightness', 0, 255), ('rb_ratio', 0, 100)]:
                pair = q[field]
                if not isinstance(pair, list) or len(pair) != 2:
                    raise ValueError('invalid_range')
                number(pair[0], lo, hi); number(pair[1], lo, hi)
                if pair[0] > pair[1]:
                    raise ValueError('invalid_range')
            number(q['min_sharpness'], 0, 1e8)

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as handle:
            return cls(yaml.safe_load(handle))

    def resolve(self, kind, params):
        if not isinstance(kind, str) or kind not in self.tasks or not isinstance(params, dict) or set(params) - set(self.whitelist):
            raise ValueError('invalid_task_or_params')
        task = deepcopy(self.tasks[kind])
        if 'announce' in params:
            speech_text(params['announce'])
        if 'prepare_sec' in params:
            number(params['prepare_sec'], 0, 30)
        task.update(params)
        return task
