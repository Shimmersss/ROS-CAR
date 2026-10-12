"""Exact, bounded voice intents. Shared by router and manager, without ROS imports."""
import re
import unicodedata
import yaml
RESERVED_STOP = {'停止', '急停', '停下', '停车', '别动', '退出遥控', '取消遥控', '不要动',
                 '撤销授权', '不用了', '退出语音', '休眠', '解除紧急制动', '复位故障'}


def normalize(text):
    value = unicodedata.normalize('NFKC', str(text))
    value = re.sub(r'[\s，。、,.!！]', '', value)
    while True:
        stripped = re.sub(r'^(?:麻烦你|麻烦|请你|请|帮我|小车|现在)', '', value, count=1)
        if stripped == value:
            break
        value = stripped
    return value


class Catalog:
    def __init__(self, data):
        self.modes = {}
        self.phrases = {}
        for row in data.get('modes', []):
            mode = row.get('id', '')
            if not re.fullmatch(r'[a-z][a-z0-9_]{0,31}', mode) or mode in self.modes:
                raise ValueError('Invalid or duplicate mode id')
            if not re.fullmatch(r'/[A-Za-z0-9_/]+', row.get('action', '')):
                raise ValueError('Invalid action endpoint')
            self.modes[mode] = row
            for key, op in (('enter', 'ENTER'), ('retake', 'RETAKE'),
                            ('repeat', 'REPEAT'), ('exit', 'EXIT')):
                for phrase in row.get(key, []):
                    word = normalize(phrase)
                    if not word or word in self.phrases or word in RESERVED_STOP:
                        raise ValueError('Empty or overlapping voice phrase: ' + word)
                    self.phrases[word] = (mode, op)
            for op, phrases in row.get('commands', {}).items():
                if not re.fullmatch(r'[A-Z][A-Z0-9_]{0,31}', op) or op in ('ENTER', 'EXIT', 'RETAKE', 'REPEAT'):
                    raise ValueError('Invalid custom mode operation')
                for phrase in phrases:
                    word = normalize(phrase)
                    if not word or word in self.phrases or word in RESERVED_STOP:
                        raise ValueError('Overlapping/reserved custom command')
                    self.phrases[word] = (mode, op)
        for phrase in data.get('common_exit', []):
            word = normalize(phrase)
            if not word or word in self.phrases or word in RESERVED_STOP:
                raise ValueError('Overlapping exit phrase')
            self.phrases[word] = ('', 'EXIT')

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            return cls(yaml.safe_load(f) or {})

    def operations(self, mode):
        row = self.modes[mode]
        values = set(row.get('commands', {}))
        if row.get('retake'): values.add('RETAKE')
        if row.get('repeat'): values.add('REPEAT')
        return values

    def parse(self, text, active=''):
        word = normalize(text)
        # Negation is handled before affirmative matching. No substring triggers.
        for prefix in ('不要', '别', '不用'):
            if word.startswith(prefix):
                intent = self.phrases.get(word[len(prefix):])
                return (intent[0], 'EXIT') if intent else None
        intent = self.phrases.get(word)
        if intent and intent[1] in ('RETAKE', 'REPEAT') and not active:
            # Manager reports there is no session; it does not silently take a photo.
            return intent
        return intent
