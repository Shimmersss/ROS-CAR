"""Small model port shared by real and test backends."""


class Unavailable:
    def __init__(self, reason='not_configured'):
        self.reason = reason

    def ready(self):
        return False

    def info(self):
        return {'name': 'unavailable', 'version': '', 'weights_sha256': '',
                'device': 'none', 'ready': False, 'reason': self.reason}

    def infer(self, image, metadata, deadline):
        return {'status': 'model_unavailable', 'result': {},
                'summary': {'positive': False, 'count': 0, 'text_key': 'unavailable'}}
