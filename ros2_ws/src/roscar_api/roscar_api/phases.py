"""Brief transition announcements. Selection and motion remain independent."""
OWNER_TEXT = {
    'ready': '等待主人认领', 'claimed': '已认领主人', 'lost': '主人丢失，等待返回',
    'returned': '主人已返回', 'expired': '主人未返回，可重新认领', 'released': '已解除认领',
}


class PhaseTransitions:
    def __init__(self):
        self.owner_key = None
        self.control_key = None
        self.follow_started = False

    def owner(self, state, event, sequence):
        key = state, event, sequence
        if key == self.owner_key:
            return None
        self.owner_key = key
        return OWNER_TEXT.get(event)

    def control(self, state):
        key = state.get('command_mode'), state.get('authority_state')
        if key == self.control_key:
            return None
        previous, self.control_key = self.control_key, key
        mode, authority = key
        if authority == 'ESTOP':
            self.follow_started = False
            return '已紧急制动'
        if authority == 'FAULT':
            if not str(state.get('last_fault', '')).startswith('target_'):
                return '跟随暂停，请检查' if mode == 'FOLLOW' else None
            return None  # Owner loss already has a single announcement.
        if mode == 'FOLLOW' and authority == 'ARMED_FOLLOW' and not self.follow_started:
            self.follow_started = True
            return '开始跟随'
        if mode == 'IDLE':
            self.follow_started = False
            if previous and previous[0] == 'FOLLOW':
                return '跟随已暂停'
        return None
