"""TTS voice candidates, measured profiles and error hints.

Pure data and helpers: no ROS, no network, so this is unit-testable anywhere.

iFLYTEK only synthesises voices activated for the account, so the table is a
shortlist to audition, never a guarantee. `ACTIVATED_VOICES` mirrors the console
page 发音人授权管理 -> 基础发音人; `EXTRA_VOICES` are unconfirmed and answer
11200 until they are activated.
"""

# Verified 2026-09-16 against the live account: `xiaoyan` and `aisjixu` returned
# byte-identical audio (same MD5 and length), so the legacy alias resolves to
# 讯飞许久, NOT to x4_xiaoyan (讯飞小燕) as the console entry suggests.
LEGACY_ALIAS_IS = {'xiaoyan': 'aisjixu'}

# Owner's choice after listening to all five activated voices on the vehicle
# speaker (2026-09-16). This is a listening decision, not an F0 ranking: see
# steadiest_vcns() for the objective profile winner.
DEFAULT_VOICE = 'x4_yezi'

# Measured on the same sentence, 16 kHz mono: median F0 and its spread describe
# how high and how jumpy a voice sits. A high median with a wide spread is what
# makes a voice read as stiff broadcast-style speech.
MEASURED_PROFILES = {
    'aisjixu': {'seconds': 9.24, 'f0_hz': 250.0, 'f0_spread_hz': 72.9},
    'xiaoyan': {'seconds': 9.24, 'f0_hz': 250.0, 'f0_spread_hz': 72.9},
    'x4_xiaoyan': {'seconds': 8.80, 'f0_hz': 244.3, 'f0_spread_hz': 75.7},
    'x4_yezi': {'seconds': 7.89, 'f0_hz': 250.0, 'f0_spread_hz': 54.9},
    'aisjinger': {'seconds': 10.14, 'f0_hz': 216.2, 'f0_spread_hz': 44.4},
    'aisbabyxu': {'seconds': 9.46, 'f0_hz': 285.7, 'f0_spread_hz': 69.8},
}

ACTIVATED_VOICES = (
    {'vcn': 'x4_yezi', 'label': '讯飞小露', 'gender': '女', 'group': '已开通',
     'note': '现场试听选定为默认；语速稍快，听感自然'},
    {'vcn': 'aisjinger', 'label': '讯飞小婧', 'gender': '女', 'group': '已开通',
     'note': '基频最低最平稳（216 Hz、起伏 σ44），另一备选'},
    {'vcn': 'x4_xiaoyan', 'label': '讯飞小燕', 'gender': '女', 'group': '已开通',
     'note': '控制台登记的 x4 引擎，起伏仍偏大'},
    {'vcn': 'aisbabyxu', 'label': '讯飞许小宝', 'gender': '童', 'group': '已开通',
     'note': '音高最高，童声'},
    {'vcn': 'aisjixu', 'label': '讯飞许久', 'gender': '女', 'group': '已开通',
     'note': '改动前实际生效的旧音色，音高高且起伏大'},
    {'vcn': 'xiaoyan', 'label': '小燕（旧写法）', 'gender': '女', 'group': '已开通',
     'note': '与 aisjixu 字节相同，仅为历史兼容写法'},
)

EXTRA_VOICES = (
    {'vcn': 'x4_lingxiaoxuan_oral', 'label': '聆小璇', 'gender': '女',
     'group': '需先开通', 'note': '超拟人，自然口语；实测 11200'},
    {'vcn': 'x_dangdang', 'label': '小姐姐', 'gender': '女',
     'group': '需先开通', 'note': '对话风格；实测 11200'},
    {'vcn': 'x2_xiaoxue', 'label': '小雪', 'gender': '女',
     'group': '需先开通', 'note': '甜美磁性；实测 11200'},
    {'vcn': 'x2_yifeng', 'label': '一峰', 'gender': '男',
     'group': '需先开通', 'note': '年轻时尚亲切；实测 11200'},
)

CANDIDATE_VOICES = ACTIVATED_VOICES + EXTRA_VOICES

DEFAULT_PREVIEW_TEXT = '你好，我是小车助手。请跟在我后面，保持一米距离。前面路口右转，注意脚下。'

ERROR_HINTS = {
    10105: '该 appid 没有这个发音人的权限',
    10161: '授权已过期',
    10163: '参数校验失败，发音人名称可能拼写错误',
    10164: '会话数量超限',
    10165: '鉴权失败',
    11200: '该发音人未在当前 appid 的控制台开通或激活',
}


def find_voice(vcn):
    """Return the candidate record for `vcn`, or None when not on the list."""
    for record in CANDIDATE_VOICES:
        if record['vcn'] == vcn:
            return record
    return None


def activated_vcns():
    """vcn values that were confirmed usable against the live account."""
    return tuple(record['vcn'] for record in ACTIVATED_VOICES)


def steadiest_vcns():
    """Activated voices ranked by how low and steady they sit, steadiest first.

    The legacy alias is skipped so one voice cannot appear twice. This is the
    objective ranking; it does not have to agree with DEFAULT_VOICE, which is a
    listening decision.
    """
    return tuple(sorted(
        (vcn for vcn in activated_vcns() if vcn not in LEGACY_ALIAS_IS),
        key=lambda vcn: (MEASURED_PROFILES[vcn]['f0_hz']
                         + MEASURED_PROFILES[vcn]['f0_spread_hz']),
    ))


def explain_error(code, message=''):
    """Human hint for an iFLYTEK TTS error code, falling back to the server text."""
    hint = ERROR_HINTS.get(int(code))
    if hint is None:
        return message or '未知错误'
    if message and message not in hint:
        return f'{hint}（服务端原文：{message}）'
    return hint


def grouped_candidates():
    """Candidates grouped by their display group, order preserved."""
    groups = {}
    for record in CANDIDATE_VOICES:
        groups.setdefault(record['group'], []).append(record)
    return groups
