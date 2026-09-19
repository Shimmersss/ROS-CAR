import re
import unittest
from pathlib import Path

from xfyun_speech.voices import (
    ACTIVATED_VOICES,
    CANDIDATE_VOICES,
    DEFAULT_PREVIEW_TEXT,
    DEFAULT_VOICE,
    LEGACY_ALIAS_IS,
    MEASURED_PROFILES,
    activated_vcns,
    explain_error,
    find_voice,
    grouped_candidates,
    steadiest_vcns,
)

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'voice_assistant.yaml'


class CandidateTableTest(unittest.TestCase):
    def test_entries_are_complete(self):
        for record in CANDIDATE_VOICES:
            with self.subTest(vcn=record.get('vcn')):
                self.assertTrue(record['vcn'].strip())
                self.assertTrue(record['label'].strip())
                self.assertIn(record['gender'], ('男', '女', '童'))
                self.assertTrue(record['group'].strip())
                self.assertTrue(record['note'].strip())

    def test_vcn_values_are_unique(self):
        values = [record['vcn'] for record in CANDIDATE_VOICES]
        self.assertEqual(len(values), len(set(values)))

    def test_console_activated_vcns_are_present(self):
        for vcn in ('x4_xiaoyan', 'x4_yezi', 'aisjixu', 'aisjinger', 'aisbabyxu'):
            with self.subTest(vcn=vcn):
                self.assertIsNotNone(find_voice(vcn))

    def test_grouping_preserves_order(self):
        groups = grouped_candidates()
        self.assertEqual(
            sum(len(items) for items in groups.values()), len(CANDIDATE_VOICES))
        flat = [record['vcn'] for items in groups.values() for record in items]
        self.assertEqual(flat, [record['vcn'] for record in CANDIDATE_VOICES])

    def test_preview_text_is_a_realistic_utterance(self):
        self.assertIn('小车', DEFAULT_PREVIEW_TEXT)


class RecommendationTest(unittest.TestCase):
    def test_default_voice_is_activated(self):
        self.assertIn(DEFAULT_VOICE, activated_vcns())

    def test_every_activated_voice_has_a_measured_profile(self):
        for vcn in activated_vcns():
            with self.subTest(vcn=vcn):
                self.assertIn(vcn, MEASURED_PROFILES)

    def test_measured_profiles_are_physically_plausible(self):
        for vcn, profile in MEASURED_PROFILES.items():
            with self.subTest(vcn=vcn):
                self.assertGreater(profile['seconds'], 0.0)
                self.assertTrue(60.0 <= profile['f0_hz'] <= 400.0)
                self.assertGreaterEqual(profile['f0_spread_hz'], 0.0)

    def test_legacy_alias_points_at_an_activated_voice(self):
        for alias, target in LEGACY_ALIAS_IS.items():
            self.assertIn(alias, activated_vcns())
            self.assertIn(target, activated_vcns())

    def test_legacy_alias_profile_matches_its_target(self):
        for alias, target in LEGACY_ALIAS_IS.items():
            self.assertEqual(MEASURED_PROFILES[alias], MEASURED_PROFILES[target])

    def test_objective_ranking_puts_the_steadiest_first(self):
        self.assertEqual(steadiest_vcns()[0], 'aisjinger')

    def test_objective_ranking_covers_every_activated_voice_once(self):
        self.assertEqual(
            sorted(steadiest_vcns()),
            sorted(vcn for vcn in activated_vcns()
                   if vcn not in LEGACY_ALIAS_IS))

    def test_objective_ranking_matches_measured_profiles(self):
        scores = [MEASURED_PROFILES[vcn]['f0_hz']
                  + MEASURED_PROFILES[vcn]['f0_spread_hz']
                  for vcn in steadiest_vcns()]
        self.assertEqual(scores, sorted(scores))


class ShippedConfigTest(unittest.TestCase):
    def test_parameter_file_uses_an_activated_voice(self):
        text = CONFIG.read_text(encoding='utf-8')
        match = re.search(r'^[ \t]*voice_name:[ \t]*(\S+)[ \t]*$', text, re.MULTILINE)
        self.assertIsNotNone(match, 'voice_assistant.yaml 缺少 voice_name')
        self.assertIn(match.group(1), activated_vcns())


class ErrorHintTest(unittest.TestCase):
    def test_known_code_maps_to_hint(self):
        self.assertIn('控制台', explain_error(11200))

    def test_server_message_is_appended(self):
        self.assertIn('boom', explain_error(11200, 'boom'))

    def test_message_is_repeated_only_once(self):
        text = explain_error(11200, '该发音人未在当前 appid 的控制台开通或激活')
        self.assertEqual(text.count('控制台'), 1)

    def test_unknown_code_falls_back_to_message(self):
        self.assertEqual(explain_error(99999, 'some server text'), 'some server text')

    def test_unknown_code_without_message(self):
        self.assertEqual(explain_error(99999), '未知错误')

    def test_string_code_is_accepted(self):
        self.assertIn('控制台', explain_error('11200'))


if __name__ == '__main__':
    unittest.main()
