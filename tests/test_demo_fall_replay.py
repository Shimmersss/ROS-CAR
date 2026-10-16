"""Offline evaluation integrity; no hardware, models or recordings required."""
import sys
import json
import tempfile
from types import SimpleNamespace
import numpy as np
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import replay_falls as replay


class ReplayTests(unittest.TestCase):
    def test_depth_cache_rejects_wrong_threshold_profile_or_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = root/'profile.yaml'
            profile.write_text('profile')
            (root/'frames.jsonl').write_text('frames')
            self.assertFalse(replay.depth_cache_matches(root, profile, .35))
            marker = root/'rebuilt-depth.json'
            marker.write_text(json.dumps(replay.depth_cache_metadata(root, profile, .35)))
            self.assertTrue(replay.depth_cache_matches(root, profile, .35))
            self.assertFalse(replay.depth_cache_matches(root, profile, .5))
            profile.write_text('changed')
            self.assertFalse(replay.depth_cache_matches(root, profile, .35))
            marker.write_text(json.dumps(replay.depth_cache_metadata(root, profile, .35)))
            (root/'frames.jsonl').write_text('changed')
            self.assertFalse(replay.depth_cache_matches(root, profile, .35))

    def test_depth_rebuild_uses_new_threshold_and_clears_unmatched(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory)/'profile.yaml'
            profile.write_text('/**:\n  ros__parameters: {}\n')
            frame = dict(depth_matched=True, depth_frame='1', people=[
                dict(box=(0,0,20,20), keypoints=[], joints3d=[[99]*3]*17),
                dict(box=(30,0,40,20), keypoints=[], joints3d=[[99]*3]*17)])
            reader = SimpleNamespace(reg=SimpleNamespace(apply=lambda blank, depth: (blank, depth)),
                                     blank=None, depths={'1': {}}, intrinsics=(1,1,1,1))
            with patch.object(replay, 'Continuation', return_value=reader), \
                 patch.object(replay, 'load_depth_m', return_value=np.ones((20,40))), \
                 patch('yolo_person_tracker.fusion.fuse_depth', return_value=SimpleNamespace(joints=np.ones((17,3)))) as fuse:
                replay.recompute_depth(Path(directory), [frame], .35, profile)
                self.assertEqual(fuse.call_args.kwargs['confidence'], .35)
                self.assertEqual(fuse.call_args.kwargs['other_boxes'], [(0,0,20,20)])
                self.assertEqual(frame['people'][0]['joints3d'][0], [1.,1.,1.])
                frame['depth_matched'] = False
                replay.recompute_depth(Path(directory), [frame], .35, profile)
                self.assertEqual(frame['people'][0]['joints3d'][0], [None,None,None])

    def test_bottom_disabled_and_boundary(self):
        self.assertFalse(replay.bottom_clipped((0,0,100,480), 0))
        self.assertFalse(replay.bottom_clipped((0,0,100,471), 8))
        self.assertTrue(replay.bottom_clipped((0,0,100,472), 8))
        for value in (-1, 480, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                replay.bottom_clipped((0,0,100,480), value)

    def test_combination_retains_continued_and_maximum(self):
        pose = [dict(t=1., id='1:1', phase=2, state='FALLEN')]
        height = [dict(t=1., id='1:1', phase=0, state='UNKNOWN'),
                  dict(t=2., id='1:2', phase=1, state='LYING')]
        rows = replay.combine_timelines(pose, height)
        self.assertEqual([r['phase'] for r in rows], [2,1])
        self.assertEqual(rows[1]['id'], '1:2')
        self.assertEqual(pose[0]['phase'], 2)

    def test_recomputed_state_controls_height_veto(self):
        frames = [dict(time_s=0., frame=1, people=[dict(id='1:1', box=(50,30,150,300), state='STANDING')])]
        with patch.dict(replay.VARIANT, {'veto2d': True}, clear=True), patch.object(replay.HeightFallTracker, 'update', return_value=(0,'')) as update:
            replay.replay_height(frames, {}, postures={(0.,'1:1'): 'LYING'})
            self.assertFalse(update.call_args.kwargs['upright_hint'])
            replay.replay_height(frames, {})
            self.assertTrue(update.call_args.kwargs['upright_hint'])
            with self.assertRaises(KeyError):
                replay.replay_height(frames, {}, postures={})

    def test_explicit_profile_does_not_force_confirmation(self):
        cfg, spatial = replay.load_configs(replay.ROOT/'ros2_ws/src/perception_bringup/config/pose_fixed_27cm_demo.yaml')
        self.assertFalse(cfg.upright_confirmed)
        self.assertEqual(cfg.confidence, .5)
        self.assertEqual(spatial['up_source'], 'static')

    def test_bottom_guard_withholds_low_evidence(self):
        frames = [dict(time_s=0., frame=1, people=[dict(id='1:1', box=(50,30,150,478), state='UNKNOWN')])]
        with patch.dict(replay.VARIANT, {}, clear=True), patch.object(replay.HeightFallTracker, 'update', return_value=(0,'')) as update:
            replay.replay_height(frames, {}, bottom_edge_px=0)
            self.assertFalse(update.call_args.kwargs['clipped'])
            replay.replay_height(frames, {}, bottom_edge_px=8)
            self.assertTrue(update.call_args.kwargs['clipped'])


if __name__ == '__main__':
    unittest.main()
