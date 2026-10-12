"""Deterministic identity lifecycle tests; descriptors here are synthetic, not model accuracy."""
import unittest
import numpy as np
from yolo_person_tracker.reid import IdentityManager, IdentityConfig

A=np.array([1.,0.,0.]);B=np.array([0.,1.,0.]);C=np.array([0.,0.,1.])

class ReIDTests(unittest.TestCase):
    def setUp(self):self.m=IdentityManager();self.t=10.
    def frame(self, observations):
        self.t+=.2
        return self.m.update(self.t,observations)
    def register(self,track='0:7',feature=A):
        for _ in range(3):r=self.frame({track:feature})
        self.assertEqual(r[0].state,'REGISTERED');return r[0].person_id

    def test_registration_needs_distinct_consistent_observations(self):
        self.assertEqual(self.frame({'0:7':A})[0].state,'PENDING')
        self.assertEqual(self.frame({'0:7':B})[0].state,'PENDING')
        self.assertEqual(self.frame({'0:7':A})[0].state,'PENDING')
        self.assertFalse(self.m.galleries)

    def test_occlusion_reacquires_new_track_same_identity(self):
        identity=self.register();self.frame({})
        for i in range(3):
            r=self.frame({'0:19':A})[0]
            self.assertEqual(bool(r.person_id),i==2)
        self.assertEqual(r.person_id,identity);self.assertEqual(r.state,'REASSOCIATED')

    def test_stable_track_is_rechecked_and_switch_rejected(self):
        identity=self.register();anchor=self.m.galleries[identity].anchor.copy()
        r=self.frame({'0:7':B})[0]
        self.assertFalse(r.person_id);self.assertNotIn('0:7',self.m.bindings)
        np.testing.assert_array_equal(anchor,self.m.galleries[identity].anchor)

    def test_identical_candidates_compete_in_both_directions(self):
        identity=self.register();self.frame({})
        r=self.frame({'0:19':A,'0:20':A})
        self.assertTrue(all(x.state=='AMBIGUOUS' and not x.person_id for x in r))
        self.assertEqual(len(self.m.galleries),1)

    def test_visible_identity_cannot_be_stolen(self):
        identity=self.register()
        r=self.frame({'0:7':None,'0:9':A})
        self.assertFalse(any(x.person_id for x in r))
        self.assertEqual(self.m.galleries[identity].last_seen,self.t-.2)

    def test_bad_quality_freezes_gallery_and_requires_reconfirmation(self):
        identity=self.register();count=len(self.m.galleries[identity].features)
        self.frame({'0:7':None})
        self.assertEqual(len(self.m.galleries[identity].features),count)
        self.assertEqual(self.frame({'0:7':A})[0].state,'PENDING')

    def test_simultaneous_distinct_people_never_share_identity(self):
        for _ in range(3):r=self.frame({'0:7':A,'0:8':B})
        self.assertEqual(len({x.person_id for x in r}),2)
        self.assertTrue(all(x.person_id for x in r))

    def test_simultaneous_similar_new_people_stay_unknown(self):
        for _ in range(5):r=self.frame({'0:7':A,'0:8':A})
        self.assertFalse(self.m.galleries)
        self.assertTrue(all(x.state=='AMBIGUOUS' for x in r))

    def test_crossing_silent_switch_can_reassociate_both_after_confirmation(self):
        for _ in range(3):r=self.frame({'0:7':A,'0:8':B})
        identities={x.track_id:x.person_id for x in r}
        for _ in range(3):r=self.frame({'0:7':B,'0:8':A})
        self.assertEqual(r[0].person_id,identities['0:8'])
        self.assertEqual(r[1].person_id,identities['0:7'])

    def test_capacity_and_retention_are_bounded(self):
        self.m=IdentityManager(IdentityConfig(max_identities=2))
        self.register();self.register('0:8',B)
        for _ in range(3):r=self.frame({'0:9':C})
        self.assertFalse(r[0].person_id);self.assertEqual(len(self.m.galleries),2)
        self.t+=31;self.frame({});self.assertFalse(self.m.galleries)

    def test_epoch_new_id_requires_appearance_not_numeric_track_id(self):
        identity=self.register()
        self.assertFalse(self.frame({'1:7':B})[0].person_id)
        self.assertFalse(self.frame({'2:7':A})[0].person_id)
        self.frame({'2:7':A});r=self.frame({'2:7':A})[0]
        self.assertEqual(r.person_id,identity)

    def test_time_reversal_never_reuses_old_identity(self):
        identity=self.register();self.t=1.
        new=self.register();self.assertNotEqual(new,identity)

    def test_stream_gap_requires_reconfirmation(self):
        self.register();self.t+=2.
        self.assertEqual(self.frame({'0:7':A})[0].state,'PENDING')

    def test_invalid_descriptor_and_config_rejected(self):
        for f in ([0.,0.],[float('nan'),1.],[[1.,0.]]):
            with self.assertRaises(ValueError):self.frame({'0:7':f})
        for kw in ({'confirm_frames':1},{'max_identities':0},{'margin':float('nan')},
                   {'match_distance':.6},{'gallery_size':2.5}):
            with self.assertRaises(ValueError):IdentityConfig(**kw)

    def test_gallery_anchor_does_not_drift(self):
        identity=self.register();original=self.m.galleries[identity].anchor.copy()
        for _ in range(30):self.frame({'0:7':np.array([.99,.01,0.])})
        np.testing.assert_array_equal(original,self.m.galleries[identity].anchor)
        self.assertLessEqual(len(self.m.galleries[identity].features),8)

    def test_gallery_row_margin_rejects_two_plausible_identities(self):
        self.m=IdentityManager(IdentityConfig(match_distance=.4,new_identity_distance=.6))
        for _ in range(3):self.frame({'0:7':A,'0:8':B})
        self.frame({})
        r=self.frame({'0:9':A+B})[0]
        self.assertEqual(r.state,'AMBIGUOUS');self.assertFalse(r.person_id)

    def test_model_dimension_change_rejected(self):
        self.register()
        with self.assertRaises(ValueError):self.frame({'0:7':np.ones(4)})

    def test_visible_occluded_owner_reserves_identity_across_multiple_frames(self):
        identity=self.register()
        for _ in range(5):
            r=self.frame({'0:7':None,'0:9':A})
            self.assertFalse(any(x.person_id for x in r))
        for _ in range(3):r=self.frame({'0:9':A})
        self.assertEqual(r[0].person_id,identity)
