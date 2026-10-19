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

    def test_forget_owner_removes_gallery_and_pending_links(self):
        old=self.register();self.m.pin(old)
        self.m.pending['0:99']=(old,1)
        self.m.forget(old)
        self.assertNotIn(old,self.m.galleries)
        self.assertFalse(self.m.pinned or self.m.bindings or self.m.pending)
        self.assertFalse(self.m.excluded or self.m.last_identity or self.m.swap_votes)
        new=self.register('0:8')
        self.assertNotEqual(old,new)

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


def near(feature, d):
    """Unit vector at cosine distance d from feature (in the A/B plane)."""
    other=B if feature is A else A
    return (1-d)*feature+np.sqrt(1-(1-d)**2)*other


class ReIDRefinementTests(unittest.TestCase):
    """Optional refinements from the 10-11 recording evaluation; synthetic descriptors only."""
    def setUp(self):self.t=10.
    def make(self,**kw):self.m=IdentityManager(IdentityConfig(**kw))
    def frame(self,observations,overlapped=(),dt=.2):
        self.t+=dt
        return {r.track_id:r for r in self.m.update(self.t,observations,overlapped)}

    def test_co_visible_track_never_takes_identity_of_its_neighbour(self):
        for enabled,expected in ((False,'REASSOCIATED'),(True,'AMBIGUOUS')):
            self.make(co_visible_exclusion=enabled)
            for _ in range(3):identity=self.frame({'0:7':A})['0:7'].person_id
            self.frame({'0:7':A,'0:8':None})  # someone else seen next to the owner
            for _ in range(3):r=self.frame({'0:8':A})['0:8']  # owner gone, look-alike remains
            self.assertEqual(r.state,expected)
            self.assertEqual(r.person_id,identity if not enabled else '')

    def test_exclusion_cleared_by_repeated_swap_evidence_so_both_reassociate(self):
        self.make(co_visible_exclusion=True)
        for _ in range(3):r=self.frame({'0:7':A,'0:8':B})
        ids={t:x.person_id for t,x in r.items()}
        for _ in range(4):  # 3 frames of swap evidence overlap the 3-frame re-confirmation
            r=self.frame({'0:7':B,'0:8':A})
            self.assertFalse(any(x.person_id for x in r.values()))
        r=self.frame({'0:7':B,'0:8':A})
        self.assertEqual(r['0:7'].person_id,ids['0:8'])
        self.assertEqual(r['0:8'].person_id,ids['0:7'])

    def test_single_swap_like_frame_or_departed_holder_keeps_exclusion(self):
        self.make(co_visible_exclusion=True)
        for _ in range(3):self.frame({'0:7':A,'0:8':B})
        self.frame({'0:7':B,'0:8':A})  # one noisy crossing frame
        for _ in range(4):self.frame({'0:7':A,'0:8':B})
        r=self.frame({'0:7':A,'0:8':B})
        self.assertEqual({x.state for x in r.values()},{'VERIFIED'})
        self.assertIn(r['0:7'].person_id,self.m.excluded['0:8'])
        for _ in range(5):r=self.frame({'0:8':A})  # holder 0:7 left; look-alike stays
        self.assertFalse(r['0:8'].person_id)

    def test_brief_track_miss_keeps_exclusion(self):
        self.make(co_visible_exclusion=True)
        for _ in range(3):identity=self.frame({'0:7':A,'0:8':B})['0:7'].person_id
        self.frame({'0:7':A})  # 0:8 missed for one sample
        self.assertIn(identity,self.m.excluded['0:8'])
        self.t+=1.5;self.frame({'0:7':A})
        self.assertNotIn('0:8',self.m.excluded)

    def test_verify_distance_keeps_bound_track_but_not_new_association(self):
        self.make(verify_distance=.3)
        for _ in range(3):identity=self.frame({'0:7':A})['0:7'].person_id
        r=self.frame({'0:7':near(A,.28)})['0:7']
        self.assertEqual((r.state,r.person_id),('VERIFIED',identity))
        self.frame({})
        for _ in range(4):r=self.frame({'0:9':near(A,.28)})['0:9']
        self.assertFalse(r.person_id)
        self.assertNotEqual(r.state,'VERIFIED')
        self.make()
        for _ in range(3):self.frame({'0:7':A})
        self.assertFalse(self.frame({'0:7':near(A,.28)})['0:7'].person_id)

    def test_confirmation_counts_samples_by_time(self):
        self.make(confirm_min_interval_s=.15)
        states=[self.frame({'0:7':A},dt=.05)['0:7'].state for _ in range(7)]
        self.assertEqual(states,['PENDING']*6+['REGISTERED'])

    def test_only_overlap_forces_reconfirmation(self):
        self.make(reconfirm_on_overlap_only=True)
        for _ in range(3):self.frame({'0:7':A})
        self.frame({'0:7':None})
        self.assertEqual(self.frame({'0:7':A})['0:7'].state,'VERIFIED')
        self.frame({'0:7':None},overlapped={'0:7'})
        self.assertEqual(self.frame({'0:7':A})['0:7'].state,'PENDING')

    def test_refinement_config_validated(self):
        for kw in ({'verify_distance':.2},{'verify_distance':.5},{'confirm_min_interval_s':-1.},
                   {'co_visible_exclusion':1},{'reconfirm_on_overlap_only':'yes'}):
            with self.assertRaises(ValueError):IdentityConfig(**kw)


class OwnerPinTests(unittest.TestCase):
    def setUp(self):
        self.m = IdentityManager(IdentityConfig(owner_views=4)); self.t = 10.

    def frame(self, obs, overlapped=(), focus=None, dt=.2):
        self.t += dt
        return {r.track_id: r for r in self.m.update(self.t, obs, overlapped, focus)}

    def test_pinned_owner_survives_retention_and_others_expire(self):
        for _ in range(3): owner = self.frame({'0:1': A, '0:2': B})['0:1'].person_id
        self.assertTrue(self.m.pin(owner)); self.assertFalse(self.m.pin('nobody'))
        self.frame({}, dt=40.)
        self.assertEqual(list(self.m.galleries), [owner])

    def test_views_only_from_followed_unoccluded_verified_owner(self):
        for _ in range(3): owner = self.frame({'0:1': A})['0:1'].person_id
        self.m.pin(owner)
        side = near(A, .2)
        self.assertEqual(self.frame({'0:1': side})['0:1'].state, 'VERIFIED')
        self.assertEqual(len(self.m.galleries[owner].views), 0)  # not the followed track
        self.frame({'0:1': side}, overlapped={'0:1'}, focus='0:1')
        self.assertEqual(len(self.m.galleries[owner].views), 0)  # occluded
        self.frame({'0:1': side}, focus='0:1')
        self.assertEqual(len(self.m.galleries[owner].views), 1)
        self.frame({'0:1': side}, focus='0:1')
        self.assertEqual(len(self.m.galleries[owner].views), 1)  # not novel
        self.m.unpin(); self.assertEqual(len(self.m.galleries[owner].views), 0)

    def test_extra_view_extends_recognition_to_that_viewpoint(self):
        for _ in range(3): owner = self.frame({'0:1': A})['0:1'].person_id
        self.m.pin(owner)
        turned = near(A, .24)
        self.frame({'0:1': turned}, focus='0:1')
        far = near(A, .3)  # beyond match distance from the registration, close to the stored view
        self.assertLess(self.m.galleries[owner].score(unit_(far)), .25)


def unit_(v):
    return v/np.linalg.norm(v)


class OwnerContinuityVerifyTests(unittest.TestCase):
    def setup_owner(self, enabled=True):
        manager = IdentityManager(IdentityConfig(owner_continuity_verify=enabled,
            verify_distance=.30, owner_views=4))
        for stamp in (10., 10.2, 10.4):
            identity = manager.update(stamp, {'0:1': A})[0].person_id
        manager.pin(identity)
        manager.suspend()
        return manager, identity

    def test_focused_owner_reconfirms_then_learns_view(self):
        manager, owner = self.setup_owner()
        side = near(A, .27)
        for i, stamp in enumerate((11., 11.2, 11.4)):
            row = manager.update(stamp, {'0:1': side}, focus='0:1')[0]
            self.assertEqual(bool(row.person_id), i == 2)
        self.assertEqual(row.person_id, owner)
        self.assertEqual(len(manager.galleries[owner].views), 0)
        row = manager.update(11.6, {'0:1': side}, focus='0:1')[0]
        self.assertEqual(row.state, 'VERIFIED')
        self.assertEqual(len(manager.galleries[owner].views), 1)
        # The learned viewpoint may later identify a new track with normal search gates.
        manager.suspend()
        for stamp in (12., 12.2, 12.4):
            row = manager.update(stamp, {'0:9': side})[0]
        self.assertEqual(row.person_id, owner)

    def test_no_relaxation_without_current_focus_or_when_occluded(self):
        for enabled, focus, track, overlapped in (
            (False, '0:1', '0:1', ()), (True, None, '0:1', ()),
            (True, '0:1', '0:9', ()), (True, '0:1', '0:1', ('0:1',))):
            manager, owner = self.setup_owner(enabled)
            for stamp in (11., 11.2, 11.4, 11.6):
                row = manager.update(stamp, {track: near(A, .27)}, overlapped, focus)[0]
                self.assertFalse(row.person_id)
            self.assertFalse(manager.galleries[owner].views)

    def test_competitor_or_distant_descriptor_still_rejected(self):
        for observations in ({'0:1': near(A, .27), '0:2': near(A, .27)},
                             {'0:1': near(A, .34)}):
            manager, owner = self.setup_owner()
            for stamp in (11., 11.2, 11.4):
                rows = manager.update(stamp, observations, focus='0:1')
                self.assertTrue(all(not row.person_id for row in rows))
            self.assertFalse(manager.galleries[owner].views)

    def test_option_requires_boolean(self):
        with self.assertRaises(ValueError):
            IdentityConfig(owner_continuity_verify=1)
