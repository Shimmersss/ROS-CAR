import math
from pathlib import Path
from unittest.mock import patch, Mock
import pytest
from demo_motion.runtime import manual_command, validate_mode, ModuleOwner


def test_manual_watchdog_and_ackermann():
    values=(.2,0,0,0,0,1.)
    assert manual_command(values,.1,'base_link') == (.2,.5)
    assert manual_command((-.1,0,0,0,0,1),.1,'base_link') == (-.1,.25)
    assert manual_command((0,0,0,0,0,1),.1,'base_link') == (0.,0.)
    for age in (-.01,.21,math.nan):
        assert manual_command(values,age,'base_link') == (0.,0.)
    assert manual_command(values,0,'camera') == (0.,0.)
    for index in range(6):
        bad=list(values);bad[index]=math.nan
        assert manual_command(bad,0,'base_link') == (0.,0.)
    for index in (1,2,3,4):
        bad=list(values);bad[index]=.1
        assert manual_command(bad,0,'base_link') == (0.,0.)


def test_fixed_mode_whitelist_and_exact_target():
    assert validate_mode('FOLLOW','3:19') == 'FOLLOW'
    for mode,target in [('FOLLOW;reboot',''),('FACE','3:19'),('FOLLOW','../x'),('FOLLOW','3:4:5')]:
        with pytest.raises(ValueError):validate_mode(mode,target)


def test_cgroup_must_stop_before_another_start(tmp_path):
    (tmp_path/'scripts').mkdir();(tmp_path/'scripts/run_demo_module.sh').touch()
    owner=ModuleOwner(tmp_path)
    with patch.object(owner,'active',return_value=True),patch.object(owner,'run') as runner:
        with pytest.raises(RuntimeError):owner.start('FACE')
        runner.assert_not_called()
    with patch('demo_motion.runtime.subprocess.run',return_value=Mock(returncode=0)),patch.object(owner,'active',return_value=True):
        with pytest.raises(RuntimeError):owner.stop()
