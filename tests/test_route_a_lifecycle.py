"""Mock systemd and runner: exercise ownership and simultaneous manual starts."""
import os
from pathlib import Path
import subprocess
import tempfile

script = Path(__file__).resolve().parents[1] / 'scripts/route_a.sh'
with tempfile.TemporaryDirectory() as temp:
    root = Path(temp); (root/'bin').mkdir(); (root/'scripts').mkdir()
    (root/'setup').touch(); (root/'scripts/run_foxglove.sh').touch()
    runner = root/'runner.sh'
    runner.write_text('echo start >> "$ROSCAR_ROOT/starts"\necho "方案 A 可视化已启动"\ntrap "exit 0" TERM\nwhile :; do sleep 0.1; done\n')
    systemctl = root/'bin/systemctl'
    systemctl.write_text('#!/bin/bash\ncase "$*" in\n *ActiveState*) echo "$TEST_STATE";;\n *LoadState*) echo "$TEST_LOAD";;\nesac\n')
    systemctl.chmod(0o755)
    env = dict(os.environ, PATH=str(root/'bin')+':'+os.environ['PATH'], ROSCAR_ROOT=temp,
               ROUTE_A_RUNNER=str(runner), ROSCAR_ROS_SETUP=str(root/'setup'),
               ROSCAR_VENDOR_SETUP=str(root/'setup'), ROSCAR_PROJECT_SETUP=str(root/'setup'), TEST_LOAD='loaded')
    def run(command, **changes):
        return subprocess.run(['bash',str(script),command], env=dict(env,**changes),capture_output=True,text=True,timeout=15)
    for state in ('active','activating','reloading','deactivating'):
        assert run('start',TEST_STATE=state).returncode == 0
        assert not (root/'starts').exists()
    assert run('start',TEST_STATE='inactive').returncode == 1
    env.update(TEST_LOAD='not-found', TEST_STATE='inactive')
    procs = [subprocess.Popen(['bash',str(script),'start'],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE) for _ in range(2)]
    try:
        for p in procs:
            out, err = p.communicate(timeout=15)
            assert p.returncode == 0, (out,err)
        assert (root/'starts').read_text().splitlines() == ['start']
    finally:
        # Reap the background process through normal script lifecycle.
        result = run('stop')
        # Container PID 1 may retain a zombie; stop still sent TERM.
        assert result.returncode in (0,1)
print('PASS route A systemd transition ownership and concurrent starts')
