"""CPU pinning must not escape the selected launcher process tree."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('robot_cpu_layout',Path(__file__).resolve().parents[1]/'scripts/apply_robot_cpu_layout.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_only_descendants_of_the_selected_launcher_match():
    tree={100:(1,'parent'),101:(100,'child'),103:(101,'grandchild'),200:(1,'external'),201:(200,'external-child'),300:(301,'cycle'),301:(300,'cycle')}
    def read(pid):
        if pid not in tree:raise FileNotFoundError()
        return tree[pid]
    with patch.object(module,'read_process',read):
        assert module.belongs_to(103,100)
        assert module.belongs_to(101,100)
        assert not module.belongs_to(201,100)
        assert not module.belongs_to(300,100)
        assert not module.belongs_to(999,100)


def test_emergency_capture_and_guard_have_reserved_cores():
    assert module.cores_for({'python3','emergency_node'})=={4}
    assert module.cores_for({'audio_node'})=={5}
    assert module.cores_for({'guard'})=={5}
    assert module.cores_for({'tts_node'})=={0,1}
    assert module.cores_for({'asr_manager'})=={0,1}
    assert module.cores_for({'python3','offline_voice.asr_worker'})=={0,1}
    assert module.cores_for({'reid_observer'})=={0,1}
    assert module.cores_for({'pose_tracker'})=={2,3}
    assert module.cores_for({'other'})=={0,1,2,3}
