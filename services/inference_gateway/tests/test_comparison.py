from copy import deepcopy
import pytest
from services.inference_gateway.tools.compare_tongue import compare


def fixture():
    return dict(pipeline_digest='frozen',pipeline_id='debug',samples=[dict(image_sha256='image',
        status='ok',classification_performed=True,pipeline_labels={},
        labels={'Crack':dict(score=.25,threshold=.5,decision='negative')})])


def test_numerical_deviation_and_changed_decision_refused():
    cpu=fixture();gpu=deepcopy(cpu)
    gpu['samples'][0]['labels']['Crack']['score']+=.0005
    assert compare(cpu,gpu)['status']=='PASS'
    gpu['samples'][0]['labels']['Crack']['score']+=.002
    assert compare(cpu,gpu)['status']=='FAIL'
    gpu=deepcopy(cpu);gpu['samples'][0]['labels']['Crack']['decision']='positive'
    assert compare(cpu,gpu)['status']=='FAIL'
    gpu=deepcopy(cpu);gpu['samples'][0]['labels']['Crack']['threshold']=.4
    assert compare(cpu,gpu)['status']=='FAIL'


def test_wrong_input_or_pipeline_refused():
    cpu=fixture();gpu=deepcopy(cpu);gpu['samples'][0]['image_sha256']='other'
    with pytest.raises(ValueError,match='input_mismatch'):compare(cpu,gpu)
    gpu=deepcopy(cpu);gpu['pipeline_digest']='new'
    with pytest.raises(ValueError,match='pipeline_mismatch'):compare(cpu,gpu)
