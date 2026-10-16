import pytest
from services.inference_gateway.backends.tongue import TongueBackend, PIPELINE_ID, DIGEST, LABELS


@pytest.mark.parametrize('expected,actual', [('cuda','cuda'),('cuda','cuda:0'),('cuda:0','cuda'),('cpu','cpu')])
def test_expected_device_matches_live_service(monkeypatch,expected,actual):
    def request(self,method,path,**kwargs):
        if path=='/v1/model-info':
            return dict(pipeline_id=PIPELINE_ID,pipeline_digest=DIGEST,labels=list(LABELS),device=actual)
        return dict(status='ready')
    monkeypatch.setattr(TongueBackend,'request',request)
    assert TongueBackend(dict(device=expected)).info()['device']==actual


@pytest.mark.parametrize('expected,actual', [('cuda','cpu'),('cpu','cuda:0'),('cuda',None)])
def test_wrong_device_is_not_a_silent_fallback(monkeypatch,expected,actual):
    monkeypatch.setattr(TongueBackend,'request',lambda *args,**kwargs:
        dict(pipeline_id=PIPELINE_ID,pipeline_digest=DIGEST,labels=list(LABELS),device=actual))
    with pytest.raises(ValueError,match='tongue_device_mismatch'):
        TongueBackend(dict(device=expected))
