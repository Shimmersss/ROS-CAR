import json
import re
from types import SimpleNamespace
import pytest
from capture_task.api_logic import request_goal,service_timeout,action_result,rejection_result,response_fields


def request(**values):
    defaults=dict(task_id='',announce='',prepare_sec=-1.,timeout_sec=0.);defaults.update(values)
    return SimpleNamespace(**defaults)


def test_mapping_defaults_and_unique_ids(catalog):
    first=request_goal('face',request(),catalog);second=request_goal('face',request(),catalog)
    assert re.fullmatch(r'ros-[a-f0-9]{32}',first['task_id']) and first['task_id']!=second['task_id']
    assert first['source']=='ros_api' and first['task_type']=='face'
    assert json.loads(first['params_json'])=={} and first['ttl_sec']==0.


def test_supplied_id_and_safe_overrides(catalog):
    req=SimpleNamespace(task_id='provided_ID-1',announce='请面向镜头。',prepare_sec=30.,timeout_sec=600.)
    goal=request_goal('tongue',req,catalog)
    assert goal['task_id']==req.task_id
    assert json.loads(goal['params_json'])=={'announce':req.announce,'prepare_sec':30.}
    assert goal['ttl_sec']==600.


@pytest.mark.parametrize('field,value',[
    ('task_id','../wrong'),('task_id','x'*65),('task_id',None),('announce','现在停车'),
    ('announce','停，车'),('announce',' '),('announce','a'*301),
    ('prepare_sec',30.01),('prepare_sec',float('nan')),('prepare_sec',float('-inf')),('prepare_sec',True),
    ('timeout_sec',-.01),('timeout_sec',600.01),('timeout_sec',float('inf')),('timeout_sec',True)])
def test_invalid_never_maps_to_goal(catalog,field,value):
    req=request();setattr(req,field,value)
    with pytest.raises(ValueError):request_goal('pcb',req,catalog)


@pytest.mark.parametrize('prepare',[-100.,-.001,0.,30.])
def test_prepare_sentinel_and_bounds(catalog,prepare):
    req=request();req.prepare_sec=prepare
    params=json.loads(request_goal('helmet',req,catalog)['params_json'])
    assert ('prepare_sec' in params)==(prepare>=0)


@pytest.mark.parametrize('timeout',[0.,.001,600.])
def test_timeout_budget(catalog,timeout):
    req=request();req.timeout_sec=timeout
    assert request_goal('face',req,catalog)['ttl_sec']==timeout
    assert service_timeout(timeout,catalog.defaults)==(timeout or catalog.defaults['task_timeout_sec'])+catalog.defaults['speech_timeout_sec']+5


@pytest.mark.parametrize('status',['uploaded','failed','cancelled'])
def test_result_preserves_executor_fields(status):
    value=SimpleNamespace(status=status,detail='reason',capture_id='cap',upload_json='[]')
    result=action_result('ros-task',value)
    assert result==dict(success=status=='uploaded',status=status,detail='reason',task_id='ros-task',capture_id='cap',upload_json='[]')


@pytest.mark.parametrize('reason,expected',[('busy','busy'),('voice_mode_active','rejected'),('invalid','rejected'),('rejected','rejected')])
def test_rejections(reason,expected):
    value=rejection_result('t',reason)
    assert not value['success'] and value['status']==expected and value['detail']==reason


def test_only_uploaded_success_and_catalog_future_task(catalog):
    for status in ('failed','cancelled','busy','timeout','invalid','unavailable','rejected'):
        assert not response_fields('t',status)['success']
    catalog.tasks['inspection']=catalog.tasks['face']
    assert request_goal('inspection',request(),catalog)['task_type']=='inspection'
