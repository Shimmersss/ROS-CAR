from copy import deepcopy
import pytest
from capture_task.catalog import Catalog, speech_text


@pytest.mark.parametrize('text', ['', '字'*301, '请停。止', '请急 停', '不要动', '退出语音', '解\u200b除紧急制动'])
def test_unsafe_text(text):
    with pytest.raises(ValueError):
        speech_text(text)


def test_catalog_overrides(catalog):
    assert set(catalog.tasks) == {'face','tongue','helmet','pcb'}
    assert catalog.resolve('pcb', {'prepare_sec':0})['min_width'] == 1280
    for params in ({'burst':1}, {'announce':'停车'}, {'prepare_sec':True}, {'prepare_sec':float('nan')}, {'prepare_sec':31}):
        with pytest.raises(ValueError):
            catalog.resolve('face', params)
    assert catalog.tasks['face']['prepare_sec'] == 3.


@pytest.mark.parametrize('key,value', [('burst',0), ('burst',True), ('min_width',1.2), ('upload','none'), ('require_hold',1)])
def test_catalog_bad_values(catalog,key,value):
    data = deepcopy(catalog.data)
    data['tasks']['face'][key] = value
    with pytest.raises(ValueError):
        Catalog(data)
