from capture_task.bridge_logic import CommandCache, remaining_ttl


def test_dedup_expiry_capacity():
    cache=CommandCache(capacity=2)
    ack=dict(accepted=False,reason='busy')
    cache.put('a',ack,now=10)
    assert cache.get('a',now=609) == ack
    assert cache.get('a',now=610) is None
    cache.put('a',ack,now=700);cache.put('b',ack,now=701);cache.put('c',ack,now=702)
    assert cache.get('a',now=703) is None
    assert remaining_ttl({'ttl_sec':1},100,60,now=101) == 0
    assert remaining_ttl({'ttl_sec':0},100,60,now=101) == 59


def test_default_keeps_full_ten_minute_window():
    cache=CommandCache()
    for index in range(2050):
        cache.put(str(index),dict(accepted=True,reason='accepted'),now=10.)
    assert cache.get('0',now=609.)==dict(accepted=True,reason='accepted')
    assert cache.get('0',now=610.) is None


def test_poll_configuration_url_and_socket_budget():
    import pytest
    from capture_task.bridge_logic import validate_poll_wait,poll_request
    for value in (0,1,5,20):
        assert validate_poll_wait(value)==value
        assert poll_request('r',value,False)==('/v1/robots/r/commands/next?wait='+str(value),value+2.)
        assert poll_request('r',value,True)==('/v1/robots/r/commands/next?wait=0',2.)
    for value in (-1,21,True,1.,'5',None):
        with pytest.raises(ValueError):validate_poll_wait(value)
