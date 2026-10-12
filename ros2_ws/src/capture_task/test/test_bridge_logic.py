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
