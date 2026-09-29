import ast,unittest
from pathlib import Path
from collections import deque
import numpy as np
from types import SimpleNamespace as NS
source=(Path(__file__).resolve().parents[1] / 'offline_voice/asr_node.py').read_text()
method=next(n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef) and n.name=='_listen_once')
class Tests(unittest.TestCase):
 def run_case(self, frames, speaking_at=None):
  clock=[0]; published=[]; captured=[]; closed=[]
  class Capture:
   def __init__(self,*a):pass
   def start(self):pass
   def read(self,n):
    clock[0]+=0.04
    if speaking_at and clock[0]>=speaking_at: node._speaking=True
    return np.full(640,next(frames,0),dtype='<i2').tobytes()
   def close(self):closed.append(True)
  stream=NS(accept_waveform=lambda sr,x:captured.extend(x),result=NS(text='前进'))
  cfg=dict(sample_rate=16000,frame_ms=40,energy_threshold=2800,speech_onset_ms=80,silence_ms=120,speech_timeout_sec=.2,max_utterance_sec=2,capture_device='test')
  node=NS(get_parameter=lambda k:NS(value=cfg[k]),_recognizer=NS(create_stream=lambda:stream,decode_stream=lambda s:None),_backend='sensevoice',_stop=NS(is_set=lambda:False),_speaking=False,_publish_state=lambda s:None,get_logger=lambda:NS(info=lambda s:None),_text_pub=NS(publish=lambda m:published.append(m.data)))
  env=dict(deque=deque,np=np,time=NS(monotonic=lambda:clock[0]),ArecordCapture=Capture,pcm_rms_s16le=lambda p:abs(float(np.frombuffer(p,dtype='<i2')[0])),String=lambda **kw:NS(**kw))
  exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_asr','exec'),env)
  env['_listen_once'](node,'manual');self.assertTrue(closed)
  return published,captured
 def test_onset_crosses_deadline(self):
  result,audio=self.run_case(iter([0,0,0,0,4000,4000,4000,0,0,0]));self.assertEqual(result,['前进']);self.assertEqual(sum(x!=0 for x in audio),1920)
 def test_tts_discards_partial_command(self):
  result,_=self.run_case(iter([4000]*8),.12);self.assertEqual(result,[])
 def test_silence_no_command(self):
  result,_=self.run_case(iter([]));self.assertEqual(result,[])
if __name__ == '__main__':
 unittest.main()
