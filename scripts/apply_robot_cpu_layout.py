#!/usr/bin/env python3
"""Pin only descendants of one launcher on the six-core Jetson during startup."""
import argparse
import json
import os
from pathlib import Path
import time


def read_process(pid):
    fields=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
    return int(fields[1]),fields[19]


def belongs_to(pid,parent):
    seen=set()
    while pid not in seen and pid>1:
        if pid==parent:return True
        seen.add(pid)
        try:pid,_=read_process(pid)
        except (OSError,ValueError,IndexError):return False
    return False


def cores_for(names):
    if 'emergency_node' in names:return {4}
    if names & {'audio_node','guard'}:return {5}
    if names & {'tts_node','asr_node','reid_observer'}:return {0,1}
    if names & {'pose_tracker','gemini_registration'}:return {2,3}
    return {0,1,2,3}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--parent',type=int,required=True)
    p.add_argument('--seconds',type=float,default=60.)
    args=p.parse_args()
    if not hasattr(os,'sched_setaffinity') or not Path('/proc').exists():
        raise RuntimeError('CPU layout requires Linux')
    if not {0,1,2,3,4,5}.issubset(os.sched_getaffinity(0)):
        raise RuntimeError('CPU layout requires accessible Jetson CPUs 0–5')
    _,birth=read_process(args.parent)
    seen=set();end=time.monotonic()+args.seconds
    while time.monotonic()<end:
        try:
            if read_process(args.parent)[1]!=birth:return
        except OSError:return
        for proc in Path('/proc').glob('[0-9]*'):
            pid=int(proc.name)
            if pid in (args.parent,os.getpid()) or not belongs_to(pid,args.parent):continue
            try:
                names={Path(a.decode(errors='replace')).name for a in (proc/'cmdline').read_bytes().split(b'\0') if a}
                cores=cores_for(names)
                for thread in (proc/'task').iterdir():
                    key=(pid,int(thread.name),tuple(sorted(cores)))
                    if key not in seen:
                        os.sched_setaffinity(int(thread.name),cores);seen.add(key)
                if (pid,'logged') not in seen:
                    print(json.dumps(dict(pid=pid,cores=sorted(cores),name=(proc/'comm').read_text().strip())),flush=True)
                    seen.add((pid,'logged'))
            except (OSError,ValueError):pass
        time.sleep(.2)

if __name__=='__main__':main()
