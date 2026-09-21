"""Standoff goal in a common map frame."""
import math


def standoff(robot,person,distance):
    if not all(math.isfinite(v) for v in (*robot,*person,distance)) or distance<=0:
        raise ValueError('Finite positions and positive standoff required')
    dx,dy=person[0]-robot[0],person[1]-robot[1]
    length=math.hypot(dx,dy)
    if length<=distance+.1:
        return None
    return person[0]-distance*dx/length,person[1]-distance*dy/length,math.atan2(dy,dx)
