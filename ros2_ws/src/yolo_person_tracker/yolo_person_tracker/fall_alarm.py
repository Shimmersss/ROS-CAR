"""Per-frame fall alarm over detected and continued people (pose_node publishes fall_alarm).

One summary per RGB observation, so consumers need neither to merge person_states with
person_states_continued nor to time-match them. A fall handed from a continued person back to
a returning detection keeps its stage, so the alarm has no new rising edge."""
from dataclasses import dataclass
from typing import Any


@dataclass
class AlarmSummary:
    stage: int = 0
    person: Any = None          # the person carrying the stage, None when stage is 0
    continued: bool = False     # that person came from person_states_continued
    suspected: int = 0
    confirmed: int = 0


def summarize(detected, continued):
    """detected/continued: people with fall_stage (0/1/2), track_id and box.

    The highest stage wins; on a tie a detected person (current evidence) is preferred over a
    continued one, then list order."""
    out = AlarmSummary()
    for is_continued, people in ((False, detected), (True, continued)):
        for person in people:
            stage = int(person.fall_stage)
            out.suspected += stage == 1
            out.confirmed += stage == 2
            if stage > out.stage:
                out.stage, out.person, out.continued = stage, person, is_continued
    return out
