"""LoadState — 1분 load / 코어 수 (os.getloadavg — /proc/loadavg). 코어당 fielddebug.LOAD_RED 넘으면 빨강."""
from __future__ import annotations

from ...fielddebug import LOAD_RED, load_ratio
from ...terminal import GREEN, RED
from .state import State


class LoadState(State):
    def text(self) -> str:
        lr = load_ratio()
        if not lr:
            return ''
        load, cores = lr
        return '부하 ' + self.term.paint(f'{load:.1f}/{cores}', RED if load / cores > LOAD_RED else GREEN)
