"""TaskState — 임무(/task/status 요약) 칸."""
from __future__ import annotations

from .state import State


class TaskState(State):
    def text(self) -> str:
        task = self.state.task_status or '-'
        return '임무 ' + (task if len(task) <= 38 else task[:37] + '…')
