"""노드별 추가 상태·기능의 공통 인터페이스.

`op/bus.py`는 노드 이름으로 feature를 찾아 status_lines만 붙인다.
노드별 ROS 구독·서비스 클라이언트는 각 feature의 wire()에서 만든다.
"""
from __future__ import annotations


class NodeFeature:
    """한 노드의 추가 상태/동작.

    - `node`: 노드 이름 (네임스페이스가 있으면 마지막 조각으로 매칭)
    - `wire(tool)`: OperatorTool이 뜰 때 구독·클라이언트 생성
    - `status_lines(bus, name, summary)`: `node status`와 노드 메뉴에 붙일 줄
    - `actions(app, bus, name)`: 노드 메뉴에 추가할 MenuItem (필요할 때)
    """

    node = ''

    def wire(self, tool) -> None:
        pass

    def status_lines(self, bus, name: str, summary: bool = False) -> list[str]:
        return []

    def actions(self, app, bus, name: str) -> list:
        return []
