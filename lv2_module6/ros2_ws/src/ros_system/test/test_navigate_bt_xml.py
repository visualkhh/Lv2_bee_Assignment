#!/usr/bin/env python3
"""navigate_to_pose.xml 복구 순서 시험 — ROS 없이: python3 test/test_navigate_bt_xml.py (pytest 도 됨)

확인: ① XML 문법 ② 쓰는 노드가 Nav2 노드 목록(nav2_tree_nodes.xml)·BT 기본 노드에 있음 (목록이 없는 환경이면 건너뜀)
③ 복구 RoundRobin 의 첫 번째 = [코스트맵 지우기 2개 → 멈춰 있을 때만 후진] ④ 후진은 CheckStopStatus 뒤에만, 거리 0.10 m
⑤ RoundRobin 의 직속 자식에 BackUp 이 없음 (예전처럼 4번째로 돌아가지 않음).
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

XML = Path(__file__).resolve().parents[1] / 'config' / 'navigate_to_pose.xml'
NODES = Path('/opt/ros/lyrical/share/nav2_behavior_tree/nav2_tree_nodes.xml')
BT_BUILTIN = {'Sequence', 'Fallback', 'ReactiveFallback', 'ReactiveSequence', 'AlwaysSuccess', 'AlwaysFailure', 'Inverter',
              'ForceFailure', 'ForceSuccess', 'RetryUntilSuccessful', 'KeepRunningUntilFailure', 'Script', 'SetBlackboard'}


def tree():
    return ET.parse(XML).getroot()


def recovery_round_robin(root):
    rr = [e for e in root.iter('RoundRobin') if e.get('name') == 'RecoveryActions']
    assert len(rr) == 1, '복구 RoundRobin(RecoveryActions)이 하나여야 함'
    return rr[0]


def test_nodes_exist():
    if not NODES.is_file():
        return   # Nav2 가 없는 환경 — 구조 시험만
    known = set(re.findall(r'ID="([A-Za-z]+)"', NODES.read_text())) | BT_BUILTIN
    used = {e.tag for e in tree().iter() if e.tag not in ('root', 'BehaviorTree')}
    assert used <= known, f'모르는 BT 노드: {sorted(used - known)}'


def test_backup_first_and_only_when_stopped():
    rr = recovery_round_robin(tree())
    kids = list(rr)
    assert [k.tag for k in kids] == ['Sequence', 'Spin', 'Wait'], [k.tag for k in kids]   # 후진이 직속 4번째가 아님
    first = kids[0]
    parts = list(first)
    assert [p.tag for p in parts[:2]] == ['ClearEntireCostmap', 'ClearEntireCostmap'], '코스트맵(로컬·전역)을 먼저 지움'
    fb = parts[2]
    assert fb.tag == 'Fallback' and [c.tag for c in fb] == ['Sequence', 'AlwaysSuccess'], '멈춰 있지 않으면 후진을 건너뛰고 계속'
    seq = list(fb[0])
    assert [c.tag for c in seq] == ['CheckStopStatus', 'BackUp'], '후진 앞에 CheckStopStatus'
    assert float(seq[1].get('backup_dist')) == 0.10, '후진은 0.10 m (뒤가 안 보임)'
    assert float(seq[0].get('velocity_threshold')) > 0 and int(seq[0].get('duration_stopped')) >= 500


if __name__ == '__main__':
    test_nodes_exist()
    test_backup_first_and_only_when_stopped()
    print('PASS test_navigate_bt_xml')
