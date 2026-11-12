"""팔 관절 막대 보기 — 포커스 가능한 부품 한 개. 한 파일에 한 클래스.

  ArmView  arm_view.py  관절마다 한계 범위 막대 + 현재(●)·목표(◆) · 키로 목표만 바꿈 (이동은 페이지가 확인 후)

쓰는 곳: pages/arm/ (팔 조작 화면).
"""
from .arm_view import ArmView

__all__ = ['ArmView']
