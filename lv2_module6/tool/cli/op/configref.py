"""config yaml 참조 — 노드별 기본값 읽기.

ros2_ws/src/*/config/*.yaml 를 뒤져 최상위 키(= ROS 노드 이름)로
노드 → (패키지, 파일) 매핑을 만든다. launch 인자로 덮어쓰기 전 기본값 확인·복원용.
"""
from __future__ import annotations

from pathlib import Path

try:
    import yaml as _yaml
except ImportError:  # ROS 없는 최소 환경 — 원문 보기만
    _yaml = None

_CANDIDATE_SRC = [
    Path(__file__).resolve().parent.parent.parent / 'ros2_ws' / 'src',  # tool/cli/../../ros2_ws/src
    Path('/ws/src'),                                                     # 컨테이너
]


def src_dir() -> Path | None:
    for p in _CANDIDATE_SRC:
        if p.is_dir():
            return p
    return None


def _load_yaml(path: Path) -> dict:
    if _yaml is None:
        raise RuntimeError('PyYAML 없음 (ROS 환경에서 실행하세요) — 원문 보기로 대체됩니다')
    with open(path, encoding='utf-8') as f:
        data = _yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def node_map() -> dict[str, tuple[str, Path]]:
    """{노드 이름: (패키지, yaml 경로)} — 모든 config yaml 스캔."""
    src = src_dir()
    if src is None or _yaml is None:
        return {}
    out: dict[str, tuple[str, Path]] = {}
    for cfg in sorted(src.glob('*/config/*.yaml')):
        try:
            data = _load_yaml(cfg)
        except Exception:
            continue
        for key, val in data.items():
            if isinstance(val, dict) and key not in out:
                out[key] = (cfg.parent.parent.name, cfg)
    return out


def node_defaults(node: str) -> tuple[str | None, dict]:
    """(파일경로문자열, {파라미터: 기본값}) — ros__parameters 밑만."""
    entry = node_map().get(node)
    if entry is None:
        return None, {}
    _, path = entry
    try:
        data = _load_yaml(path)
    except Exception as e:
        return str(path), {'<읽기 실패>': str(e)}
    section = (data.get(node) or {})
    params = section.get('ros__parameters', {}) if isinstance(section, dict) else {}
    flat: dict = {}
    if isinstance(params, dict):
        for k, v in params.items():
            flat[k] = v
    return str(path), flat


def raw_text(node: str) -> tuple[str | None, str]:
    """yaml 라이브러리 없을 때 원문 보기용 — 노드 섹션이 있는 파일 전체."""
    if _yaml is not None:
        entry = node_map().get(node)
        if entry is None:
            return None, ''
        try:
            return str(entry[1]), entry[1].read_text(encoding='utf-8')
        except Exception as e:
            return str(entry[1]), f'<읽기 실패: {e}>'
    src = src_dir()
    if src is None:
        return None, ''
    for cfg in sorted(src.glob('*/config/*.yaml')):
        try:
            text = cfg.read_text(encoding='utf-8')
        except Exception:
            continue
        if any(l.startswith(node + ':') for l in text.splitlines()):
            return str(cfg), text
    return None, ''


def has_yaml() -> bool:
    return _yaml is not None
