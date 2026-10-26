"""Houdini -> Isaac Sim USD cleanup.

Houdini(Solaris)에서 export한 usda를 Isaac Sim에서 쓰기 전에 정리한다.

  1. sim/source/**/*.rat 삭제 (Houdini Mantra/Karma 텍스처 캐시, Isaac은 원본 png/exr만 씀)
  2. usda에서 Houdini 전용 메타데이터 제거
     - customData 의 HoudiniPreviewTags, HoudiniIsAutoCreatedShader 등 (STRIP_CUSTOM_DATA_KEYS)
     - 비게 된 customData / 메타데이터 괄호 ( ) 도 함께 제거
     - --strip-ui: NodeGraphNodeAPI 스키마 + ui:nodegraph:* 속성 (노드 에디터 배치 정보)
  3. asset 경로(@...@) 정리
     - 역슬래시 -> 슬래시
     - sim/ 안을 가리키는 절대경로 -> usda 기준 상대경로
     - *.rat 를 가리키면 원본 이미지로 교체 (foo.png.rat -> foo.png)
     - 존재하지 않는 경로는 경고만 출력

stdlib만 사용 (pxr 불필요). 바이너리 crate(.usd)는 건드리지 않는다.
여러 번 실행해도 결과가 같다 (idempotent).

usage:
  python convert.py              # 적용
  python convert.py --dry-run    # 변경 내용만 출력
  python convert.py --strip-ui   # 노드그래프 UI 정보까지 제거
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
USDA_DIR = SCRIPT_DIR.parent          # sim/usda
SIM_DIR = USDA_DIR.parent             # sim
SOURCE_DIR = SIM_DIR / "source"       # sim/source

# customData 안에서 지울 키 (dictionary 든 스칼라든 키 이름으로 매칭)
STRIP_CUSTOM_DATA_KEYS = {
    "HoudiniPreviewTags",
    "HoudiniIsAutoCreatedShader",
}

# --strip-ui 일 때 apiSchemas 에서 뺄 스키마 / 지울 속성 prefix
STRIP_UI_SCHEMAS = {"NodeGraphNodeAPI"}
STRIP_UI_ATTR_PREFIX = "ui:nodegraph:"

ASSET_RE = re.compile(r"@([^@]+)@")
DICT_OPEN_RE = re.compile(r"^\s*customData\s*=\s*\{\s*$")
ENTRY_KEY_RE = re.compile(r"^\s*(?:[\w\[\]]+\s+)?(\w+)\s*=")
EMPTY_META_RE = re.compile(r"[ \t]*\(\r?\n(?:[ \t]*\r?\n)*[ \t]*\)(?=\r?\n)")
API_SCHEMAS_RE = re.compile(r"^(\s*)(prepend |append |add )?apiSchemas\s*=\s*\[(.*)\](\s*)$")
UI_ATTR_RE = re.compile(r"^\s*(?:uniform\s+)?[\w\[\]]+\s+" + re.escape(STRIP_UI_ATTR_PREFIX))


# ---------------------------------------------------------------- helpers

def brace_delta(line: str) -> int:
    """따옴표 밖의 { } 개수 차이."""
    depth, in_str = 0, False
    for i, c in enumerate(line):
        if c == '"' and (i == 0 or line[i - 1] != "\\"):
            in_str = not in_str
        elif not in_str:
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
    return depth


def take_block(lines: list[str], start: int) -> int:
    """lines[start] 에서 열린 { 가 닫히는 줄의 index 를 반환."""
    depth = 0
    for i in range(start, len(lines)):
        depth += brace_delta(lines[i])
        if depth <= 0:
            return i
    raise ValueError(f"unbalanced braces from line {start + 1}")


def strip_dict_body(body: list[str], keys: set[str]) -> tuple[list[str], int]:
    """dictionary 본문에서 keys 에 해당하는 최상위 항목을 지운다. (남은 줄, 지운 항목 수)"""
    out, removed, i = [], 0, 0
    while i < len(body):
        end = i if brace_delta(body[i]) <= 0 else take_block(body, i)
        m = ENTRY_KEY_RE.match(body[i])
        if m and m.group(1) in keys:
            removed += 1
        else:
            out.extend(body[i:end + 1])
        i = end + 1
    return out, removed


# ---------------------------------------------------------------- passes

def strip_custom_data(lines: list[str], stats: dict) -> list[str]:
    out, i = [], 0
    while i < len(lines):
        if DICT_OPEN_RE.match(lines[i]):
            end = take_block(lines, i)
            body, removed = strip_dict_body(lines[i + 1:end], STRIP_CUSTOM_DATA_KEYS)
            stats["customData"] += removed
            if any(l.strip() for l in body):
                out.append(lines[i])
                out.extend(body)
                out.append(lines[end])
            i = end + 1
            continue
        out.append(lines[i])
        i += 1
    return out


def strip_ui(lines: list[str], stats: dict) -> list[str]:
    out = []
    for line in lines:
        if UI_ATTR_RE.match(line):
            stats["ui"] += 1
            continue
        m = API_SCHEMAS_RE.match(line)
        if m:
            items = [s.strip() for s in m.group(3).split(",") if s.strip()]
            kept = [s for s in items if s.strip('"') not in STRIP_UI_SCHEMAS]
            if len(kept) != len(items):
                stats["ui"] += len(items) - len(kept)
                if not kept:
                    continue
                line = f"{m.group(1)}{m.group(2) or ''}apiSchemas = [{', '.join(kept)}]{m.group(4)}"
        out.append(line)
    return out


def fix_asset_paths(text: str, usda: Path, stats: dict, missing: list[str]) -> str:
    def repl(m: re.Match) -> str:
        raw = m.group(1)
        path = raw.replace("\\", "/")
        # 검색경로로 해석되는 이름 (OmniPBR.mdl 등), URL 은 건드리지 않음
        if "/" not in path or "://" in path:
            return m.group(0)

        p = Path(path)
        target = p if p.is_absolute() else (usda.parent / p)
        target = Path(os.path.normpath(target))

        if target.suffix.lower() == ".rat" and target.with_suffix("").exists():
            target = target.with_suffix("")
            path = path[:-4]

        if p.is_absolute():
            try:
                target.relative_to(SIM_DIR)
                path = Path(os.path.relpath(target, usda.parent)).as_posix()
                if not path.startswith("."):
                    path = "./" + path
            except ValueError:
                pass  # sim/ 밖의 절대경로는 그대로 둔다

        if not target.exists():
            missing.append(f"{usda.relative_to(USDA_DIR).as_posix()}: @{raw}@")

        if path != raw:
            stats["paths"] += 1
        return f"@{path}@"

    return ASSET_RE.sub(repl, text)


def process_usda(usda: Path, args, missing: list[str]) -> dict:
    with open(usda, encoding="utf-8", newline="") as f:
        text = f.read()
    stats = {"customData": 0, "ui": 0, "paths": 0, "emptyMeta": 0}

    lines = text.splitlines(keepends=True)
    lines = strip_custom_data(lines, stats)
    if args.strip_ui:
        lines = strip_ui(lines, stats)
    new = "".join(lines)

    new, n = EMPTY_META_RE.subn("", new)
    stats["emptyMeta"] = n
    new = fix_asset_paths(new, usda, stats, missing)

    if new != text and not args.dry_run:
        with open(usda, "w", encoding="utf-8", newline="") as f:
            f.write(new)
    stats["changed"] = new != text
    return stats


def delete_rat(args) -> int:
    rats = sorted(SOURCE_DIR.rglob("*.rat")) if SOURCE_DIR.exists() else []
    for rat in rats:
        print(f"  rm {rat.relative_to(SIM_DIR).as_posix()}")
        if not args.dry_run:
            rat.unlink()
    return len(rats)


# ---------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="파일을 바꾸지 않고 변경 내용만 출력")
    ap.add_argument("--strip-ui", action="store_true", help="NodeGraphNodeAPI / ui:nodegraph:* 도 제거")
    ap.add_argument("--keep-rat", action="store_true", help=".rat 삭제 건너뛰기")
    args = ap.parse_args()

    tag = "[dry-run] " if args.dry_run else ""

    if not args.keep_rat:
        print(f"{tag}.rat cleanup ({SOURCE_DIR.relative_to(SIM_DIR.parent).as_posix()})")
        n = delete_rat(args)
        print(f"  {n} file(s)\n")

    print(f"{tag}usda cleanup ({USDA_DIR.relative_to(SIM_DIR.parent).as_posix()})")
    missing: list[str] = []
    total = 0
    for usda in sorted(USDA_DIR.rglob("*.usda")):
        s = process_usda(usda, args, missing)
        if s["changed"]:
            total += 1
            parts = [f"{k}={v}" for k, v in s.items() if k != "changed" and v]
            print(f"  {usda.relative_to(USDA_DIR).as_posix()}: {', '.join(parts)}")
    print(f"  {total} file(s) changed\n")

    if missing:
        print("missing asset paths:")
        for m in missing:
            print(f"  ! {m}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
