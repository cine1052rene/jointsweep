"""Compile a URDF or MJCF file with MuJoCo, resolving mesh paths the file cannot."""
from __future__ import annotations

import re
from pathlib import Path

import mujoco

MESH_REF = re.compile(r"""(?:filename|file)\s*=\s*["']([^"']+\.(?:stl|obj|msh|STL|OBJ))["']""")


def is_urdf(path: Path) -> bool:
    head = path.read_text(encoding="utf-8", errors="replace")[:4000]
    return path.suffix.lower() == ".urdf" or "<robot" in head


def _find(root: Path, name: str) -> Path | None:
    hits = sorted(root.rglob(name), key=lambda p: len(p.parts))
    return hits[0] if hits else None


def resolve_meshes(path: Path, search: Path | None = None) -> str:
    """The file's text with mesh references that do not exist on disk (package://pkg/..., or a
    folder kept elsewhere) replaced by a file found by name under `search`."""
    text = path.read_text(encoding="utf-8", errors="replace")
    base = path.parent
    if not is_urdf(path):
        m = re.search(r"""meshdir\s*=\s*["']([^"']*)["']""", text)
        base = path.parent / m.group(1) if m else path.parent
    search = search or path.parent
    for ref in set(MESH_REF.findall(text)):
        local = ref.split("://", 1)[1].split("/", 1)[-1] if "://" in ref else ref
        if (base / local).is_file():
            continue
        found = _find(search, Path(local).name)
        if found:
            text = text.replace(f'"{ref}"', f'"{found.as_posix()}"').replace(f"'{ref}'", f"'{found.as_posix()}'")
    return text


def load(path: str | Path, search: str | Path | None = None) -> mujoco.MjModel:
    """URDF or MJCF -> compiled MjModel. Visual-only geometry is dropped for URDF so that
    contacts come from the collision elements, as they do in the simulator."""
    p = Path(path).resolve()
    text = resolve_meshes(p, Path(search).resolve() if search else None)
    urdf = is_urdf(p)
    tmp = None
    if text != p.read_text(encoding="utf-8", errors="replace"):
        # includes and meshdir stay relative to the original folder, so the rewrite lives beside it
        tmp = p.with_name(p.stem + ".jointsweep-tmp" + p.suffix)
        tmp.write_text(text, encoding="utf-8")
    try:
        spec = mujoco.MjSpec.from_file(str(tmp or p))
        if urdf:
            spec.meshdir = str(p.parent)
            spec.compiler.discardvisual = True
            spec.compiler.fusestatic = False  # keep base_link's name instead of merging it into 'world'
        return spec.compile()
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)
