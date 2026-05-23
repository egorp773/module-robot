from pathlib import Path

Import("env")

project_dir = Path(env.subst("$PROJECT_DIR"))
toolchain_bin = project_dir / ".tools" / "w64devkit" / "bin"
if not toolchain_bin.exists():
    toolchain_bin = project_dir.parent / ".tools" / "w64devkit" / "bin"

if toolchain_bin.exists():
    env.PrependENVPath("PATH", str(toolchain_bin))
    env.Replace(CC="gcc", CXX="g++")
