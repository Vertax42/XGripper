from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys

from setuptools import setup
from setuptools.command.build_py import build_py as _build_py

try:
    from setuptools.command.develop import develop as _develop
except ImportError:  # pragma: no cover
    _develop = None

try:
    from setuptools.command.editable_wheel import editable_wheel as _editable_wheel
except ImportError:  # pragma: no cover
    _editable_wheel = None


REPO_ROOT = Path(__file__).resolve().parent
PYSURVIVE_PACKAGE_DIR = REPO_ROOT / "pysurvive"
LIBSURVIVE_SOURCE_DIR = REPO_ROOT / "third_party" / "libsurvive"
LIBSURVIVE_BUILD_DIR = REPO_ROOT / "build" / "libsurvive"
LIBSURVIVE_STAGE_DIR = LIBSURVIVE_BUILD_DIR / "install"
_PYSURVIVE_BUILT = False


def _purge_old_pysurvive_artifacts() -> None:
    for pattern in ("libsurvive.so*", "libsurvive.dylib*", "libsurvive.dll*", "survive.dll*", "survive.lib*"):
        for artifact in PYSURVIVE_PACKAGE_DIR.glob(pattern):
            if artifact.is_dir():
                shutil.rmtree(artifact)
            else:
                artifact.unlink()

    plugin_root = PYSURVIVE_PACKAGE_DIR / "libsurvive"
    if plugin_root.exists():
        shutil.rmtree(plugin_root)


def _build_vendored_pysurvive() -> None:
    global _PYSURVIVE_BUILT
    if _PYSURVIVE_BUILT:
        return

    if not LIBSURVIVE_SOURCE_DIR.exists():
        raise RuntimeError(f"Vendored libsurvive source not found: {LIBSURVIVE_SOURCE_DIR}")

    cmake = shutil.which("cmake")
    if cmake is None:
        raise RuntimeError("cmake is required to build vendored libsurvive for pysurvive")

    if LIBSURVIVE_BUILD_DIR.exists():
        shutil.rmtree(LIBSURVIVE_BUILD_DIR)

    LIBSURVIVE_BUILD_DIR.mkdir(parents=True, exist_ok=True)
    LIBSURVIVE_STAGE_DIR.mkdir(parents=True, exist_ok=True)
    PYSURVIVE_PACKAGE_DIR.mkdir(parents=True, exist_ok=True)
    _purge_old_pysurvive_artifacts()

    configure_cmd = [
        cmake,
        "-S",
        str(LIBSURVIVE_SOURCE_DIR),
        "-B",
        str(LIBSURVIVE_BUILD_DIR),
        "-G",
        "Unix Makefiles",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_INSTALL_PREFIX={LIBSURVIVE_STAGE_DIR}",
        f"-DPYTHON_GENERATED_DIR={PYSURVIVE_PACKAGE_DIR}{os.sep}",
        f"-DLIB_INSTALL_DIR={PYSURVIVE_PACKAGE_DIR}{os.sep}",
        f"-DPython3_EXECUTABLE={sys.executable}",
        f"-DPYTHON_EXECUTABLE={sys.executable}",
        "-DDOWNLOAD_EIGEN=OFF",
        "-DUSE_EIGEN=ON",
        "-DUSE_HIDAPI=ON",
        "-DBUILD_APPLICATIONS=OFF",
        "-DENABLE_TESTS=OFF",
    ]

    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        configure_cmd.append(f"-DCMAKE_PREFIX_PATH={conda_prefix}")

    subprocess.run(configure_cmd, check=True, cwd=REPO_ROOT)
    subprocess.run(
        [cmake, "--build", str(LIBSURVIVE_BUILD_DIR), "--parallel", str(os.cpu_count() or 1)],
        check=True,
        cwd=REPO_ROOT,
    )
    subprocess.run([cmake, "--install", str(LIBSURVIVE_BUILD_DIR)], check=True, cwd=REPO_ROOT)

    generated = PYSURVIVE_PACKAGE_DIR / "pysurvive_generated.py"
    if not generated.exists():
        raise RuntimeError(f"Expected generated pysurvive binding not found: {generated}")

    _PYSURVIVE_BUILT = True


class _BuildPysurviveMixin:
    def run(self):
        _build_vendored_pysurvive()
        super().run()


class build_py(_BuildPysurviveMixin, _build_py):
    pass


cmdclass = {"build_py": build_py}

if _editable_wheel is not None:
    class editable_wheel(_BuildPysurviveMixin, _editable_wheel):
        pass

    cmdclass["editable_wheel"] = editable_wheel

if _develop is not None:
    class develop(_BuildPysurviveMixin, _develop):
        pass

    cmdclass["develop"] = develop


setup(cmdclass=cmdclass)
