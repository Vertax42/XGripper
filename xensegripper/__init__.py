from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

from ._version import __version__

__all__ = ["BASE_DIR", "XenseGripper", "XenseSerialGripper", "__version__"]


def __getattr__(name: str):
    if name in ("XenseGripper", "XenseSerialGripper"):
        from .xense_gripper import XenseGripper, XenseSerialGripper
        return {"XenseGripper": XenseGripper, "XenseSerialGripper": XenseSerialGripper}[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
