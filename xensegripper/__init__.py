from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

from ._version import __version__

__all__ = ["BASE_DIR", "XenseCamera", "XenseGripper", "XenseSerialGripper", "__version__"]


def __getattr__(name: str):
    if name == "XenseGripper":
        from .xense_gripper import XenseGripper
        return XenseGripper
    if name == "XenseSerialGripper":
        from .xense_gripper import XenseSerialGripper
        return XenseSerialGripper
    if name == "XenseCamera":
        from .node.camera_client import CameraNode as XenseCamera
        return XenseCamera
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
