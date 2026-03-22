from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

from ._version import __version__

__all__ = ["BASE_DIR", "XenseCamera", "XenseGripper", "XenseSerialGripper", "read_board_sn", "__version__"]


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
    if name == "read_board_sn":
        from .xense_gripper import read_board_sn
        return read_board_sn
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
