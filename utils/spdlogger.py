#!/usr/bin/env python

# Copyright 2025 The Xense Robotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Logging utilities using spdlog.
"""

import spdlog

__all__ = ["get_logger", "set_log_level"]

# Cache for logger instances
_loggers: dict[str, spdlog.Logger] = {}


def get_logger(name: str = "xgripper") -> spdlog.Logger:
    """
    Get or create a spdlog logger with the given name.

    Args:
        name: Logger name

    Returns:
        spdlog.Logger instance
    """
    if name not in _loggers:
        # Create a console logger with color support
        logger = spdlog.ConsoleLogger(name, multithreaded=True, stdout=True, colored=True)
        # Set default pattern: [time] [level] [logger_name] message
        logger.set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v")
        logger.set_level(spdlog.LogLevel.INFO)
        _loggers[name] = logger
    return _loggers[name]


def set_log_level(level: str | spdlog.LogLevel, logger_name: str | None = None) -> None:
    """
    Set log level for a specific logger or all loggers.

    Args:
        level: Log level ("DEBUG", "INFO", "WARN", "ERROR", "CRITICAL") or spdlog.LogLevel
        logger_name: Logger name. If None, sets level for all loggers.
    """
    # Convert string level to spdlog.LogLevel
    if isinstance(level, str):
        level_map = {
            "DEBUG": spdlog.LogLevel.DEBUG,
            "INFO": spdlog.LogLevel.INFO,
            "WARN": spdlog.LogLevel.WARN,
            "WARNING": spdlog.LogLevel.WARN,
            "ERROR": spdlog.LogLevel.ERR,
            "CRITICAL": spdlog.LogLevel.CRITICAL,
        }
        level = level_map.get(level.upper(), spdlog.LogLevel.INFO)

    if logger_name:
        if logger_name in _loggers:
            _loggers[logger_name].set_level(level)
    else:
        for logger in _loggers.values():
            logger.set_level(level)

