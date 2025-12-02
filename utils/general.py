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

import logging
import sys
from typing import Optional  # noqa: F401
import numpy as np
import time
import threading
from typing import Callable


__all__ = ["Logger"]


class Logger:
    """
    Logger class for logging messages
    """
    def __init__(self, name: str, level: int = logging.INFO):
        """
        initialize logger

        Args:
            name: logger name (usually the node name)
            level: logger level
        """
        self.name = name
        self.logger = logging.getLogger(f"{name}")
        self.logger.setLevel(level)

        # if no handlers, add default console handler
        if not self.logger.handlers:
            handler = logging.StreamHandler(sys.stdout)
            handler.setLevel(level)

            # set formatter
            formatter = logging.Formatter(
                '[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s',
                datefmt='%m-%d %H:%M:%S'
            )
            handler.setFormatter(formatter)

            self.logger.addHandler(handler)

    def log(self, level: int, message: str):
        self.logger.log(level, message)

    def debug(self, message: str):
        """log debug information"""
        self.logger.debug(message)

    def info(self, message: str):
        """log information"""
        self.logger.info(message)

    def warn(self, message: str):
        """log warning"""
        self.logger.warning(message)

    def warning(self, message: str):
        """log warning (alias)"""
        self.warn(message)

    def error(self, message: str):
        """log error"""
        self.logger.error(message)

    def fatal(self, message: str):
        """log fatal error"""
        self.logger.critical(message)

    def critical(self, message: str):
        """log critical error (alias)"""
        self.fatal(message)

    def set_level(self, level):
        """
        set logger level

        Args:
            level: new logger level
        """
        self.logger.setLevel(level)
        for handler in self.logger.handlers:
            handler.setLevel(level)


class DataBuffer:
    """
    use np.array to simulate a circular buffer with maximum length maxlen, for storing np.array data with shape
    """
    def __init__(self, maxlen, shape, dtype=np.float32):
        self.maxlen = maxlen
        self.shape = shape
        self.buffer = np.zeros((maxlen,) + shape, dtype=dtype)
        self.start = 0
        self.size = 0

    def __len__(self):
        return self.size

    def __getitem__(self, index):
        if index < 0 or index >= self.size:
            raise IndexError("Index out of range")
        return self.buffer[(self.start + index) % self.maxlen]

    def __iter__(self):
        for i in range(self.size):
            yield self.buffer[(self.start + i) % self.maxlen]

    def clear(self):
        self.start = 0
        self.size = 0

    def extend(self, data):
        """add multiple data to buffer"""
        if data is None:
            return

        # convert to array and validate shape
        data_array = np.asarray(data).reshape(-1, *self.shape)
        n = len(data_array)
        if n == 0:
            return

        if n >= self.maxlen:
            # data is enough, directly overwrite the entire buffer
            self.buffer[:] = data_array[-self.maxlen:]
            self.start = 0
            self.size = self.maxlen
        else:
            # data is less, need to keep some old data
            keep_max = self.maxlen - n
            end = (self.start + self.size) % self.maxlen
            if keep_max < self.size:
                self.start = (self.start + (self.size - keep_max)) % self.maxlen
                self.size = self.maxlen
            else:
                self.size += n

            self._copy_to_buffer(data_array, end)

    def _copy_to_buffer(self, data_slice, start_pos):
        """copy data slice to buffer, from start_pos, at most two copies"""
        n = len(data_slice)
        if n == 0:
            return

        # calculate the space from start_pos to the end of the buffer
        space_to_end = self.maxlen - start_pos

        if n <= space_to_end:
            self.buffer[start_pos:start_pos + n] = data_slice
        else:
            self.buffer[start_pos:] = data_slice[:space_to_end]
            self.buffer[:n - space_to_end] = data_slice[space_to_end:]

    def to_array(self):
        """return a copy of the current buffer content as an array"""
        if self.size == 0:
            return np.empty((0,) + self.shape)

        end = (self.start + self.size) % self.maxlen
        if end > self.start:
            return self.buffer[self.start:end].copy()
        else:
            return np.concatenate([self.buffer[self.start:], self.buffer[:end]])


class Timer:
    """
    timer class for running a callback function at a fixed frequency
    """
    def __init__(
        self,
        frequency,
        callback: Callable,
        start: bool = False,
    ):
        """
        initialize timer

        Args:
            frequency: callback frequency
            callback: callback function
            start: start the timer immediately
        """
        self._sleep_time_s = 1 / frequency
        self._is_running = threading.Event()
        self._callback = callback
        self._thread = None

        # if windows system, set high precision timer
        if sys.platform.startswith("win"):
            import ctypes
            ctypes.windll.winmm.timeBeginPeriod(1)

        if start:
            self.start()

    def run(self):
        self._is_running.set()
        while self._is_running.is_set():
            t0 = time.monotonic()  # use monotonic clock to avoid system time jump

            self._callback()

            elapsed_time = time.monotonic() - t0
            if self._sleep_time_s > elapsed_time:
                time.sleep(self._sleep_time_s - elapsed_time)

    def start(self):
        """start the timer"""
        if self._is_running.is_set():
            return

        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()

    def stop(self):
        self._is_running.clear()
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def alive(self):
        return self._thread.is_alive()
