"""Adapter for the native C++ firmware bridge used by host CIL tests."""

from __future__ import annotations

import os
import select
import subprocess
from pathlib import Path

from navbench.transport import TransportError


class NativeFirmwareTransport:
    """Synchronous deterministic ByteTransport backed by a C++ subprocess."""

    def __init__(self, executable: Path, *, response_timeout_s: float = 2.0) -> None:
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise ValueError(f"native firmware executable is not runnable: {executable}")
        if response_timeout_s <= 0.0:
            raise ValueError("response_timeout_s must be greater than zero")
        self._process = subprocess.Popen(
            [str(executable)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._rx = bytearray()
        self._stdout_buffer = bytearray()
        self._now_ms = 0
        self._response_timeout_s = response_timeout_s

    @property
    def is_open(self) -> bool:
        return self._process.poll() is None

    @property
    def pending_bytes(self) -> int:
        return len(self._rx)

    def set_time(self, now_ms: int) -> None:
        if isinstance(now_ms, bool) or not isinstance(now_ms, int):
            raise TypeError("now_ms must be an integer")
        if now_ms < self._now_ms:
            raise ValueError("native firmware time cannot move backwards")
        self._now_ms = now_ms

    def write(self, data: bytes) -> int:
        if not isinstance(data, bytes):
            raise TypeError("transport data must be bytes")
        if not data:
            return 0
        self._exchange(f"FEED {self._now_ms} {data.hex()}")
        return len(data)

    def tick(self, now_ms: int, step_id: int) -> None:
        self.set_time(now_ms)
        if step_id < 0 or step_id > 0xFFFFFFFF:
            raise ValueError("step_id must fit in uint32")
        self._exchange(f"TICK {now_ms} {step_id}")

    def reset(self, now_ms: int = 0) -> None:
        if now_ms < 0:
            raise ValueError("now_ms cannot be negative")
        self._now_ms = now_ms
        self._rx.clear()
        self._exchange(f"RESET {now_ms}")

    def read(self, max_bytes: int = 512) -> bytes:
        if max_bytes <= 0:
            raise ValueError("max_bytes must be greater than zero")
        if not self.is_open and not self._rx:
            raise TransportError("native firmware process is closed")
        output = bytes(self._rx[:max_bytes])
        del self._rx[:max_bytes]
        return output

    def close(self) -> None:
        if not self.is_open:
            return
        stdin = self._process.stdin
        if stdin is not None:
            try:
                stdin.write(b"QUIT\n")
                stdin.flush()
            except BrokenPipeError:
                pass
            stdin.close()
        try:
            self._process.wait(timeout=self._response_timeout_s)
        except subprocess.TimeoutExpired:
            self._process.terminate()
            self._process.wait(timeout=self._response_timeout_s)
        if self._process.stdout is not None:
            self._process.stdout.close()
        if self._process.stderr is not None:
            self._process.stderr.close()

    def __enter__(self) -> NativeFirmwareTransport:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _exchange(self, command: str) -> None:
        if not self.is_open:
            raise TransportError(self._failure_message("native firmware exited"))
        stdin = self._process.stdin
        stdout = self._process.stdout
        if stdin is None or stdout is None:
            raise TransportError("native firmware pipes are unavailable")
        try:
            stdin.write((command + "\n").encode("ascii"))
            stdin.flush()
        except BrokenPipeError as error:
            raise TransportError(self._failure_message("native firmware pipe broke")) from error

        while True:
            value = self._readline(stdout.fileno())
            if value == b".":
                return
            if value.startswith(b"ERROR "):
                detail = value[6:].decode("ascii", errors="replace")
                raise TransportError(f"native firmware bridge error: {detail}")
            try:
                self._rx.extend(bytes.fromhex(value.decode("ascii")))
            except (UnicodeDecodeError, ValueError) as error:
                raise TransportError(
                    f"native firmware returned non-hex output: {value!r}"
                ) from error

    def _readline(self, file_descriptor: int) -> bytes:
        while True:
            newline = self._stdout_buffer.find(b"\n")
            if newline >= 0:
                line = bytes(self._stdout_buffer[:newline]).rstrip(b"\r")
                del self._stdout_buffer[: newline + 1]
                return line
            ready, _, _ = select.select(
                [file_descriptor],
                [],
                [],
                self._response_timeout_s,
            )
            if not ready:
                raise TransportError("native firmware response timed out")
            chunk = os.read(file_descriptor, 4096)
            if not chunk:
                raise TransportError(
                    self._failure_message("native firmware closed stdout")
                )
            self._stdout_buffer.extend(chunk)

    def _failure_message(self, prefix: str) -> str:
        stderr = self._process.stderr
        detail = ""
        if stderr is not None and self._process.poll() is not None:
            detail = stderr.read().decode("utf-8", errors="replace").strip()
        return f"{prefix}: {detail}" if detail else prefix
