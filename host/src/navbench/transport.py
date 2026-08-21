"""Byte-transport abstractions for deterministic and physical NavBench links.

The in-memory transport is intentionally single-threaded and deterministic.  The
POSIX adapter is a small dependency-free serial implementation for macOS/Linux;
constructing it opens a device, so tests only validate its configuration helpers.
"""

from __future__ import annotations

import array
import errno
import fcntl
import os
import select
import termios
from collections import deque
from dataclasses import dataclass
from typing import Protocol, Self


class TransportError(RuntimeError):
    """A transport could not complete the requested operation."""


class ByteTransport(Protocol):
    """Minimal interface consumed by the protocol session layer."""

    @property
    def is_open(self) -> bool: ...

    def write(self, data: bytes) -> int: ...

    def read(self, max_bytes: int = 512) -> bytes: ...

    def close(self) -> None: ...


class InMemoryEndpoint:
    """One endpoint of a deterministic, non-blocking in-memory byte link."""

    __slots__ = ("_chunks", "_open", "_peer", "_read_limit")

    def __init__(self, *, read_limit: int | None = None) -> None:
        if read_limit is not None and read_limit <= 0:
            raise ValueError("read_limit must be greater than zero")
        self._chunks: deque[bytearray] = deque()
        self._open = True
        self._peer: InMemoryEndpoint | None = None
        self._read_limit = read_limit

    @classmethod
    def pair(
        cls,
        *,
        first_read_limit: int | None = None,
        second_read_limit: int | None = None,
    ) -> tuple[Self, Self]:
        first = cls(read_limit=first_read_limit)
        second = cls(read_limit=second_read_limit)
        first._peer = second
        second._peer = first
        return first, second

    @property
    def is_open(self) -> bool:
        return self._open

    @property
    def pending_bytes(self) -> int:
        return sum(len(chunk) for chunk in self._chunks)

    def write(self, data: bytes) -> int:
        if not self._open:
            raise TransportError("transport is closed")
        if not isinstance(data, bytes):
            raise TypeError("transport data must be bytes")
        if not data:
            return 0
        peer = self._peer
        if peer is None or not peer._open:
            raise TransportError("peer is disconnected")
        peer._chunks.append(bytearray(data))
        return len(data)

    def inject(self, data: bytes) -> None:
        """Inject received bytes for deterministic corruption/fault tests."""

        if not self._open:
            raise TransportError("transport is closed")
        if not isinstance(data, bytes):
            raise TypeError("transport data must be bytes")
        if data:
            self._chunks.append(bytearray(data))

    def read(self, max_bytes: int = 512) -> bytes:
        if not self._open:
            raise TransportError("transport is closed")
        if max_bytes <= 0:
            raise ValueError("max_bytes must be greater than zero")
        if not self._chunks:
            return b""

        limit = max_bytes
        if self._read_limit is not None:
            limit = min(limit, self._read_limit)

        output = bytearray()
        while self._chunks and len(output) < limit:
            chunk = self._chunks[0]
            take = min(limit - len(output), len(chunk))
            output.extend(chunk[:take])
            del chunk[:take]
            if not chunk:
                self._chunks.popleft()
        return bytes(output)

    def close(self) -> None:
        self._open = False
        self._chunks.clear()


@dataclass(frozen=True, slots=True)
class LinkFaultState:
    packet_loss: bool = False
    packet_corruption: bool = False
    delay_steps: int = 0
    stale_frame: bool = False
    disconnected: bool = False

    def __post_init__(self) -> None:
        if self.delay_steps < 0:
            raise ValueError("delay_steps cannot be negative")


@dataclass(slots=True)
class LinkFaultStats:
    # The original unprefixed counters are the host-to-controller/write
    # direction and remain stable for existing callers.
    frames_forwarded: int = 0
    frames_dropped: int = 0
    frames_corrupted: int = 0
    frames_delayed: int = 0
    stale_frames_replayed: int = 0
    disconnect_drops: int = 0
    rx_frames_forwarded: int = 0
    rx_frames_dropped: int = 0
    rx_frames_corrupted: int = 0
    rx_frames_delayed: int = 0
    rx_stale_frames_replayed: int = 0
    rx_disconnect_drops: int = 0

    @property
    def tx_frames_forwarded(self) -> int:
        return self.frames_forwarded

    @property
    def tx_frames_dropped(self) -> int:
        return self.frames_dropped

    @property
    def tx_frames_corrupted(self) -> int:
        return self.frames_corrupted

    @property
    def tx_frames_delayed(self) -> int:
        return self.frames_delayed

    @property
    def tx_stale_frames_replayed(self) -> int:
        return self.stale_frames_replayed

    @property
    def tx_disconnect_drops(self) -> int:
        return self.disconnect_drops


class DeterministicFaultTransport:
    """COBS-frame-aware, step-driven faults in both link directions.

    ``state`` passed to :meth:`configure` controls host-to-controller writes.
    The optional ``rx_state`` controls controller-to-host reads and defaults to
    a healthy link, preserving the original one-direction API.  Bytes are
    accumulated to the COBS ``0x00`` delimiter before a fault decision, so a
    fragmented write/read is one frame and combined I/O is handled per frame.
    """

    def __init__(self, inner: ByteTransport) -> None:
        self.inner = inner
        self.state = LinkFaultState()
        self.rx_state = LinkFaultState()
        self.stats = LinkFaultStats()
        self._step_id = 0
        self._tx_input = bytearray()
        self._rx_input = bytearray()
        self._rx_ready = bytearray()
        self._tx_delayed: deque[tuple[int, bytes]] = deque()
        self._rx_delayed: deque[tuple[int, bytes]] = deque()
        self._tx_previous_frame: bytes | None = None
        self._rx_previous_frame: bytes | None = None

    @property
    def is_open(self) -> bool:
        return self.inner.is_open

    @property
    def pending_delayed_frames(self) -> int:
        return len(self._tx_delayed) + len(self._rx_delayed)

    def configure(
        self,
        step_id: int,
        state: LinkFaultState,
        rx_state: LinkFaultState | None = None,
    ) -> None:
        if step_id < self._step_id:
            raise ValueError("fault transport step cannot move backwards")
        self._step_id = step_id
        self.state = state
        self.rx_state = rx_state or LinkFaultState()
        self._release_due_tx()
        self._release_due_rx()

    def write(self, data: bytes) -> int:
        if not isinstance(data, bytes):
            raise TypeError("transport data must be bytes")
        if not data:
            return 0
        self._tx_input.extend(data)
        for frame in self._take_complete_frames(self._tx_input):
            self._apply_tx_fault(frame)
        return len(data)

    def read(self, max_bytes: int = 512) -> bytes:
        if max_bytes <= 0:
            raise ValueError("max_bytes must be greater than zero")
        self._release_due_rx()
        while not self._rx_ready:
            incoming = self.inner.read(max_bytes)
            if not incoming:
                break
            self._rx_input.extend(incoming)
            for frame in self._take_complete_frames(self._rx_input):
                self._apply_rx_fault(frame)
        output = bytes(self._rx_ready[:max_bytes])
        del self._rx_ready[:max_bytes]
        return output

    def close(self) -> None:
        self._tx_input.clear()
        self._rx_input.clear()
        self._rx_ready.clear()
        self._tx_delayed.clear()
        self._rx_delayed.clear()
        self.inner.close()

    def _apply_tx_fault(self, frame: bytes) -> None:
        previous = self._tx_previous_frame
        self._tx_previous_frame = frame
        current = frame
        if self.state.disconnected:
            self.stats.disconnect_drops += 1
            return
        if self.state.packet_loss:
            self.stats.frames_dropped += 1
            return
        if self.state.stale_frame and previous is not None:
            current = previous
            self.stats.stale_frames_replayed += 1
        if self.state.packet_corruption and len(current) > 2:
            current = self._corrupt(current, direction_salt=17)
            self.stats.frames_corrupted += 1
        if self.state.delay_steps:
            self._tx_delayed.append(
                (self._step_id + self.state.delay_steps, current)
            )
            self.stats.frames_delayed += 1
            return
        self._forward_tx(current)

    def _apply_rx_fault(self, frame: bytes) -> None:
        previous = self._rx_previous_frame
        self._rx_previous_frame = frame
        current = frame
        if self.rx_state.disconnected:
            self.stats.rx_disconnect_drops += 1
            return
        if self.rx_state.packet_loss:
            self.stats.rx_frames_dropped += 1
            return
        if self.rx_state.stale_frame and previous is not None:
            current = previous
            self.stats.rx_stale_frames_replayed += 1
        if self.rx_state.packet_corruption and len(current) > 2:
            current = self._corrupt(current, direction_salt=29)
            self.stats.rx_frames_corrupted += 1
        if self.rx_state.delay_steps:
            self._rx_delayed.append(
                (self._step_id + self.rx_state.delay_steps, current)
            )
            self.stats.rx_frames_delayed += 1
            return
        self._forward_rx(current)

    def _release_due_tx(self) -> None:
        while self._tx_delayed and self._tx_delayed[0][0] <= self._step_id:
            _, frame = self._tx_delayed.popleft()
            if self.state.disconnected:
                self.stats.disconnect_drops += 1
            else:
                self._forward_tx(frame)

    def _release_due_rx(self) -> None:
        while self._rx_delayed and self._rx_delayed[0][0] <= self._step_id:
            _, frame = self._rx_delayed.popleft()
            if self.rx_state.disconnected:
                self.stats.rx_disconnect_drops += 1
            else:
                self._forward_rx(frame)

    def _forward_tx(self, frame: bytes) -> None:
        offset = 0
        while offset < len(frame):
            written = self.inner.write(frame[offset:])
            if written <= 0:
                raise TransportError(
                    "inner transport could not accept a complete test frame"
                )
            offset += written
        self.stats.frames_forwarded += 1

    def _forward_rx(self, frame: bytes) -> None:
        self._rx_ready.extend(frame)
        self.stats.rx_frames_forwarded += 1

    def _corrupt(self, frame: bytes, *, direction_salt: int) -> bytes:
        corrupted = bytearray(frame)
        index = 1 + (self._step_id * direction_salt) % (len(corrupted) - 2)
        corrupted[index] ^= 0x01
        return bytes(corrupted)

    @staticmethod
    def _take_complete_frames(buffer: bytearray) -> tuple[bytes, ...]:
        frames: list[bytes] = []
        start = 0
        while True:
            delimiter = buffer.find(0, start)
            if delimiter < 0:
                break
            frames.append(bytes(buffer[start : delimiter + 1]))
            start = delimiter + 1
        if start:
            del buffer[:start]
        return tuple(frames)


@dataclass(frozen=True, slots=True)
class SerialConfig:
    device: str
    baud: int = 115200

    def __post_init__(self) -> None:
        if not self.device or "\x00" in self.device:
            raise ValueError("device must be a non-empty path")
        if self.baud not in _BAUD_CONSTANTS:
            supported = ", ".join(str(value) for value in _BAUD_CONSTANTS)
            raise ValueError(f"unsupported baud; choose one of: {supported}")


class PosixSerialTransport:
    """Non-blocking raw POSIX serial adapter with no third-party dependency."""

    __slots__ = ("_fd", "config")

    def __init__(self, config: SerialConfig) -> None:
        self.config = config
        try:
            fd = os.open(
                config.device,
                os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
            )
        except OSError as error:
            raise TransportError(
                f"cannot open serial device {config.device!r}: {error}"
            ) from error

        try:
            _configure_serial_fd(fd, config.baud)
            _assert_dtr(fd)
        except BaseException:
            os.close(fd)
            raise
        self._fd = fd

    @property
    def is_open(self) -> bool:
        return self._fd >= 0

    def write(self, data: bytes) -> int:
        if not self.is_open:
            raise TransportError("transport is closed")
        if not isinstance(data, bytes):
            raise TypeError("transport data must be bytes")
        if not data:
            return 0
        try:
            return os.write(self._fd, data)
        except BlockingIOError:
            return 0
        except OSError as error:
            raise TransportError(f"serial write failed: {error}") from error

    def read(self, max_bytes: int = 512) -> bytes:
        if not self.is_open:
            raise TransportError("transport is closed")
        if max_bytes <= 0:
            raise ValueError("max_bytes must be greater than zero")
        try:
            ready, _, _ = select.select([self._fd], [], [], 0.0)
            if not ready:
                return b""
            return os.read(self._fd, max_bytes)
        except BlockingIOError:
            return b""
        except OSError as error:
            if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return b""
            raise TransportError(f"serial read failed: {error}") from error

    def reset_input_buffer(self) -> None:
        """Discard bytes received before the binary session starts."""

        if not self.is_open:
            raise TransportError("transport is closed")
        try:
            termios.tcflush(self._fd, termios.TCIFLUSH)
        except (termios.error, OSError) as error:
            raise TransportError(f"cannot flush serial input: {error}") from error

    def close(self) -> None:
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


_BAUD_CONSTANTS: dict[int, int] = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}


def _configure_serial_fd(fd: int, baud: int) -> None:
    try:
        attributes = termios.tcgetattr(fd)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attributes[3] = 0
        attributes[4] = _BAUD_CONSTANTS[baud]
        attributes[5] = _BAUD_CONSTANTS[baud]
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attributes)
        termios.tcflush(fd, termios.TCIOFLUSH)
    except (KeyError, termios.error, OSError) as error:
        raise TransportError(f"cannot configure serial device: {error}") from error


def _assert_dtr(fd: int) -> None:
    """Assert USB CDC DTR once without a low-to-high reset pulse."""

    request = getattr(termios, "TIOCMBIS", None)
    dtr = getattr(termios, "TIOCM_DTR", None)
    if request is None or dtr is None:
        return
    control_bits = array.array("i", [dtr])
    try:
        fcntl.ioctl(fd, request, control_bits, True)
    except OSError as error:
        raise TransportError(f"cannot assert serial DTR: {error}") from error
