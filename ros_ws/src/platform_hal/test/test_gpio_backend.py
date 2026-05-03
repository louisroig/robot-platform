"""Unit tests for Pca9685Backend with a mocked smbus2.

Verifies the bits where a quiet bug would silently misdrive an H-bridge:
  - presence-probe → init register sequence (sleep, prescale, wake, AI)
  - prescale arithmetic per datasheet §7.3.5
  - per-channel address math (LED0_ON_L + 4·channel)
  - mutex ordering (opposite channel zeroed BEFORE active set)
  - full-off encoding (OFF_H bit 4) for direction-zero
  - duty count arithmetic for partial duty
  - frequency-out-of-range and missing-chip error paths
"""
from __future__ import annotations

import sys

import pytest


# ---------------------------------------------------------------------------
# Fake smbus2 module — recorded at sys.modules level so the lazy
# `import smbus2` inside Pca9685Backend picks up the fake.
# ---------------------------------------------------------------------------

class _FakeSMBus:
    """Records every transaction; surfaces them via .calls for assertions."""

    def __init__(self, bus_num):
        self.bus_num = bus_num
        self.closed = False
        self.calls: list[tuple] = []
        self.read_byte_data_returns = 0x11  # PCA9685 reset MODE1; non-raising
        self.read_byte_data_raises: OSError | None = None

    def read_byte_data(self, addr, reg):
        self.calls.append(('read_byte_data', addr, reg))
        if self.read_byte_data_raises is not None:
            raise self.read_byte_data_raises
        return self.read_byte_data_returns

    def write_byte_data(self, addr, reg, val):
        self.calls.append(('write_byte_data', addr, reg, val))

    def write_i2c_block_data(self, addr, reg, block):
        self.calls.append(('write_i2c_block_data', addr, reg, list(block)))

    def close(self):
        self.closed = True


class _FakeSmbus2Module:
    SMBus = _FakeSMBus


def _make_node(i2c_bus: int = 1, i2c_address: int = 0x40):
    """Minimal stand-in for an rclpy Node — only get_parameter is exercised."""
    class _Param:
        def __init__(self, value):
            self.value = value
    class _Node:
        def get_parameter(self, name):
            if name == 'i2c_bus':
                return _Param(i2c_bus)
            if name == 'i2c_address':
                return _Param(i2c_address)
            raise KeyError(name)
    return _Node()


@pytest.fixture
def fake_smbus2(monkeypatch):
    fake = _FakeSmbus2Module()
    monkeypatch.setitem(sys.modules, 'smbus2', fake)
    return fake


@pytest.fixture
def backend(fake_smbus2):
    from platform_hal.gpio_backend import Pca9685Backend
    return Pca9685Backend(_make_node())


# ---------------------------------------------------------------------------
# Constants for readability in assertions.
# ---------------------------------------------------------------------------

ADDR = 0x40
REG_MODE1 = 0x00
REG_PRESCALE = 0xFE
REG_LED0_ON_L = 0x06
BIT_SLEEP = 0x10
BIT_AI = 0x20
FULL_OFF_BLOCK = [0x00, 0x00, 0x00, 0x10]


def _channel_base(ch: int) -> int:
    return REG_LED0_ON_L + 4 * ch


def _writes_to(calls, base):
    """Return only the i2c_block_data calls targeting a particular register base."""
    return [c for c in calls if c[0] == 'write_i2c_block_data' and c[2] == base]


# ---------------------------------------------------------------------------
# setup() — init sequence, prescale, channel zero on entry.
# ---------------------------------------------------------------------------

class TestSetup:
    def test_init_sequence_in_order(self, backend):
        backend.setup(0, 1, 2, 3, pwm_frequency_hz=1000, gpio_chip=4)
        bus = backend._bus  # noqa: SLF001 — white-box: confirm the fake recorded ours
        # Init must be: probe → sleep → prescale → wake → set AI.
        # round(25_000_000 / (4096 * 1000)) - 1 = 6 - 1 = 5
        head = bus.calls[:5]
        assert head == [
            ('read_byte_data', ADDR, REG_MODE1),
            ('write_byte_data', ADDR, REG_MODE1, BIT_SLEEP),
            ('write_byte_data', ADDR, REG_PRESCALE, 5),
            ('write_byte_data', ADDR, REG_MODE1, 0x00),
            ('write_byte_data', ADDR, REG_MODE1, BIT_AI),
        ]

    def test_setup_zeroes_all_four_configured_channels(self, backend):
        backend.setup(7, 8, 9, 10, pwm_frequency_hz=1000, gpio_chip=4)
        bus = backend._bus  # noqa: SLF001
        # After init, exactly one full-off write per channel should appear.
        block_writes = [c for c in bus.calls if c[0] == 'write_i2c_block_data']
        assert len(block_writes) == 4
        seen_bases = {c[2] for c in block_writes}
        assert seen_bases == {_channel_base(7), _channel_base(8),
                              _channel_base(9), _channel_base(10)}
        for c in block_writes:
            assert c[3] == FULL_OFF_BLOCK

    def test_prescale_at_1526hz_clamps_low(self, backend):
        # round(25e6 / (4096 * 1526)) - 1 ≈ round(4.0) - 1 = 3 (the floor).
        backend.setup(0, 1, 2, 3, pwm_frequency_hz=1526, gpio_chip=4)
        bus = backend._bus  # noqa: SLF001
        prescale_writes = [c for c in bus.calls
                           if c[0] == 'write_byte_data' and c[2] == REG_PRESCALE]
        assert prescale_writes == [('write_byte_data', ADDR, REG_PRESCALE, 3)]

    def test_frequency_below_range_raises(self, backend):
        with pytest.raises(ValueError, match='24-1526'):
            backend.setup(0, 1, 2, 3, pwm_frequency_hz=10, gpio_chip=4)

    def test_frequency_above_range_raises(self, backend):
        with pytest.raises(ValueError, match='24-1526'):
            backend.setup(0, 1, 2, 3, pwm_frequency_hz=2000, gpio_chip=4)

    def test_missing_chip_raises_runtime_error_not_oserror(self, fake_smbus2):
        from platform_hal.gpio_backend import Pca9685Backend
        # Make the presence probe fail like a real bus would when nothing
        # is acking at 0x40 — the backend must wrap that in a clear message
        # so bringup tells you "no PCA9685" instead of a raw I/O error.
        original_init = _FakeSMBus.__init__
        def patched_init(self, bus_num):
            original_init(self, bus_num)
            self.read_byte_data_raises = OSError(121, 'Remote I/O error')
        fake_smbus2.SMBus.__init__ = patched_init
        try:
            b = Pca9685Backend(_make_node())
            with pytest.raises(RuntimeError, match='no PCA9685 acking'):
                b.setup(0, 1, 2, 3, pwm_frequency_hz=1000, gpio_chip=4)
        finally:
            fake_smbus2.SMBus.__init__ = original_init


# ---------------------------------------------------------------------------
# write() — direction, mutex order, duty math.
# ---------------------------------------------------------------------------

class TestWrite:
    def _setup(self, backend):
        backend.setup(0, 1, 2, 3, pwm_frequency_hz=1000, gpio_chip=4)
        backend._bus.calls.clear()  # noqa: SLF001

    def test_positive_duty_zeroes_lpwm_first_then_sets_rpwm(self, backend):
        self._setup(backend)
        backend.write(left_signed=0.5, right_signed=0.0)
        bus = backend._bus  # noqa: SLF001
        block_writes = [c for c in bus.calls if c[0] == 'write_i2c_block_data']
        # Left side: lpwm (ch 1) zeroed first, then rpwm (ch 0) set.
        # Right side: zero/zero (signed=0 path zeroes both rpwm then lpwm).
        # round(0.5 * 4095) = 2048 = 0x0800 → off_l=0x00, off_h=0x08.
        assert block_writes[0] == (
            'write_i2c_block_data', ADDR, _channel_base(1), FULL_OFF_BLOCK,
        )
        assert block_writes[1] == (
            'write_i2c_block_data', ADDR, _channel_base(0), [0x00, 0x00, 0x00, 0x08],
        )
        # Right side untouched in direction (both zero) — but the mutex code
        # still emits two writes per track to keep the invariant uniform.
        right_writes = [c for c in block_writes
                        if c[2] in (_channel_base(2), _channel_base(3))]
        assert all(c[3] == FULL_OFF_BLOCK for c in right_writes)

    def test_negative_duty_zeroes_rpwm_first_then_sets_lpwm(self, backend):
        self._setup(backend)
        backend.write(left_signed=0.0, right_signed=-0.3)
        bus = backend._bus  # noqa: SLF001
        right_writes = _writes_to(bus.calls, _channel_base(2)) + \
                       _writes_to(bus.calls, _channel_base(3))
        # Order matters: the rpwm zero must appear in the call log BEFORE
        # the lpwm activation (no overlapping non-zero on both sides).
        rpwm_zero_idx = bus.calls.index(
            ('write_i2c_block_data', ADDR, _channel_base(2), FULL_OFF_BLOCK)
        )
        # round(0.3 * 4095) = 1228 = 0x04CC → off_l=0xCC, off_h=0x04.
        lpwm_set_call = (
            'write_i2c_block_data', ADDR, _channel_base(3), [0x00, 0x00, 0xCC, 0x04],
        )
        lpwm_set_idx = bus.calls.index(lpwm_set_call)
        assert rpwm_zero_idx < lpwm_set_idx

    def test_zero_signed_writes_full_off_to_both_channels(self, backend):
        self._setup(backend)
        backend.write(0.0, 0.0)
        bus = backend._bus  # noqa: SLF001
        block_writes = [c for c in bus.calls if c[0] == 'write_i2c_block_data']
        # All four configured channels should land on the full-off block.
        bases = {c[2] for c in block_writes}
        assert bases == {_channel_base(0), _channel_base(1),
                         _channel_base(2), _channel_base(3)}
        assert all(c[3] == FULL_OFF_BLOCK for c in block_writes)

    def test_full_duty_does_not_overflow_off_count(self, backend):
        self._setup(backend)
        backend.write(left_signed=1.0, right_signed=0.0)
        bus = backend._bus  # noqa: SLF001
        # round(1.0 * 4095) = 4095 = 0x0FFF → off_l=0xFF, off_h=0x0F.
        # Importantly, OFF_H bit 4 stays clear (no accidental full-off).
        rpwm_set = _writes_to(bus.calls, _channel_base(0))
        assert rpwm_set == [
            ('write_i2c_block_data', ADDR, _channel_base(0), [0x00, 0x00, 0xFF, 0x0F]),
        ]
        assert (rpwm_set[0][3][3] & 0x10) == 0  # full-off bit not set

    def test_duty_clamped_above_one(self, backend):
        # Out-of-range commands shouldn't break encoding — they should clamp
        # to the same bytes as duty=1.0.
        self._setup(backend)
        backend.write(left_signed=5.0, right_signed=0.0)
        bus = backend._bus  # noqa: SLF001
        rpwm_set = _writes_to(bus.calls, _channel_base(0))
        assert rpwm_set[0][3] == [0x00, 0x00, 0xFF, 0x0F]


# ---------------------------------------------------------------------------
# cleanup() — zero everything, close bus.
# ---------------------------------------------------------------------------

class TestCleanup:
    def test_cleanup_zeroes_channels_and_closes_bus(self, backend):
        backend.setup(0, 1, 2, 3, pwm_frequency_hz=1000, gpio_chip=4)
        bus = backend._bus  # noqa: SLF001
        bus.calls.clear()
        backend.cleanup()
        block_writes = [c for c in bus.calls if c[0] == 'write_i2c_block_data']
        assert len(block_writes) == 4
        assert all(c[3] == FULL_OFF_BLOCK for c in block_writes)
        assert bus.closed is True
        assert backend._bus is None  # noqa: SLF001

    def test_cleanup_is_idempotent(self, backend):
        backend.setup(0, 1, 2, 3, pwm_frequency_hz=1000, gpio_chip=4)
        backend.cleanup()
        # Second cleanup must not raise even though the bus is already closed.
        backend.cleanup()
