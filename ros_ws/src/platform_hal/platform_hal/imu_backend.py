"""I²C backend abstraction for imu_driver.

Two implementations:

  - Ism330Backend — reads the ISM330DHCX 6-DoF IMU over I²C using smbus2.
    Lazy-imports smbus2 so the package builds on hosts without it.
  - MockImuBackend — returns injectable synthetic samples for unit and
    launch_testing tests. No hardware access.

The imu_driver selects via the `imu_backend` parameter. The `read()`
return convention is ((ax, ay, az) in m/s², (gx, gy, gz) in rad/s);
backends raise OSError on I²C fault, which the node catches to trigger
the SRS-HAL-002-S01 halt-publication-on-fault behavior.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod

AccelTriple = tuple[float, float, float]
GyroTriple = tuple[float, float, float]


class ImuBackend(ABC):
    @abstractmethod
    def setup(self, i2c_bus: int, i2c_address: int) -> None:
        ...

    @abstractmethod
    def read(self) -> tuple[AccelTriple, GyroTriple]:
        """Read one accel+gyro sample. May raise OSError on I²C fault."""
        ...

    @abstractmethod
    def cleanup(self) -> None:
        ...


class Ism330Backend(ImuBackend):
    """ISM330DHCX 6-DoF IMU over I²C. Register map per ST datasheet rev 3.

    Configures accel + gyro at ODR = 104 Hz (nearest ISM330DHCX rate to the
    driver's 100 Hz publish target), accel FS = ±4 g, gyro FS = ±500 dps.
    """

    REG_WHO_AM_I = 0x0F
    REG_CTRL1_XL = 0x10
    REG_CTRL2_G = 0x11
    REG_OUTX_L_G = 0x22
    REG_OUTX_L_A = 0x28

    WHO_AM_I = 0x6B

    # Datasheet §1 (Mechanical characteristics).
    ACCEL_SENSITIVITY_MG_LSB = 0.122   # at ±4 g FS
    GYRO_SENSITIVITY_MDPS_LSB = 17.5   # at ±500 dps FS
    G_MPS2 = 9.80665
    _DEG2RAD = math.pi / 180.0

    def __init__(self) -> None:
        import smbus2  # lazy — package must build without smbus2 installed
        self._smbus2 = smbus2
        self._bus = None
        self._address = 0

    def setup(self, i2c_bus: int, i2c_address: int) -> None:
        self._bus = self._smbus2.SMBus(i2c_bus)
        self._address = i2c_address
        who = self._bus.read_byte_data(self._address, self.REG_WHO_AM_I)
        if who != self.WHO_AM_I:
            raise RuntimeError(
                f'ISM330DHCX WHO_AM_I mismatch on i2c bus {i2c_bus} @ '
                f'0x{i2c_address:02x}: got 0x{who:02x}, expected 0x6B'
            )
        # CTRL1_XL (0x10): ODR_XL=104 Hz [7:4]=0100, FS_XL=±4 g [3:2]=10.
        self._bus.write_byte_data(self._address, self.REG_CTRL1_XL, 0x48)
        # CTRL2_G  (0x11): ODR_G=104 Hz,        FS_G=±500 dps [3:2]=01.
        self._bus.write_byte_data(self._address, self.REG_CTRL2_G, 0x44)

    def read(self) -> tuple[AccelTriple, GyroTriple]:
        g = self._bus.read_i2c_block_data(self._address, self.REG_OUTX_L_G, 6)
        a = self._bus.read_i2c_block_data(self._address, self.REG_OUTX_L_A, 6)
        k_a = self.ACCEL_SENSITIVITY_MG_LSB * 1e-3 * self.G_MPS2
        k_g = self.GYRO_SENSITIVITY_MDPS_LSB * 1e-3 * self._DEG2RAD
        accel = (
            self._s16(a[0], a[1]) * k_a,
            self._s16(a[2], a[3]) * k_a,
            self._s16(a[4], a[5]) * k_a,
        )
        gyro = (
            self._s16(g[0], g[1]) * k_g,
            self._s16(g[2], g[3]) * k_g,
            self._s16(g[4], g[5]) * k_g,
        )
        return accel, gyro

    @staticmethod
    def _s16(lo: int, hi: int) -> int:
        v = (hi << 8) | lo
        return v - 0x10000 if v & 0x8000 else v

    def cleanup(self) -> None:
        if self._bus is not None:
            self._bus.close()
            self._bus = None


class MockImuBackend(ImuBackend):
    """Synthetic IMU source for tests. Default sample is 'rover at rest, level':
    +1g on z, zero gyro. `set_sample()` and `fail_next_read()` let tests drive
    specific scenarios (tilt, I²C fault, etc.)."""

    def __init__(self) -> None:
        self._accel: AccelTriple = (0.0, 0.0, Ism330Backend.G_MPS2)
        self._gyro: GyroTriple = (0.0, 0.0, 0.0)
        self._fail_next = False

    def set_sample(self, accel: AccelTriple, gyro: GyroTriple) -> None:
        self._accel = tuple(accel)  # type: ignore[assignment]
        self._gyro = tuple(gyro)    # type: ignore[assignment]

    def fail_next_read(self) -> None:
        self._fail_next = True

    def setup(self, *_args, **_kwargs) -> None:
        return

    def read(self) -> tuple[AccelTriple, GyroTriple]:
        if self._fail_next:
            self._fail_next = False
            raise OSError('mock I²C fault')
        return self._accel, self._gyro

    def cleanup(self) -> None:
        return


def make_imu_backend(name: str) -> ImuBackend:
    if name == 'mock':
        return MockImuBackend()
    if name == 'ism330':
        return Ism330Backend()
    raise ValueError(f"unknown imu_backend: {name!r} (expected 'ism330' or 'mock')")
