import ctypes
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


class _CFrameInfo(ctypes.Structure):
    _fields_ = [
        ("timestamp", ctypes.c_int64),
        ("frame_number", ctypes.c_int32),
        ("rigid_body_count", ctypes.c_int32),
        ("frame_params", ctypes.c_int32),
        ("timecode", ctypes.c_uint32),
        ("timecode_subframe", ctypes.c_uint32),
        ("latency", ctypes.c_float),
    ]


class _CRigidBody(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_int32),
        ("has_extend", ctypes.c_int32),
        ("params", ctypes.c_int32),

        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),

        ("qx", ctypes.c_float),
        ("qy", ctypes.c_float),
        ("qz", ctypes.c_float),
        ("qw", ctypes.c_float),

        ("mean_error", ctypes.c_float),

        ("roll", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("yaw", ctypes.c_float),

        ("roll_vel", ctypes.c_float),
        ("pitch_vel", ctypes.c_float),
        ("yaw_vel", ctypes.c_float),

        ("roll_acc", ctypes.c_float),
        ("pitch_acc", ctypes.c_float),
        ("yaw_acc", ctypes.c_float),

        ("vx", ctypes.c_float),
        ("vy", ctypes.c_float),
        ("vz", ctypes.c_float),
        ("speed", ctypes.c_float),

        ("ax", ctypes.c_float),
        ("ay", ctypes.c_float),
        ("az", ctypes.c_float),
        ("acc", ctypes.c_float),
    ]


@dataclass(frozen=True)
class FrameInfo:
    timestamp: int
    frame_number: int
    rigid_body_count: int
    frame_params: int
    timecode: int
    timecode_subframe: int
    latency: float


@dataclass(frozen=True)
class RigidBody:
    id: int
    has_extend: bool
    params: int

    x: float
    y: float
    z: float

    qx: float
    qy: float
    qz: float
    qw: float

    mean_error: float

    roll: float
    pitch: float
    yaw: float

    roll_vel: float
    pitch_vel: float
    yaw_vel: float

    roll_acc: float
    pitch_acc: float
    yaw_acc: float

    vx: float
    vy: float
    vz: float
    speed: float

    ax: float
    ay: float
    az: float
    acc: float

    @property
    def position_m(self) -> Tuple[float, float, float]:
        return self.x * 0.001, self.y * 0.001, self.z * 0.001

    @property
    def velocity_mps(self) -> Tuple[float, float, float]:
        return self.vx * 0.001, self.vy * 0.001, self.vz * 0.001

    @property
    def acceleration_mps2(self) -> Tuple[float, float, float]:
        return self.ax * 0.001, self.ay * 0.001, self.az * 0.001


class NokovClient:
    def __init__(
        self,
        server_ip: str,
        library_path: Optional[str] = None,
        max_bodies: int = 1000,
    ):
        self.server_ip = server_ip
        self.max_bodies = max_bodies

        if library_path is None:
            library_path = str(
                Path(__file__).resolve().parent / "libnokov_pybridge.so"
            )

        self._lib = ctypes.CDLL(library_path)

        self._lib.nokov_connect.argtypes = [ctypes.c_char_p]
        self._lib.nokov_connect.restype = ctypes.c_int

        self._lib.nokov_get_latest.argtypes = [
            ctypes.POINTER(_CFrameInfo),
            ctypes.POINTER(_CRigidBody),
            ctypes.c_int,
        ]
        self._lib.nokov_get_latest.restype = ctypes.c_int

        self._lib.nokov_has_frame.argtypes = []
        self._lib.nokov_has_frame.restype = ctypes.c_int

        self._lib.nokov_disconnect.argtypes = []
        self._lib.nokov_disconnect.restype = None

        self._buffer = (_CRigidBody * self.max_bodies)()
        self._connected = False

    def connect(self) -> None:
        ret = self._lib.nokov_connect(self.server_ip.encode("ascii"))
        if ret != 0:
            raise RuntimeError(f"Nokov Initialize failed, return code={ret}")
        self._connected = True

    def disconnect(self) -> None:
        if self._connected:
            self._lib.nokov_disconnect()
            self._connected = False

    def latest(self) -> Optional[Tuple[FrameInfo, List[RigidBody]]]:
        c_info = _CFrameInfo()
        count = self._lib.nokov_get_latest(
            ctypes.byref(c_info),
            self._buffer,
            self.max_bodies,
        )

        if count < 0:
            raise RuntimeError(f"nokov_get_latest failed, return code={count}")

        if self._lib.nokov_has_frame() == 0:
            return None

        info = FrameInfo(
            timestamp=int(c_info.timestamp),
            frame_number=int(c_info.frame_number),
            rigid_body_count=int(c_info.rigid_body_count),
            frame_params=int(c_info.frame_params),
            timecode=int(c_info.timecode),
            timecode_subframe=int(c_info.timecode_subframe),
            latency=float(c_info.latency),
        )

        bodies: List[RigidBody] = []
        for i in range(count):
            c = self._buffer[i]
            bodies.append(
                RigidBody(
                    id=int(c.id),
                    has_extend=bool(c.has_extend),
                    params=int(c.params),
                    x=float(c.x),
                    y=float(c.y),
                    z=float(c.z),
                    qx=float(c.qx),
                    qy=float(c.qy),
                    qz=float(c.qz),
                    qw=float(c.qw),
                    mean_error=float(c.mean_error),
                    roll=float(c.roll),
                    pitch=float(c.pitch),
                    yaw=float(c.yaw),
                    roll_vel=float(c.roll_vel),
                    pitch_vel=float(c.pitch_vel),
                    yaw_vel=float(c.yaw_vel),
                    roll_acc=float(c.roll_acc),
                    pitch_acc=float(c.pitch_acc),
                    yaw_acc=float(c.yaw_acc),
                    vx=float(c.vx),
                    vy=float(c.vy),
                    vz=float(c.vz),
                    speed=float(c.speed),
                    ax=float(c.ax),
                    ay=float(c.ay),
                    az=float(c.az),
                    acc=float(c.acc),
                )
            )

        return info, bodies

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()


def main():
    server_ip = "192.168.5.6"

    with NokovClient(server_ip) as client:
        print(f"Connected to Nokov server: {server_ip}")

        last_frame = None

        while True:
            result = client.latest()
            if result is None:
                time.sleep(0.005)
                continue

            info, bodies = result

            if info.frame_number == last_frame:
                time.sleep(0.001)
                continue

            last_frame = info.frame_number

            print(
                f"frame={info.frame_number} "
                f"bodies={info.rigid_body_count} "
                f"latency={info.latency:.6f} "
                f"timestamp={info.timestamp}"
            )

            for rb in bodies:
                x_m, y_m, z_m = rb.position_m

                msg = (
                    f"  ID={rb.id} "
                    f"pos=({x_m:.4f}, {y_m:.4f}, {z_m:.4f}) m "
                    f"quat=({rb.qx:.5f}, {rb.qy:.5f}, "
                    f"{rb.qz:.5f}, {rb.qw:.5f}) "
                    f"mean_error={rb.mean_error:.5f}"
                )

                if rb.has_extend:
                    vx, vy, vz = rb.velocity_mps
                    ax, ay, az = rb.acceleration_mps2
                    msg += (
                        f" vel=({vx:.4f}, {vy:.4f}, {vz:.4f}) m/s"
                        f" acc=({ax:.4f}, {ay:.4f}, {az:.4f}) m/s^2"
                        f" rpy=({rb.roll:.5f}, {rb.pitch:.5f}, {rb.yaw:.5f})"
                    )
                else:
                    msg += " extend=no"

                print(msg)

            print()
            time.sleep(0.001)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
