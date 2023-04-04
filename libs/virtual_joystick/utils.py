# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import struct
import time

from farm_ng.canbus import canbus_pb2
from farm_ng.canbus.packet import DASHBOARD_NODE_ID
from farm_ng.canbus.packet import Packet


class Vec2:
    """Simple container for keeping joystick coords in x & y terms.

    Defaults to a centered joystick (0,0). Clips values to range [-1.0, 1.0], as with the Amiga joystick.
    """

    def __init__(self, x: float = 0.0, y: float = 0.0) -> None:
        self.x: float = min(max(-1.0, x), 1.0)
        self.y: float = min(max(-1.0, y), 1.0)

    def __str__(self) -> str:
        return f"({self.x:0.2f}, {self.y:0.2f})"


class Timer:
    def __init__(self, period_s: float) -> None:
        self.period_s = period_s
        self.stamp = time.monotonic()

    def check(self) -> bool:
        """Check if long enough in the timer has passed."""
        stamp = time.monotonic()
        if stamp < self.stamp + self.period_s:
            return False
        missed_periods: int = int((stamp - self.stamp) // self.period_s)
        if missed_periods > 1:
            print(f"Catching up on {missed_periods} missed periods in Timer")
        self.stamp += (missed_periods + 1) * self.period_s
        return True

    def reset(self):
        """Reset the timer to start from the current time."""
        self.stamp = time.monotonic()


class ReqRepOpIds:
    NOP = 0
    READ = 1
    WRITE = 2
    STORE = 3


class ReqRepValIds:
    NOP = 0
    MAX_SPEED_LEVEL = 10
    FLIP_JOYSTICK = 11
    MAX_TURN_RATE = 20
    MIN_TURN_RATE = 21
    MAX_LIN_ACC = 22
    MAX_ANG_ACC = 23
    M10_ON = 30
    M11_ON = 31
    M12_ON = 32
    M13_ON = 33
    M14_ON = 34
    M15_ON = 35
    BATT_LO = 40
    BATT_HI = 41
    WHEEL_TRACK = 50
    WHEEL_BASELINE = 51
    WHEEL_GEAR_RATIO = 52
    WHEEL_RADIUS = 53
    PTO_CUR_DEV = 80
    PTO_CUR_RPM = 81
    PTO_MIN_RPM = 82
    PTO_MAX_RPM = 83
    PTO_DEF_RPM = 84
    PTO_GEAR_RATIO = 84
    STEERING_GAMMA = 90


class ReqRepValFmts:
    SHORT = "<h2x"
    USHORT = "<H2x"
    FLOAT = "<f"
    BOOL = "<B3x"


req_rep_val_fmt_dict = {
    ReqRepValIds.MAX_SPEED_LEVEL: ReqRepValFmts.USHORT,
    ReqRepValIds.FLIP_JOYSTICK: ReqRepValFmts.BOOL,
    ReqRepValIds.MAX_TURN_RATE: ReqRepValFmts.FLOAT,
    ReqRepValIds.MIN_TURN_RATE: ReqRepValFmts.FLOAT,
    # ReqRepValIds.MAX_LIN_ACC: ReqRepValFmts.FLOAT,
    ReqRepValIds.MAX_ANG_ACC: ReqRepValFmts.FLOAT,
    ReqRepValIds.M10_ON: ReqRepValFmts.BOOL,
    ReqRepValIds.M11_ON: ReqRepValFmts.BOOL,
    ReqRepValIds.M12_ON: ReqRepValFmts.BOOL,
    ReqRepValIds.M13_ON: ReqRepValFmts.BOOL,
    # ReqRepValIds.M14_ON: ReqRepValFmts.BOOL,
    # ReqRepValIds.M15_ON: ReqRepValFmts.BOOL,
    ReqRepValIds.BATT_LO: ReqRepValFmts.FLOAT,
    ReqRepValIds.BATT_HI: ReqRepValFmts.FLOAT,
    ReqRepValIds.WHEEL_TRACK: ReqRepValFmts.FLOAT,
    # ReqRepValIds.WHEEL_BASELINE: ReqRepValFmts.FLOAT,
    ReqRepValIds.WHEEL_GEAR_RATIO: ReqRepValFmts.FLOAT,
    ReqRepValIds.WHEEL_RADIUS: ReqRepValFmts.FLOAT,
    ReqRepValIds.PTO_CUR_DEV: ReqRepValFmts.USHORT,
    ReqRepValIds.PTO_CUR_RPM: ReqRepValFmts.FLOAT,
    ReqRepValIds.PTO_MIN_RPM: ReqRepValFmts.FLOAT,
    ReqRepValIds.PTO_MAX_RPM: ReqRepValFmts.FLOAT,
    ReqRepValIds.PTO_DEF_RPM: ReqRepValFmts.FLOAT,
    ReqRepValIds.PTO_GEAR_RATIO: ReqRepValFmts.FLOAT,
    ReqRepValIds.STEERING_GAMMA: ReqRepValFmts.FLOAT,
}


def unpack_req_rep_value(val_id: ReqRepValIds, payload: bytes):
    assert len(payload) == 4, "FarmngRepReq payload should be 4 bytes"
    (value,) = struct.unpack(req_rep_val_fmt_dict[val_id], payload)
    return value


class FarmngRepReq(Packet):
    """Supervisor request.

    farm-ng parallel to SDO protocol
    """

    cob_id_req = 0x600  # SDO command id
    cob_id_rep = 0x580  # SDO reply id

    format = "<BHx4s"

    def __init__(
        self,
        op_id=ReqRepOpIds.NOP,
        val_id=ReqRepValIds.NOP,
        success=False,
        payload=bytes(4),
    ) -> None:
        self.op_id = op_id
        self.val_id = val_id
        self.success = success
        self.payload = payload

        self.stamp_packet(time.monotonic())

    def encode(self):
        """Returns the data contained by the class encoded as CAN message data."""
        return struct.pack(
            self.format, self.op_id | (self.success << 7), self.val_id, self.payload
        )

    def decode(self, data):
        """Decodes CAN message data and populates the values of the class."""
        (op_and_s, self.val_id, self.payload) = struct.unpack(self.format, data)
        self.success = op_and_s >> 7
        self.op_id = op_and_s & ~0x80

    @classmethod
    def make_proto(
        cls,
        req: bool,
        op_id,
        val_id,
        node_id: int = DASHBOARD_NODE_ID,
        payload=bytes(4),
    ) -> canbus_pb2.RawCanbusMessage:
        """Creates a canbus_pb2.RawCanbusMessage."""
        return canbus_pb2.RawCanbusMessage(
            id=((cls.cob_id_req if req else cls.cob_id_rep) | node_id),
            data=cls(op_id=op_id, val_id=val_id, payload=payload).encode(),
        )

    def __str__(self):
        return "supervisor req OP {} VAL {} success {} payload {}".format(
            self.op_id,
            self.val_id,
            self.success,
            self.payload,
        )


class AmigaPdo2(Packet):
    """Contains a request or reply of RPM for each in individual motor (0xA - 0xD).

    Identical packet for RPDO (request) & TPDO (reply (measured)). Should be used in conjunction with AmigaRpdo1 /
    AmigaTpdo1 for auto control.

    Introduced in fw version v0.2.0
    """

    cob_id_req = 0x300  # RPDO2
    cob_id_rep = 0x280  # TPDO2

    def __init__(
        self,
        a_rpm: int = 0,
        b_rpm: int = 0,
        c_rpm: int = 0,
        d_rpm: int = 0,
    ):
        self.format = "<4h"
        self.a_rpm: int = a_rpm
        self.b_rpm: int = b_rpm
        self.c_rpm: int = c_rpm
        self.d_rpm: int = d_rpm

        self.stamp_packet(time.monotonic())

    def encode(self):
        """Returns the data contained by the class encoded as CAN message data."""
        return struct.pack(self.format, self.a_rpm, self.b_rpm, self.c_rpm, self.d_rpm)

    def decode(self, data):
        """Decodes CAN message data and populates the values of the class."""
        (self.a_rpm, self.b_rpm, self.c_rpm, self.d_rpm) = struct.unpack(
            self.format, data
        )

    def __str__(self):
        return "AMIGA PDO2 Motor RPMs | A {} B {} C {} D {}".format(
            self.a_rpm, self.b_rpm, self.c_rpm, self.d_rpm
        )
