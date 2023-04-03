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
import time


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
        """This is the recommended method to use for checking the TickRepeater timer.

        Wraps the `update()` method.
        """
        stamp = time.monotonic()
        if stamp < self.stamp + self.period_s:
            return False
        missed_periods: int = (stamp - self.stamp) // self.period_s
        if missed_periods:
            print(f"Catching up on {missed_periods} missed periods in Timer")
        self.stamp += (missed_periods + 1) * self.period_s
        return True

    def reset(self):
        """Reset the timer to start from the current time."""
        self.stamp = time.monotonic()
