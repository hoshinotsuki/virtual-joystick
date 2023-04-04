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
import argparse
import asyncio
import os
from typing import List
from typing import Optional

import grpc
from farm_ng.canbus import canbus_pb2
from farm_ng.canbus.canbus_client import CanbusClient
from farm_ng.canbus.packet import AmigaControlState
from farm_ng.canbus.packet import AmigaTpdo1
from farm_ng.canbus.packet import make_amiga_rpdo1_proto
from farm_ng.canbus.packet import parse_amiga_tpdo1_proto
from farm_ng.oak import oak_pb2
from farm_ng.oak.camera_client import OakCameraClient
from farm_ng.service import service_pb2
from farm_ng.service.service_client import ClientConfig
from turbojpeg import TurboJPEG
from virtual_joystick.joystick import VirtualJoystickWidget
from virtual_joystick.utils import DASHBOARD_NODE_ID
from virtual_joystick.utils import FarmngRepReq
from virtual_joystick.utils import req_rep_val_fmt_dict
from virtual_joystick.utils import ReqRepOpIds
from virtual_joystick.utils import Timer
from virtual_joystick.utils import unpack_req_rep_value

# Must come before kivy imports
os.environ["KIVY_NO_ARGS"] = "1"

from kivy.config import Config  # noreorder # noqa: E402

Config.set("graphics", "resizable", False)
Config.set("graphics", "width", "1280")
Config.set("graphics", "height", "800")
Config.set("graphics", "fullscreen", "false")
Config.set("input", "mouse", "mouse,disable_on_activity")
Config.set("kivy", "keyboard_mode", "systemanddock")

from kivy.app import App  # noqa: E402
from kivy.graphics.texture import Texture  # noqa: E402
from kivy.lang.builder import Builder  # noqa: E402
from kivy.properties import StringProperty  # noqa: E402


class VirtualJoystickApp(App):
    # For kivy labels
    amiga_state = StringProperty("???")
    amiga_speed = StringProperty("???")
    amiga_rate = StringProperty("???")
    read_value = StringProperty("???")
    read_success = StringProperty("???")

    def __init__(
        self, address: str, camera_port: int, canbus_port: int, stream_every_n: int
    ) -> None:
        super().__init__()
        self.address: str = address
        self.camera_port: int = camera_port
        self.canbus_port: int = canbus_port
        self.stream_every_n: int = stream_every_n

        # Received values
        self.amiga_tpdo1: AmigaTpdo1 = AmigaTpdo1()

        # Parameters
        self.max_speed: float = 1.0
        self.max_angular_rate: float = 1.0

        self.image_decoder = TurboJPEG()

        self.async_tasks: List[asyncio.Task] = []

        # For testing read / write protocol
        # self.read_value = "???"
        # self.read_success = "???"
        self.send_req_timer: None | Timer = None

    def build(self):
        return Builder.load_file("res/main.kv")

    def on_exit_btn(self) -> None:
        """Kills the running kivy application."""
        App.get_running_app().stop()

    async def app_func(self):
        async def run_wrapper() -> None:
            # we don't actually need to set asyncio as the lib because it is
            # the default, but it doesn't hurt to be explicit
            await self.async_run(async_lib="asyncio")
            for task in self.async_tasks:
                task.cancel()

        # configure the camera client
        camera_config: ClientConfig = ClientConfig(
            address=self.address, port=self.camera_port
        )
        camera_client: OakCameraClient = OakCameraClient(camera_config)

        # configure the canbus client
        canbus_config: ClientConfig = ClientConfig(
            address=self.address, port=self.canbus_port
        )
        canbus_client: CanbusClient = CanbusClient(canbus_config)

        # Camera task(s)
        self.async_tasks.append(
            asyncio.ensure_future(self.stream_camera(camera_client))
        )

        # Canbus task(s)
        self.async_tasks.append(
            asyncio.ensure_future(self.stream_canbus(canbus_client))
        )
        self.async_tasks.append(
            asyncio.ensure_future(self.send_can_msgs(canbus_client))
        )

        return await asyncio.gather(run_wrapper(), *self.async_tasks)

    async def stream_canbus(self, client: CanbusClient) -> None:
        """This task:

        - listens to the canbus client's stream
        - filters for AmigaTpdo1 messages
        - extracts useful values from AmigaTpdo1 messages
        """
        while self.root is None:
            await asyncio.sleep(0.01)

        response_stream = None

        while True:
            # check the state of the service
            state = await client.get_state()

            if state.value not in [
                service_pb2.ServiceState.IDLE,
                service_pb2.ServiceState.RUNNING,
            ]:
                if response_stream is not None:
                    response_stream.cancel()
                    response_stream = None

                print("Canbus service is not streaming or ready to stream")
                await asyncio.sleep(0.1)
                continue

            if (
                response_stream is None
                and state.value != service_pb2.ServiceState.UNAVAILABLE
            ):
                # get the streaming object
                response_stream = client.stream()

            try:
                # try/except so app doesn't crash on killed service
                response: canbus_pb2.StreamCanbusReply = await response_stream.read()
                assert response and response != grpc.aio.EOF, "End of stream"
            except Exception as e:
                print(e)
                response_stream.cancel()
                response_stream = None
                continue

            for proto in response.messages.messages:
                amiga_tpdo1: Optional[AmigaTpdo1] = parse_amiga_tpdo1_proto(proto)
                if amiga_tpdo1:
                    # Store the value for possible other uses
                    self.amiga_tpdo1 = amiga_tpdo1

                    # Update the Label values as they are received
                    self.amiga_state = AmigaControlState(amiga_tpdo1.state).name[6:]
                    self.amiga_speed = str(amiga_tpdo1.meas_speed)
                    self.amiga_rate = str(amiga_tpdo1.meas_ang_rate)

                elif proto.id == FarmngRepReq.cob_id_rep + DASHBOARD_NODE_ID:
                    rep: FarmngRepReq = FarmngRepReq.from_can_data(
                        proto.data, proto.stamp
                    )
                    if int(rep.val_id) == int(self.root.ids.read_val_sld.value):
                        self.read_success = str(bool(rep.success))
                        if rep.val_id in req_rep_val_fmt_dict:
                            self.read_value = str(
                                unpack_req_rep_value(rep.val_id, rep.payload)
                            )
                    self.send_req_timer = None

    async def stream_camera(self, client: OakCameraClient) -> None:
        """This task listens to the camera client's stream and populates the tabbed panel with all 4 image streams
        from the oak camera."""
        while self.root is None:
            await asyncio.sleep(0.01)

        response_stream = None

        while True:
            # check the state of the service
            state = await client.get_state()

            if state.value not in [
                service_pb2.ServiceState.IDLE,
                service_pb2.ServiceState.RUNNING,
            ]:
                # Cancel existing stream, if it exists
                if response_stream is not None:
                    response_stream.cancel()
                    response_stream = None
                # print("Camera service is not streaming or ready to stream")
                await asyncio.sleep(0.1)
                continue

            # Create the stream
            if response_stream is None:
                response_stream = client.stream_frames(every_n=1)

            try:
                # try/except so app doesn't crash on killed service
                response: oak_pb2.StreamFramesReply = await response_stream.read()
                assert response and response != grpc.aio.EOF, "End of stream"
            except Exception as e:
                print(e)
                response_stream.cancel()
                response_stream = None
                continue

            # get the sync frame
            frame: oak_pb2.OakSyncFrame = response.frame

            # get image and show
            for view_name in ["rgb", "disparity", "left", "right"]:
                # Skip if view_name was not included in frame
                try:
                    # Decode the image and render it in the correct kivy texture
                    img = self.image_decoder.decode(
                        getattr(frame, view_name).image_data
                    )
                    texture = Texture.create(
                        size=(img.shape[1], img.shape[0]), icolorfmt="bgr"
                    )
                    texture.flip_vertical()
                    texture.blit_buffer(
                        img.tobytes(),
                        colorfmt="bgr",
                        bufferfmt="ubyte",
                        mipmap_generation=False,
                    )
                    self.root.ids[view_name].texture = texture
                except Exception as e:
                    print(e)

    async def send_can_msgs(self, client: CanbusClient) -> None:
        """This task ensures the canbus client sendCanbusMessage method has the pose_generator it will use to send
        messages on the CAN bus to control the Amiga robot."""
        while self.root is None:
            await asyncio.sleep(0.01)

        response_stream = None
        while True:
            # check the state of the service
            state = await client.get_state()

            # Wait for a running CAN bus service
            if state.value != service_pb2.ServiceState.RUNNING:
                # Cancel existing stream, if it exists
                if response_stream is not None:
                    response_stream.cancel()
                    response_stream = None
                print("Waiting for running canbus service...")
                await asyncio.sleep(0.1)
                continue

            if response_stream is None:
                print("Start sending CAN messages")
                response_stream = client.stub.sendCanbusMessage(self.pose_generator())

            try:
                async for response in response_stream:
                    # Sit in this loop and wait until canbus service reports back it is not sending
                    assert response.success
            except Exception as e:
                print(e)
                response_stream.cancel()
                response_stream = None
                continue

            await asyncio.sleep(0.1)

    async def pose_generator(self, period: float = 0.02):
        """The pose generator yields an AmigaRpdo1 (auto control command) for the canbus client to send on the bus
        at the specified period (recommended 50hz) based on the onscreen joystick position."""
        while self.root is None:
            await asyncio.sleep(0.01)

        joystick: VirtualJoystickWidget = self.root.ids["joystick"]
        while True:
            msg: canbus_pb2.RawCanbusMessage = make_amiga_rpdo1_proto(
                state_req=AmigaControlState.STATE_AUTO_ACTIVE,
                cmd_speed=self.max_speed * joystick.joystick_pose.y,
                cmd_ang_rate=self.max_angular_rate * -joystick.joystick_pose.x,
            )
            yield canbus_pb2.SendCanbusMessageRequest(message=msg)
            if self.send_req_timer and self.send_req_timer.check():
                yield canbus_pb2.SendCanbusMessageRequest(
                    message=FarmngRepReq.make_proto(
                        req=True,
                        op_id=ReqRepOpIds.READ,
                        val_id=int(self.root.ids.read_val_sld.value),
                    )
                )

            await asyncio.sleep(period)

    # For testing read / write protocol
    def trigger_read_req(self):
        self.send_req_timer = Timer(period_s=0.5)

    # For testing read / write protocol
    def val_slider_move(self):
        self.read_value = "???"
        self.read_success = "???"
        self.send_req_timer = None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="virtual-joystick")
    parser.add_argument(
        "--address", type=str, default="localhost", help="The server address"
    )
    parser.add_argument(
        "--camera-port",
        type=int,
        required=True,
        help="The grpc port where the camera service is running.",
    )
    parser.add_argument(
        "--canbus-port",
        type=int,
        required=True,
        help="The grpc port where the canbus service is running.",
    )
    parser.add_argument(
        "--stream-every-n",
        type=int,
        default=1,
        help="Streaming frequency (used to skip frames)",
    )

    args = parser.parse_args()

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(
            VirtualJoystickApp(
                args.address, args.camera_port, args.canbus_port, args.stream_every_n
            ).app_func()
        )
    except asyncio.CancelledError:
        pass
    loop.close()
