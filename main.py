from typing import Callable

from drivebuildclient.AIExchangeService import AIExchangeService
from drivebuildclient.aiExchangeMessages_pb2 import SimulationID, VehicleID
from lxml.etree import _Element, Element

from DDController import Controller

BRIGHTNESS: float = 0.4
MAX_SPEED: float = 25
CONTROLLER = Controller(MAX_SPEED)


class AI:
    def __init__(self, service: AIExchangeService):
        self.service = service

    # The Image needs to be preprocessed
    @staticmethod
    def _preprocess(img, brightness):
        from cv2 import cvtColor, resize, COLOR_HSV2BGR, COLOR_BGR2HSV
        from numpy import array
        pil_image = img.convert('RGB')
        open_cv_image = array(pil_image)
        open_cv_image = open_cv_image[:, :, ::-1].copy()
        hsv = cvtColor(resize(open_cv_image, (280, 210)), COLOR_BGR2HSV)
        hsv[..., 2] = hsv[..., 2] * brightness
        preprocessed = cvtColor(hsv, COLOR_HSV2BGR)
        return preprocessed

    def start(self, sid: SimulationID, vid: VehicleID, dynamic_stats_callback: Callable[[], None]) -> None:
        from drivebuildclient.aiExchangeMessages_pb2 import SimStateResponse, DataRequest, Control
        from drivebuildclient.common import eprint
        from PIL import Image
        from io import BytesIO
        request = DataRequest()
        request.request_ids.append("egoFrontCamera")
        while True:
            sim_state = self.service.wait_for_simulator_request(sid, vid)
            dynamic_stats_callback()
            if sim_state == SimStateResponse.SimState.RUNNING:
                data = self.service.request_data(sid, vid, request)
                speed = data.data["egoSpeed"].speed.speed
                if data:
                    color_image = Image.open(BytesIO(data.data["egoFrontCamera"].camera.color))
                    controls = CONTROLLER.getControl(AI._preprocess(color_image, BRIGHTNESS), speed)
                    control = Control()
                    control.avCommand.steer = controls.steering
                    control.avCommand.accelerate = controls.throttle
                    self.service.control(sid, vid, control)
                else:
                    eprint("The request for data returned None.")
            else:
                break

    @staticmethod
    def add_data_requests(parent: _Element) -> None:
        speed_node = Element("speed")
        speed_node.set("id", "egoSpeed")
        parent.append(speed_node)
