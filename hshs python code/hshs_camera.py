import simple_pyspin
from simple_pyspin import Camera
import PySpin

class HshsCam:
    def __init__(self):
        try:
            self.hshs_cam = Camera()
            # self.hshs_cam.init()

        except simple_pyspin.CameraError as ex:
            raise ex
        except:
            print("failed")

    # def take_photo():
