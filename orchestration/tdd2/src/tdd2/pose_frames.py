"""
    Jason Hughes
    August 2024
    Data strucutre to handle all poses frames
    local, relative, utm and gps

    DTC PRONTO 2024
"""

from geometry_msgs.msg import Pose
from tdd2.msg import TDDetection

class PoseFrames:

    def __init__(self, 
                 lat : float, 
                 lon : float,
                 easting : float,
                 northing : float,
                 local_x : float,
                 local_y : float,
                 rel_x : float,
                 rel_y : float) -> None:

        self.lat_ = lat
        self.lon_ = lon
        self.easting_ = easting 
        self.northing_ = northing
        self.local_x_ = local_x
        self.local_y_ = local_y
        self.rel_x_ = rel_x
        self.rel_y_ = rel_y

    @property
    def lat(self) -> float:
        return self.lat_

    @property
    def lon(self) -> float:
        return self.lon_

    @property
    def easting(self) -> float:
        return self.easting_

    @property
    def northing(self) -> float:
        return self.northing_

    @property 
    def x(self) -> float:
        return self.local_x_

    @property
    def y(self) -> float:
        return self.local_y_

    @property
    def rel_x(self) -> float:
        return self.rel_x_

    @property
    def rel_y(self) -> float:
        return self.rel_y_

    @property
    def local_pose(self) -> Pose:
        msg = Pose()
        msg.position.x = self.local_x_
        msg.position.y = self.local_y_
        msg.position.z = 0.0
        return msg

    @property 
    def msg(self) -> TDDetection:
        msg = TDDetection()
        msg.gps.latitude = self.lat_
        msg.gps.longitude = self.lon_
        msg.easting = self.easting_
        msg.northing = self.northing_
        msg.local.x = self.local_x_
        msg.local.y = self.local_y_
        msg.relative.x = self.rel_x_
        msg.relative.y = self.rel_y_

        return msg
