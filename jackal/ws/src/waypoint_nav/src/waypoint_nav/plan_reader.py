"""
    Jason Hughes
    January 2024
    
    Object for reading in QGC plan files
    and making them usable for navigation
"""
import json

#from pykml import parser
from typing import Tuple

from waypoint_nav.converter import LLtoUTM

class WaypointPlan:

    def __init__(self, path : str) -> None:
    
        self.local_waypoints_ = list()
        self.internal_iter_ = 0

        self.yaw_ = list()

        if path.split(".")[-1] == "kml":
            print("[WAYPOINT-NAV] Loading a Global kml from: ", path)
            with open(path, "r", encoding="utf-8") as f:
                root = parser.parse(f).getroot()

            wp_str = str(root.Document.Placemark.LineString.coordinates)
            wp_list = wp_str.split("\n")

            nu_waypoints = list()
            self.waypoints_ = list()

            # extract the waypoints and convert to float
            for entry in wp_list[:-1]:
                lon, lat = entry.split(",")[:2]
                nu_waypoints.append((float(lat), float(lon)))

            # get rid of duplicates
            self.waypoints_.append(nu_waypoints[0])
            for iter in range(len(nu_waypoints) - 1):
                if nu_waypoints[iter][0] == nu_waypoints[iter+1][0] and nu_waypoints[iter][1] == nu_waypoints[iter+1][1]:
                    continue
                else:
                    self.waypoints_.append(nu_waypoints[iter+1])

            # convert to local
            zone, e0, n0 = LLtoUTM(23, self.waypoints_[0][0], self.waypoints_[0][1])
            for wp in self.waypoints_:
                _, e, n = LLtoUTM(23, wp[0], wp[1])
                y = e0 - e 
                x = n0 - n
                self.local_waypoints_.append((x,y))
        else:
            with open(path) as f:
                data = json.load(f)

            regions = data["regions"]
            for r in regions:
                coords = r["coords"]
                xy = (coords[1], coords[0])
                self.local_waypoints_.append(xy)
            

    def __len__(self) -> int:
        return len(self.waypoints_)

    def __get_item__(self, iter : int) -> Tuple[float, float]:
        return self.waypoints_[iter]

    def getNextGPS(self) -> Tuple[float, float]:
        """ return the next waypoint """
        if self.internal_iter_ < len(self.waypoints_):
            wp = self.waypoints_[self.internal_iter_]
            self.internal_iter_ += 1
            return wp 
        else:
            return None
    
    def getNextLocal(self) -> Tuple[float, float]:
        """ return next waypoint in the local frame """
        if self.internal_iter_ < len(self.local_waypoints_):
            wp = self.local_waypoints_[self.internal_iter_]
            self.internal_iter_ += 1
            return wp
        else:
            return None

if __name__ == "__main__":
    # quick test
    pf = WaypointPlan("/home/jason/Apps/pennov.kml")
    
    print(pf.local_waypoints_)
