"""
    Jason Hughes
    September 2024

    Save the config of a service

    DTC PRONTO 2024

"""

from std_msgs.msg import Header

class ServiceModule:

    def __init__(self, service : str, run : bool) -> None:
        self.service_ = service
        self.run_ = run

        self.received_ = False
        self.reading_ = None
        self.header_ = Header()

    @property
    def service(self) -> str:
        return self.service_

    @property
    def run(self) -> bool:
        return self.run_

    @property
    def received(self) -> bool:
        return self.received_

    @property
    def header(self) -> Header:
        return self.header_

    @header.setter
    def header(self, h : Header) -> None:
        self.header_ = h

    @received.setter
    def received(self, r : bool) -> None:
        self.received_ = r

    @property
    def reading(self):
        return self.reading_

    @reading.setter
    def reading(self, r):
        self.reading_ = r
