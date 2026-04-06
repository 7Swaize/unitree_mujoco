from abc import ABC

from .adapter import Adapter


class SensorAdapter(ABC, Adapter):
    def __init__(self) -> None:
        Adapter.__init__()