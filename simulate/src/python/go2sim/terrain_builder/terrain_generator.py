import cv2
import numpy as np
import xml.etree.ElementTree as ET
from enum import Enum

from .helpers import (
    euler_to_quat,
    list_to_str,
    rot2d,
    rot3d
)


class GeometryType(str, Enum):
    PLANE = "plane"
    SPHERE = "sphere"
    CAPSULE = "capsule"
    ELLIPSOID = "ellipsoid"
    CYLINDER = "cylinder"
    BOX = "box"  


class TerrainGenerator:
    ROBOT = "go2"
    GENERATED_SCENE_PATH = "../resources/scene/scene.xml"
    BASE_SCENE_PATH = "../resources/scene/base.xml"

    def __init__(self) -> None:
        self._gen_scene = ET.parse(TerrainGenerator.GENERATED_SCENE_PATH)
        self._base_scene = ET.parse(TerrainGenerator.BASE_SCENE_PATH)

        self._root = self._gen_scene.getroot()
        self._worldbody = self._root.find("worldbody")
        self._asset = self._root.find("asset")

    def add_geometry(self,
                 position=[1.0, 0.0, 0.0],
                 euler=[0.0, 0.0, 0.0],
                 size=[0.1, 0.1, 0.1],
                 geo_type=GeometryType.BOX) -> None:
        geo = ET.SubElement(self._worldbody, "geom")
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["type"]  = geo_type.value # geo_type supports "plane", "sphere", "capsule", "ellipsoid", "cylinder", "box"
        geo.attrib["size"] = list_to_str(0.5 * np.array(size)) # half size of box for mujoco
        geo.attrib["quat"] = list_to_str(euler_to_quat(euler[0], euler[1], euler[2]))

    def add_stairs(self,
                   init_pos=[1.0, 0.0, 0.0],
                   yaw=0.0,
                   width=0.2,
                   height=0.15,
                   length=1.5,
                   stair_nums=10) -> None:
        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.add_geometry(
                [x + init_pos[0], y + init_pos[1], local_pos[2]],
                [0.0, 0.0, yaw],
                [width, length, height],
                GeometryType.BOX
            )
    
    def add_suspend_stairs(self,
                           init_pos=[1.0, 0.0, 0.0],
                           yaw=1.0,
                           width=0.2,
                           height=0.15,
                           length=1.5,
                           gap=0.1,
                           stair_nums=10) -> None:
        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.add_geometry(
                [x + init_pos[0], y + init_pos[1], local_pos[2]],
                [0.0, 0.0, yaw],
                [width, length, abs(height - gap)],
                GeometryType.BOX
            )

    def add_rough_ground(self,
                         init_pos=[1.0, 0.0, 0.0],
                         euler=[0.0, -0.0, 0.0],
                         nums=[10, 10],
                         box_size=[0.5, 0.5, 0.5],
                         box_euler=[0.0, 0.0, 0.0],
                         separation=[0.2, 0.2],
                         box_size_rand=[0.05, 0.05, 0.05],
                         box_euler_rand=[0.2, 0.2, 0.2],
                         separation_rand=[0.05, 0.05]) -> None:
        local_pos = [0.0, 0.0, -0.5 * box_size[2]]
        new_separation = np.array(separation) + np.array(
            separation_rand) * np.random.uniform(-1.0, 1.0, 2)
        for i in range(nums[0]):
            local_pos[0] += new_separation[0]
            local_pos[1] = 0.0
            for j in range(nums[1]):
                new_box_size = np.array(box_size) + np.array(
                    box_size_rand) * np.random.uniform(-1.0, 1.0, 3)
                new_box_euler = np.array(box_euler) + np.array(
                    box_euler_rand) * np.random.uniform(-1.0, 1.0, 3)
                new_separation = np.array(separation) + np.array(
                    separation_rand) * np.random.uniform(-1.0, 1.0, 2)
                
                local_pos[1] += new_separation[1]
                pos = rot3d(local_pos, euler) + np.array(init_pos)
                self.add_geometry(
                    pos, 
                    new_box_euler,
                    new_box_size,
                    GeometryType.BOX
                )

    def save(self) -> None:
        ET.indent(self._root, space="    ", level=0)
        self._gen_scene.write(TerrainGenerator.GENERATED_SCENE_PATH)


if __name__ == "__main__":
    tg = TerrainGenerator()

    # Box obstacle
    tg.AddBox(position=[1.5, 0.0, 0.1], euler=[0, 0, 0.0], size=[1, 1.5, 0.2])
    
    # Geometry obstacle
    # geo_type supports "plane", "sphere", "capsule", "ellipsoid", "cylinder", "box"
    tg.AddGeometry(position=[1.5, 0.0, 0.25], euler=[0, 0, 0.0], size=[1.0,0.5,0.5],geo_type="cylinder")

    # Slope
    tg.AddBox(position=[2.0, 2.0, 0.5],
              euler=[0.0, -0.5, 0.0],
              size=[3, 1.5, 0.1])

    # Stairs
    tg.AddStairs(init_pos=[1.0, 4.0, 0.0], yaw=0.0)

    # Suspend stairs
    tg.AddSuspendStairs(init_pos=[1.0, 6.0, 0.0], yaw=0.0)

    # Rough ground
    tg.AddRoughGround(init_pos=[-2.5, 5.0, 0.0],
                      euler=[0, 0, 0.0],
                      nums=[10, 8])

    tg.save()
