import os
import numpy as np
import xml.etree.ElementTree as ET
from enum import Enum
from pathlib import Path

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
    BASE_DIR = Path(__file__).resolve().parent.parent
    GENERATED_SCENE_PATH = (BASE_DIR / "resources/scene/scene.xml").resolve()
    BASE_SCENE_PATH = (BASE_DIR / "resources/scene/base.xml").resolve()

    def __init__(self) -> None:
        self._gen_scene = ET.parse(TerrainGenerator.GENERATED_SCENE_PATH)
        self._base_scene = ET.parse(TerrainGenerator.BASE_SCENE_PATH)
        self._reset_roots()

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

    def add_aruco_marker(self,
                         position=[1.0, 0.0, 0.0],
                         euler=[0.0, 0.0, 0.0],
                         size=[0.1, 0.1, 0.1],
                         marker_num=0) -> None:
        if marker_num < 0 or marker_num > 20:
            raise ValueError("Parameter `marker_num` must be between 0 and 20 inclusive.")

        tex = ET.SubElement(self._asset, "texture")
        tex.attrib["name"] = "aruco_tex_7x7_" + str(marker_num)
        tex.attrib["type"] = "2d"
        tex.attrib["file"] = "markers/marker_7x7_" + str(marker_num) + ".png"

        mat = ET.SubElement(self._asset, "material")
        mat.attrib["name"] = "aruco_mat_7x7_" + str(marker_num)
        mat.attrib["texture"] = "aruco_tex_7x7_" + str(marker_num)

        geo = ET.SubElement(self._worldbody, "geom")
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["type"] = GeometryType.BOX.value
        geo.attrib["size"] = list_to_str(0.5 * np.array(size)) # half size of box for mujoco
        geo.attrib["quat"] = list_to_str(euler_to_quat(euler[0], euler[1], euler[2]))
        geo.attrib["material"] = "aruco_mat_7x7_" + str(marker_num)
    

    def save(self) -> None:
        ET.indent(self._root, space="    ", level=0)
        self._gen_scene.write(TerrainGenerator.GENERATED_SCENE_PATH)

    def reset_to_base(self) -> None:
        self._base_scene.write(TerrainGenerator.GENERATED_SCENE_PATH, encoding="utf-8", xml_declaration=True)        
        self._gen_scene = ET.parse(TerrainGenerator.GENERATED_SCENE_PATH)
        self._reset_roots()

    def load_scene_from_path(self, path: str) -> None:
        if not os.path.isfile(path):
            raise FileNotFoundError("XML file at specified path was not found.")
        
        self._gen_scene = ET.parse(path)
        self._gen_scene.write(TerrainGenerator.GENERATED_SCENE_PATH, encoding="utf-8", xml_declaration=True)
        self._reset_roots()

    def export_scene_to_directory(self, path: str) -> None:
        if not os.path.exists(path):
            raise FileNotFoundError("The specified path was not found.")
        
        self._gen_scene.write(os.path.join(path, "scene.xml"), encoding="utf-8", xml_declaration=True)

    def _reset_roots(self) -> None:
        self._root = self._gen_scene.getroot()
        self._worldbody = self._root.find("worldbody")
        self._asset = self._root.find("asset")
