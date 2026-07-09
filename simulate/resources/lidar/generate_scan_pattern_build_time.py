import os 
import sys
import csv
import textwrap

from typing import TextIO
from pathlib import Path


GENERATED_FILE_NAME = "generated_scan_pattern"
CSV_PATH = Path(Path(__file__).resolve().parent / "mid360.csv").resolve()
OUT_HPP_PATH = Path(Path(__file__).resolve().parent.parent.parent / "src" / "cpp" / "sensors" / "data" / f"{GENERATED_FILE_NAME}.hpp").resolve()
OUT_CPP_PATH = Path(Path(__file__).resolve().parent.parent.parent / "src" / "cpp" / "sensors" / "data" / f"{GENERATED_FILE_NAME}.cpp").resolve()


class FileWriter:
    def __init__(self, file: TextIO) -> None:
        self._file: TextIO = file
        self._indent: int = 0
        self._indent_spaces_repr = 4

    def writeln(self, text: str = "") -> None:
        print(textwrap.indent(text, prefix=" " * self._indent_spaces_repr * self._indent), file=self._file)

    @property
    def indent(self):
        return self._indent
    
    @indent.setter
    def indent(self, value: int):
        if value < 0:
            raise ValueError("Indent level not allowed to be a negative number.")
        self._indent = value

    def push_indent(self) -> None:
        self._indent += 1

    def pop_indent(self) -> None:
        self._indent = max(0, self._indent - 1)



def generate():
    data = []

    with open(CSV_PATH, "r") as f:
        reader = csv.reader(f)
        next(reader, None) # Skip headers

        for row in reader:
            if not row:
                continue
            data.extend([float(val) for val in row])

    with open(OUT_HPP_PATH, "w") as hpp:
        writer = FileWriter(hpp)
        writer.writeln("#pragma once")
        writer.writeln("#include <array>")
        writer.writeln("#include <cstddef>")
        writer.writeln()

        writer.writeln("namespace lidar_data {")
        writer.push_indent()
        
        writer.writeln(f"inline constexpr std::size_t TotalElements = {len(data)};")
        writer.writeln(f"inline constexpr std::size_t TotalRows = {len(data) // 3};")
        writer.writeln(f"alignas(64) extern const std::array<float, {len(data)}> Mid360ScanPatternData;")
        
        writer.pop_indent()
        writer.writeln("}")


    with open(OUT_CPP_PATH, "w") as cpp:
        writer = FileWriter(cpp)
        writer.writeln(f'#include "{GENERATED_FILE_NAME}.hpp"')
        writer.writeln()

        writer.writeln("namespace lidar_data {")
        writer.push_indent()

        writer.writeln(f"alignas(64) extern const std::array<float, {len(data)}> Mid360ScanPatternData = {{")
        writer.push_indent()
        
        chunk_size = 12
        for i in range(0, len(data), chunk_size):
            chunk = data[i:i+chunk_size]
            line_str = ", ".join(f"{x}f" for x in chunk)
            writer.writeln(f"{line_str},")

        writer.pop_indent()
        writer.writeln("}")
        writer.pop_indent()
        writer.writeln("}")
        

if __name__ == "__main__":
    generate()