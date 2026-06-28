import os
import shutil
from pathlib import Path


class ResourceError(Exception):
    pass


class ResourceManager:
    SUPPORTED_IMAGE_TYPES = {".png"}

    def __init__(self, resources_root: Path) -> None:
        self._root = resources_root.resolve()
        self._root.mkdir(parents=True, exist_ok=True)

    @property
    def root(self) -> Path:
        return self._root
    
    def import_texture(self, src: str | Path, dest_folder: str = "") -> Path:
        src = Path(src).resolve()

        if not src.exists():
            raise FileNotFoundError(f"Source texture not found: {src}")
        if src.suffix.lower() not in ResourceManager.SUPPORTED_IMAGE_TYPES:
            raise ResourceError(f"Unsupported image format '{src.suffix}'. Supported formats: {ResourceManager.SUPPORTED_IMAGE_TYPES}")
        
        dest_dir = self._resolve_safe(dest_folder)
        dest_dir.mkdir(parents=True, exist_ok=True)

        dest_path = dest_dir / src.name
        shutil.copy2(src, dest_path)
        return dest_path.relative_to(self._root)
    

    def add_folder(self, relative_path: str | Path) -> None:
        self._resolve_safe(relative_path).mkdir(parents=True, exist_ok=True)


    def remove_folder(self, relative_path: str | Path, recursive: bool = True) -> None:
        folder = self._resolve_safe(relative_path)
        if not folder.exists():
            raise FileNotFoundError(f"Folder not found: '{relative_path}'")
        if not folder.is_dir():
            raise ResourceError(f"'{relative_path}' is not a directory.")
        
        if recursive:
            shutil.rmtree(folder)
        else:
            try:
                folder.rmdir()
            except OSError:
                raise ResourceError(f"Folder '{relative_path}' is not empty. User 'recursive=True' to force removal.")
            
    
    def remove_file(self, relative_path: str | Path) -> None:
        file = self._resolve_safe(relative_path)
        if not file.exists():
            raise FileNotFoundError(f"File not found: '{relative_path}'")
        if not file.is_file():
            raise ResourceError(f"'{relative_path}' is not a file.")
        
        file.unlink()


    def remove_all(self) -> None:
        if self._root.exists():
            shutil.rmtree(self._root)

        self._root.mkdir(parents=True, exist_ok=True)

    
    def list_dirs(self, relative_path: str | Path = "") -> None:
        folder = self._resolve_safe(relative_path) if relative_path else self._root
        if not folder.exists():
            raise FileNotFoundError(f"Directory not found: '{relative_path or '/'}'")
        if not folder.is_dir():
            raise NotADirectoryError(f"'{relative_path}' is not a directory.")

        print()
        print(folder.name + "/")

        self._print_recurse(folder)
    
    
    def exists(self, relative_path: str | Path = "") -> bool:
        try:
            return self._resolve_safe(relative_path).exists()
        except ResourceError:
            return False
        

    def _resolve_safe(self, relative: str | Path) -> Path:
        resolved = (self._root / relative).resolve()
        if not str(resolved).startswith(str(self._root)):
            raise ResourceError(f"Path '{relative}' resolves outside the resources root. The specified path traversal in not permitted.")
        
        return resolved

    
    def _to_scene_relative(self, resource_relative: str | Path, scene_dir: Path) -> str:
        abs_path = (self._root / resource_relative).resolve()
        return os.path.relpath(abs_path, scene_dir).replace(os.sep, "/")
    

    # Absolutely horrendous code from claude I am grateful for
    def _print_recurse(self, folder: Path, prefix: str = "") -> None:
        entries = sorted(folder.iterdir(), key=lambda e: (e.is_file(), e.name.lower()))
        for i, entry in enumerate(entries):
            is_last = i == len(entries) - 1
            connector = "└── " if is_last else "├── "
            label = entry.name + ("/" if entry.is_dir() else "")
            print(f"{prefix}{connector}{label}")

            if entry.is_dir():
                extension = "    " if is_last else "│   "
                self._print_recurse(entry, prefix + extension)