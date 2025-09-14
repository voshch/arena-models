from __future__ import annotations

import os
import typing

import bpy

from .CoordinateSystem import CoordinateSystem


class _ModelConverterExt(typing.Protocol):
    @classmethod
    def coordinates(cls) -> CoordinateSystem:
        ...

    @classmethod
    def load(cls, path: str) -> None:
        ...

    @classmethod
    def save(cls, path: str) -> None:
        ...

    @classmethod
    def inline(cls, coordinates: CoordinateSystem, load: typing.Callable, save: typing.Callable) -> typing.Type[_ModelConverterExt]:
        class Impl(cls):
            @classmethod
            def coordinates(cls) -> CoordinateSystem:
                return coordinates

            @classmethod
            def load(cls, path: str) -> None:
                load(filepath=path)

            @classmethod
            def save(cls, path: str) -> None:
                save(filepath=path)

        return Impl


class ModelConverter:
    _reset: bool
    _coordinates: CoordinateSystem

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._reset:
            bpy.ops.wm.read_factory_settings(use_empty=True)

    __exts: typing.ClassVar[dict[str, _ModelConverterExt]] = {}

    @classmethod
    def register(cls, *ext: str):
        def decorator(subcls: typing.Type[_ModelConverterExt]):
            for e in ext:
                cls.__exts[e.lower()] = subcls
        return decorator

    def transform_coordinates(self, coords: CoordinateSystem) -> None:
        x, y, z = self._coordinates.get_transformation_to(coords)
        bpy.ops.object.select_all(action='SELECT')
        try:
            bpy.ops.transform.rotate(value=x, orient_axis='X')
            bpy.ops.transform.rotate(value=y, orient_axis='Y')
            bpy.ops.transform.rotate(value=z, orient_axis='Z')
            self._coordinates = coords
        finally:
            bpy.ops.object.select_all(action='DESELECT')

    def __init__(self, *, reset: bool = True):
        self._reset: bool = reset
        self._coordinates: CoordinateSystem = CoordinateSystem.default()

    def load(self, path: str) -> None:
        ext = os.path.splitext(path)[-1].lower().strip(".")
        if ext in self.__exts:
            ext_cls = self.__exts[ext]
            ext_cls.load(path)
            self._coordinates = ext_cls.coordinates()
        else:
            raise ValueError(f"Unsupported file extension: {ext}")

    def save(self, path: str, *, ext: str | None = None) -> None:
        ext = os.path.splitext(path)[-1].lower().strip(".")
        if ext in self.__exts:
            ext_cls = self.__exts[ext]
            self.transform_coordinates(ext_cls.coordinates())
            ext_cls.save(path)
        else:
            raise ValueError(f"Unsupported file extension: {ext}")


ModelConverter.register('usd', 'usda', 'usdc', 'usdz')(_ModelConverterExt.inline(
    CoordinateSystem('X+', 'Y+', 'Z+'),
    bpy.ops.wm.usd_import,
    bpy.ops.wm.usd_export)
)

ModelConverter.register('obj')(_ModelConverterExt.inline(
    CoordinateSystem.default(),
    bpy.ops.wm.obj_import,
    bpy.ops.wm.obj_export)
)

ModelConverter.register('fbx')(_ModelConverterExt.inline(
    CoordinateSystem.default(),
    bpy.ops.import_scene.fbx,
    bpy.ops.export_scene.fbx)
)

ModelConverter.register('dae')(_ModelConverterExt.inline(
    CoordinateSystem.default(),
    bpy.ops.wm.collada_import,
    bpy.ops.wm.collada_export)
)


__all__ = ['ModelConverter']
