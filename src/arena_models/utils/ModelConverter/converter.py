from __future__ import annotations

import io
import os
import typing

import bpy
import mathutils

from arena_models.utils.geom import BoundingBox

from ..CoordinateSystem import CoordinateSystem
from ..io_utils import capture_all_output

from . import ModelFormat


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
        # Create temporary files to act as a bridge
        self.__ctx = capture_all_output(self._stdout, self._stderr)
        self.__ctx.__enter__()
        if self._reset:
            bpy.ops.wm.read_factory_settings(use_empty=True)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.__ctx.__exit__(exc_type, exc_val, exc_tb)

    __exts: typing.ClassVar[dict[ModelFormat, _ModelConverterExt]] = {}

    @classmethod
    def register(cls, *ext: ModelFormat):
        def decorator(subcls: typing.Type[_ModelConverterExt]):
            for e in ext:
                cls.__exts[e.lower()] = subcls
        return decorator

    @classmethod
    def exts(cls) -> tuple[str, ...]:
        return tuple(cls.__exts.keys())

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
        self._stdout = io.StringIO()
        self._stderr = io.StringIO()

    @property
    def stdout(self) -> str:
        return self._stdout.getvalue()

    @property
    def stderr(self) -> str:
        return self._stderr.getvalue()

    def load(self, path: str) -> None:
        ext = os.path.splitext(path)[-1].lower().strip(".")
        if ext in self.__exts:
            ext_cls = self.__exts[ext]
            ext_cls.load(path)
            self._coordinates = ext_cls.coordinates()
        else:
            raise ValueError(f"Unsupported file extension: {ext}")

    def rectify(self):
        """Rectify the model's orientation and position (make it upright and floor-touching).
        """

        self.transform_coordinates(CoordinateSystem.default())
        bbox = self.bounding_box()
        translation = mathutils.Vector((
            0.0,
            0.0,
            -bbox.min_z,
        ))
        bpy.ops.object.select_all(action='SELECT')
        try:
            bpy.ops.transform.translate(value=translation)
        finally:
            bpy.ops.object.select_all(action='DESELECT')

    def bounding_box(self) -> BoundingBox:
        coords = []
        for obj in bpy.context.scene.objects:
            if obj.type == 'MESH':
                for corner in obj.bound_box:
                    world_corner = obj.matrix_world @ mathutils.Vector(corner)
                    coords.append(world_corner)
        if not coords:
            return BoundingBox.empty()
        min_corner = mathutils.Vector(map(min, zip(*coords)))
        max_corner = mathutils.Vector(map(max, zip(*coords)))
        return BoundingBox(((min_corner.x, max_corner.x), (min_corner.y, max_corner.y), (min_corner.z, max_corner.z)))

    def save(self, path: str, *, ext: ModelFormat | None = None) -> None:
        ext = ModelFormat(os.path.splitext(path)[-1].lower().strip("."))
        if ext in self.__exts:
            ext_cls = self.__exts[ext]
            self.transform_coordinates(ext_cls.coordinates())
            ext_cls.save(path)
        else:
            raise ValueError(f"Unsupported file extension: {ext}")


ModelConverter.register(ModelFormat.USD, ModelFormat.USDA, ModelFormat.USDC, ModelFormat.USDZ)(_ModelConverterExt.inline(
    CoordinateSystem('X+', 'Y+', 'Z+'),
    bpy.ops.wm.usd_import,
    bpy.ops.wm.usd_export)
)

ModelConverter.register(ModelFormat.OBJ)(_ModelConverterExt.inline(
    CoordinateSystem.default(),
    bpy.ops.wm.obj_import,
    bpy.ops.wm.obj_export)
)

ModelConverter.register(ModelFormat.FBX)(_ModelConverterExt.inline(
    CoordinateSystem.default(),
    bpy.ops.import_scene.fbx,
    bpy.ops.export_scene.fbx)
)

ModelConverter.register(ModelFormat.DAE)(_ModelConverterExt.inline(
    CoordinateSystem.default(),
    bpy.ops.wm.collada_import,
    bpy.ops.wm.collada_export)
)

ModelConverter.register(ModelFormat.GLB, ModelFormat.GLTF)(_ModelConverterExt.inline(
    CoordinateSystem.default(),
    bpy.ops.import_scene.gltf,
    bpy.ops.export_scene.gltf)
)


__all__ = ['ModelConverter']
