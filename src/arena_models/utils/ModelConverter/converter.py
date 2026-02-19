from __future__ import annotations

import functools
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

        # Check if model is too large in x or y and scale down if needed
        size_x = bbox.max_x - bbox.min_x
        size_y = bbox.max_y - bbox.min_y
        if size_x > 5 or size_y > 5:
            bpy.ops.object.select_all(action='SELECT')
            try:
                bpy.ops.transform.resize(value=(0.01, 0.01, 0.01))
            finally:
                bpy.ops.object.select_all(action='DESELECT')
            # Recalculate bounding box after scaling
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

    def render(self, output_path: str, *, resolution: tuple[int, int] = (512, 512), samples: int = 32) -> None:
        """Render the current scene to a thumbnail image.

        Args:
            output_path: Path where the rendered image will be saved
            resolution: Tuple of (width, height) for the output image. Default is (512, 512)
            samples: Number of samples for rendering quality. Default is 32
        """
        scene = bpy.context.scene

        # Configure render settings
        scene.render.engine = 'CYCLES'
        scene.cycles.samples = samples
        scene.render.resolution_x = resolution[0]
        scene.render.resolution_y = resolution[1]
        scene.render.resolution_percentage = 100
        scene.render.film_transparent = True
        scene.render.image_settings.file_format = 'PNG'
        scene.render.filepath = output_path

        # Get bounding box to position camera
        bbox = self.bounding_box()

        # Calculate model center and size
        center = mathutils.Vector((
            (bbox.min_x + bbox.max_x) / 2,
            (bbox.min_y + bbox.max_y) / 2,
            (bbox.min_z + bbox.max_z) / 2,
        ))
        size = max(bbox.max_x - bbox.min_x, bbox.max_y - bbox.min_y, bbox.max_z - bbox.min_z)

        # Setup camera
        camera_data = bpy.data.cameras.new(name='RenderCamera')
        camera_object = bpy.data.objects.new('RenderCamera', camera_data)
        scene.collection.objects.link(camera_object)
        scene.camera = camera_object

        # Position camera at an angle to show the model nicely
        camera_distance = size * 2.5
        camera_object.location = center + mathutils.Vector((camera_distance * 0.7, -camera_distance * 0.7, camera_distance * 0.5))

        # Point camera at model center
        direction = center - camera_object.location
        rot_quat = direction.to_track_quat('-Z', 'Y')
        camera_object.rotation_euler = rot_quat.to_euler()

        # Setup lighting
        # Sun light for general illumination
        sun_data = bpy.data.lights.new(name='SunLight', type='SUN')
        sun_data.energy = 2.0
        sun_object = bpy.data.objects.new('SunLight', sun_data)
        scene.collection.objects.link(sun_object)
        sun_object.location = center + mathutils.Vector((size, size, size * 2))
        sun_object.rotation_euler = (0.785, 0, 0.785)  # 45 degrees

        # Area light for fill lighting
        area_data = bpy.data.lights.new(name='FillLight', type='AREA')
        area_data.energy = 100.0
        area_data.size = size
        area_object = bpy.data.objects.new('FillLight', area_data)
        scene.collection.objects.link(area_object)
        area_object.location = center + mathutils.Vector((-size, -size, size))
        area_direction = center - area_object.location
        area_rot_quat = area_direction.to_track_quat('-Z', 'Y')
        area_object.rotation_euler = area_rot_quat.to_euler()

        # Render
        bpy.ops.render.render(write_still=True)

    def save(self, path: str, *, ext: ModelFormat | None = None) -> None:
        ext = ModelFormat(os.path.splitext(path)[-1].lower().strip("."))
        if ext in self.__exts:
            ext_cls = self.__exts[ext]
            # self.transform_coordinates(ext_cls.coordinates())
            ext_cls.save(path)
        else:
            raise ValueError(f"Unsupported file extension: {ext}")


ModelConverter.register(ModelFormat.USD, ModelFormat.USDA, ModelFormat.USDC, ModelFormat.USDZ)(
    _ModelConverterExt.inline(
        CoordinateSystem('X+', 'Y+', 'Z+'),
        bpy.ops.wm.usd_import,
        functools.partial(
            bpy.ops.wm.usd_export,
            export_materials=True,
            export_normals=True,
            export_uvmaps=True,
            export_animation=False,
            selected_objects_only=False,
            export_textures_mode='NEW',
            overwrite_textures=True,
            export_lights=False,
            export_cameras=False,
        )
    )
)

ModelConverter.register(ModelFormat.OBJ)(
    _ModelConverterExt.inline(
        CoordinateSystem.default(),
        bpy.ops.wm.obj_import,
        bpy.ops.wm.obj_export
    )
)

ModelConverter.register(ModelFormat.FBX)(
    _ModelConverterExt.inline(
        CoordinateSystem.default(),
        bpy.ops.import_scene.fbx,
        functools.partial(
            bpy.ops.export_scene.fbx,
            object_types={'ARMATURE', 'EMPTY', 'MESH', 'OTHER'},
            embed_textures=True,
            path_mode='COPY',
        )
    )
)

ModelConverter.register(ModelFormat.DAE)(
    _ModelConverterExt.inline(
        CoordinateSystem.default(),
        bpy.ops.wm.collada_import,
        bpy.ops.wm.collada_export
    )
)

ModelConverter.register(ModelFormat.GLB, ModelFormat.GLTF)(
    _ModelConverterExt.inline(
        CoordinateSystem.default(),
        bpy.ops.import_scene.gltf,
        bpy.ops.export_scene.gltf
    )
)


__all__ = ['ModelConverter']
