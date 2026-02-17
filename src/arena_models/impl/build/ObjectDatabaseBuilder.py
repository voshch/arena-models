from __future__ import annotations

import enum
import json
import os
import typing
from pathlib import Path

import attrs

from arena_models.impl import Annotation, AssetType, convert_list_str
from arena_models.utils.geom import BoundingBox
from arena_models.utils.logging import get_logger
from arena_models.utils.ModelConverter import ModelFormat
from arena_models.utils.ModelConverter.UsdBaker import UsdBaker

from . import DatabaseBuilder, OptionRegistry

logger = get_logger('build.object')


@attrs.define
class ObjectAnnotation(Annotation):
    class Face(str, enum.Enum):
        POS_X = "+x"
        NEG_X = "-x"
        POS_Y = "+y"
        NEG_Y = "-y"
        XY = "xy"

    bounding_box: BoundingBox = attrs.field(factory=BoundingBox.empty)
    material: list[str] = attrs.field(factory=list, converter=convert_list_str)
    color: list[str] = attrs.field(factory=list, converter=convert_list_str)
    hoi: list[str] = attrs.field(factory=list, converter=convert_list_str)
    face: Face = attrs.field(default=Face.NEG_Y)
    note: str = attrs.field(default=None)

    @property
    def as_text(self):
        result = ''
        for material in self.material:
            result += f"{material} "
        for color in self.color:
            result += f"{color} "
        for tag in self.tags:
            result += f"{tag} "
        result += f"{self.desc}"
        if self.note:
            result += f" note: {self.note}"

        return result

    @property
    def as_metadata(self):
        return {
            **super().as_metadata,
            "material": ",".join(self.material),
            "color": ",".join(self.color),
            "hoi": ",".join(self.hoi),
            "bounding_box": json.dumps(list(self.bounding_box)),
            "face": self.face.value,
            "note": self.note,
        }

    @classmethod
    def from_metadata(cls, metadata: dict):
        return cls(
            name=metadata.get("name", ""),
            path=metadata.get("path", ""),
            desc=metadata.get("desc", ""),
            tags=tags.split(",") if (tags := metadata.get("tags")) else [],
            material=material.split(",") if (material := metadata.get("material")) else [],
            color=color.split(",") if (color := metadata.get("color")) else [],
            hoi=hoi.split(",") if (hoi := metadata.get("hoi")) else [],
            face=cls.Face(metadata.get("face") or "-y"),
            note=metadata.get("note", None),
            bounding_box=BoundingBox(json.loads(bounding_box)) if (bounding_box := metadata.get("bounding_box")) else BoundingBox.empty(),
        )

    @property
    def as_procthor(self) -> dict:
        return {
            "assetId": self.path,
            "bounding_box": {
                "x": self.bounding_box.max_x - self.bounding_box.min_x,
                "y": self.bounding_box.max_y - self.bounding_box.min_y,
                "z": self.bounding_box.max_z - self.bounding_box.min_z,
            },
            "objectType": self.desc,
            "tags": self.tags,
            "primaryProperty": self.hoi[0] if self.hoi else "",
            "secondaryProperties": self.hoi[1:],
            "materials": [[material, material] for material in self.material],
            "face": self.face.value,
            "note": self.note,
        }


class ObjectDatabaseBuilder(DatabaseBuilder[ObjectAnnotation]):
    _annotation_cls = ObjectAnnotation
    _DISCOVER_PATH = 'Object'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._formats: set[ModelFormat] = {ModelFormat.USDZ}
        self._usd_baker: typing.Optional[UsdBaker] = None

        def store_db(annotation: ObjectAnnotation) -> None:
            self._db.store(
                AssetType.OBJECT.value,
                annotation
            )
        self._pipeline.append(store_db)

    @classmethod
    def find_mainfile(cls, annotation: ObjectAnnotation) -> str | None:
        from arena_models.utils.ModelConverter.converter import ModelConverter

        return next(
            (
                os.path.join(
                    os.path.relpath(walk[0], annotation.path),
                    f,
                )
                for walk in os.walk(annotation.path)
                for f in walk[2]
                if f.endswith(ModelConverter.exts())
            ), None
        )

    @classmethod
    def is_usd(cls, filename: str) -> bool:
        usd_extensions = {ModelFormat.USD, ModelFormat.USDA, ModelFormat.USDC, ModelFormat.USDZ}
        return any(os.path.splitext(filename.lower())[-1].lstrip(".") == ext.value for ext in usd_extensions)

    def process_entity(self, annotation, dest):
        from arena_models.utils.ModelConverter.converter import ModelConverter

        baked_usd_path: Path | None = None
        try:
            main_file = self.find_mainfile(annotation)
            if main_file is None:
                logger.warning("No supported model file found in %s", annotation.path)
                return None

            input_file = Path(annotation.path) / main_file

            if self._usd_baker is not None and self.is_usd(main_file):
                baked_usd_path = dest / f"{annotation.name}.temp.fbx"
                baked_usd_path.parent.mkdir(exist_ok=True)

                self._usd_baker.convert(
                    str(input_file.relative_to(self.input_path)),
                    str(baked_usd_path.relative_to(self.output_path))
                )
                logger.info("Baked USD model %s to %s", input_file, baked_usd_path)
                input_file = baked_usd_path

            model_paths = (
                dest /
                f"{annotation.name}.{ext.value}"
                for ext in self._formats
            )

            with ModelConverter() as model_converter:
                model_converter.load(str(input_file))
                # model_converter.rectify()
                bounding_box = model_converter.bounding_box().round(4)
                for model_path in model_paths:
                    model_path.parent.mkdir(exist_ok=True)
                    model_converter.save(str(model_path))

            logger.info("Processed model %s", annotation.path)
            for line in model_converter.stdout.splitlines():
                if line.startswith("Warning:"):
                    logger.warning("ModelConverter: %s", line)
                else:
                    logger.info("ModelConverter: %s", line)
            for line in model_converter.stderr.splitlines():
                logger.warning("ModelConverter: %s", line)

            annotation.bounding_box = bounding_box

            return annotation

        except Exception as e:
            logger.error("Unexpected error processing entity %s: %s", annotation.path, e)
            return None

        finally:
            if baked_usd_path is not None:
                try:
                    baked_usd_path.unlink()
                    logger.debug("Removed temporary baked file %s", baked_usd_path)
                except Exception as e:
                    logger.warning("Failed to remove temporary baked file %s: %s", baked_usd_path, e)

    enable = OptionRegistry()

    @enable.register('formats')
    def sdf(self, formats: str | None = None):
        self._formats.clear()
        if formats is None:
            formats = '*'
        if formats == '*':
            self._formats = set(ModelFormat.__members__.values())
            return
        for fmt in formats.split(','):
            try:
                self._formats.add(ModelFormat(fmt.lower()))
                logger.info("Enabling %s export option.", fmt)
            except ValueError:
                logger.error("Unknown model format specified: %s", fmt)

    @enable.register('procthor')
    def procthor(self):
        logger.info("Enabling Procthor export option.")
        self._procthor = {}
        self._pipeline.append(lambda annotation: self._procthor.update({annotation.path.replace(os.sep, "_"): annotation.as_procthor}))

        def export():
            with open(os.path.join(self.output_path, "asset-database.json"), "w") as f:
                json.dump(self._procthor, f, indent=4)
        self._post.append(export)

    @enable.register('bake-mdl')
    def bake_mdl(self, isaacsim_path: str | None = None):
        if isaacsim_path is not None:
            from arena_models.utils.ModelConverter.UsdBaker.LocalUsdBaker import (
                LocalUsdBaker,
            )
            self._usd_baker = LocalUsdBaker(
                input_dir=self.input_path,
                output_dir=self.output_path,
                isaacsim_path=Path(isaacsim_path),
            )

        else:
            from arena_models.utils.ModelConverter.UsdBaker.DockerUsdBaker import (
                DockerUsdBaker,
            )
            self._usd_baker = DockerUsdBaker(
                input_dir=Path(self.input_path),
                output_dir=Path(self.input_path),
            )

        def cleanup_baker():
            if self._usd_baker is not None:
                self._usd_baker.cleanup()
        self._post.append(cleanup_baker)
