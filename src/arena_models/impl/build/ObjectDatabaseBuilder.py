from __future__ import annotations

import json
import os

import attrs

from arena_models.impl import Annotation, AssetType
from arena_models.utils.geom import BoundingBox
from arena_models.utils.logging import get_logger

from . import DatabaseBuilder, OptionRegistry


logger = get_logger('build.object')


@attrs.define
class ObjectAnnotation(Annotation):
    bounding_box: BoundingBox = attrs.field(factory=BoundingBox.empty)
    material: list[str] = attrs.field(factory=list, converter=lambda x: list() if x is None else list(x))
    color: list[str] = attrs.field(factory=list, converter=lambda x: list() if x is None else list(x))
    hoi: list[str] = attrs.field(factory=list, converter=lambda x: list() if x is None else list(x))

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

        return result

    @property
    def as_metadata(self):
        return {
            **super().as_metadata,
            "material": ",".join(self.material),
            "color": ",".join(self.color),
            "hoi": ",".join(self.hoi),
            "bounding_box": json.dumps(list(self.bounding_box)),
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
        }


class ObjectDatabaseBuilder(DatabaseBuilder[ObjectAnnotation]):
    _annotation_cls = ObjectAnnotation
    _DISCOVER_PATH = 'Object'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._formats: list[str] = ['usdz']

        def store_db(annotation: ObjectAnnotation) -> None:
            self._db.store(
                AssetType.OBJECT.value,
                annotation
            )
        self._pipeline.append(store_db)

    def process_entity(self, annotation, dest):
        from arena_models.utils.ModelConverter import ModelConverter

        try:
            main_file = next(
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
            if main_file is None:
                logger.warning("No supported model file found in %s", annotation.path)
                return None

            model_paths = (os.path.join(
                dest,
                f"{annotation.name}.{ext}")
                for ext in self._formats
            )

            with ModelConverter() as model_converter:
                model_converter.load(os.path.join(annotation.path, main_file))
                bounding_box = model_converter.bounding_box().round(4)

                for model_path in model_paths:
                    os.makedirs(os.path.dirname(model_path), exist_ok=True)
                    model_converter.save(model_path)

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

    enable = OptionRegistry()

    @enable.register('format-sdf')
    def sdf(self):
        logger.info("Enabling SDF export option.")
        self._formats.append('sdf')

    @enable.register('procthor')
    def procthor(self):
        logger.info("Enabling Procthor export option.")
        self._procthor = {}
        self._pipeline.append(lambda annotation: self._procthor.update({annotation.path.replace(os.sep, "_"): annotation.as_procthor}))

        def export():
            with open(os.path.join(self.output_path, "asset-database.json"), "w") as f:
                json.dump(self._procthor, f, indent=4)
        self._post.append(export)
