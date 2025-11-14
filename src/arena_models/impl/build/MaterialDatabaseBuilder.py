from __future__ import annotations

import json
import os
import shutil

import attrs

from arena_models.impl import ANNOTATION_NAME, Annotation, AssetType
from arena_models.utils.logging import get_logger

from . import DatabaseBuilder, OptionRegistry


logger = get_logger('build.object')


@attrs.define
class MaterialAnnotation(Annotation):
    color: list[str] = attrs.field(factory=list, converter=lambda x: list() if x is None else list(x))

    @property
    def as_text(self):
        result = ''
        for color in self.color:
            result += f"{color} "
        result += f"{self.desc}"

        return result

    @property
    def as_metadata(self):
        return {
            **super().as_metadata,
            "color": ",".join(self.color),
        }

    @classmethod
    def from_metadata(cls, metadata: dict):
        return cls(
            name=metadata.get("name", ""),
            path=metadata.get("path", ""),
            desc=metadata.get("desc", ""),
            tags=tags.split(",") if (tags := metadata.get("tags")) else [],
            color=color.split(",") if (color := metadata.get("color")) else [],
        )

    @property
    def as_procthor(self) -> dict:
        return {
            "todo": None,
        }


class MaterialDatabaseBuilder(DatabaseBuilder[MaterialAnnotation]):
    _annotation_cls = MaterialAnnotation
    _DISCOVER_PATH = 'Material'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        def store_db(annotation: MaterialAnnotation) -> None:
            self._db.store(
                AssetType.OBJECT.value,
                annotation
            )
        self._pipeline.append(store_db)

    def process_entity(self, annotation, dest):
        try:
            shutil.copytree(annotation.path, dest, ignore=shutil.ignore_patterns(ANNOTATION_NAME), dirs_exist_ok=True)

            return annotation

        except Exception as e:
            logger.error("Unexpected error processing entity %s: %s", annotation.path, e)
            return None

    enable = OptionRegistry()

    @enable.register('procthor')
    def procthor(self):
        logger.info("Enabling Procthor export option.")
        self._procthor = {}
        self._pipeline.append(lambda annotation: self._procthor.update({annotation.path.replace(os.sep, "_"): annotation.as_procthor}))

        def export():
            with open(self.output_path / "arena-materials-database.json", "w") as f:
                json.dump(self._procthor, f, indent=4)
        self._post.append(export)
