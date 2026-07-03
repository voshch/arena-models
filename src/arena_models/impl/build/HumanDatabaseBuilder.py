from __future__ import annotations

import shutil

import attrs

from arena_models.impl import ANNOTATION_NAME, AssetType, SpatialAnnotation
from arena_models.utils.logging import get_logger

from . import DatabaseBuilder, OptionRegistry

logger = get_logger("build.human")


@attrs.define
class HumanAnnotation(SpatialAnnotation):
    pass


class HumanDatabaseBuilder(DatabaseBuilder[HumanAnnotation]):
    _annotation_cls = HumanAnnotation
    _DISCOVER_PATH = "Human"

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        def store_db(annotation: HumanAnnotation) -> None:
            self._db.store(AssetType.HUMAN.value, annotation)

        self._pipeline.append(store_db)

    def process_entity(self, annotation, dest):
        try:
            shutil.copytree(
                annotation.path,
                dest,
                ignore=shutil.ignore_patterns(ANNOTATION_NAME),
                dirs_exist_ok=True,
            )

            return annotation

        except Exception as e:
            logger.error("Unexpected error processing entity %s: %s", annotation.path, e)
            return None

    enable = OptionRegistry()
