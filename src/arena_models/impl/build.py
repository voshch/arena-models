from __future__ import annotations

import abc
import json
import logging
import os
import typing
from collections.abc import Callable, Iterator

import enlighten
import yaml

from arena_models.utils.Database import Database

from . import DATABASE_NAME, Annotation, AssetType, ObjectAnnotation, converter

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger('arena_models.build')


class DatabaseBuilder(abc.ABC):
    __registry: typing.ClassVar[dict[AssetType, typing.Type[DatabaseBuilder]]] = {}
    _type: typing.ClassVar[AssetType]

    @classmethod
    def register(cls, type_: AssetType, /) -> typing.Callable[[typing.Type[DatabaseBuilder]], None]:
        def wrapper(builder_cls: typing.Type[DatabaseBuilder]) -> None:
            builder_cls._type = type_
            cls.__registry[type_] = builder_cls
        return wrapper

    @classmethod
    def Builder(cls, type_: AssetType) -> typing.Type[DatabaseBuilder]:
        if type_ not in cls.__registry:
            raise ValueError(f"No builder registered for type: {type_}")
        return cls.__registry[type_]

    @classmethod
    def read_annotation_file(cls, file_path: str) -> dict:
        with open(os.path.join(file_path, "annotation.yaml"), "r") as yaml_file:
            return yaml.safe_load(yaml_file)

    def __init__(self, input_path, output_path, **kwargs):
        self.input_path = input_path
        self.output_path = output_path

        self._pipeline: list[Callable[[Annotation], typing.Any]] = []
        self._post: list[Callable] = []

        self._db = Database(path=os.path.join(self.output_path, DATABASE_NAME))

        def save_annotation(annotation: Annotation) -> None:
            with open(os.path.join(self.output_path, annotation.path, "annotation.yaml"), "w") as f:
                yaml.safe_dump(converter.unstructure(annotation), f)
        self._pipeline.append(save_annotation)

    def build(self):
        logger.info("Starting database build...")

        with enlighten.get_manager() as manager:
            status_bar = manager.status_bar(
                status_format='Building {build_type} database from {input_path}: {stage}{fill}',
                input_path=self.input_path,
                build_type=self._type.name,
                stage='Initializing',
            )

            status_bar.update(stage='Discovery')
            queue = list(self.discover())

            status_bar.update(stage='Processing')
            progress = manager.counter(total=len(queue), desc="Entities", unit="entities")
            success = progress.add_subcounter('green')
            failure = progress.add_subcounter('red')

            for entity_path in queue:
                progress.update()
                status_bar.update(stage=f'Processing {entity_path}')
                logger.info("Processing entity at %s", entity_path)
                if (annotation := self.process_entity(entity_path)) is not None:
                    for fn in self._pipeline:
                        fn(annotation)
                    success.update_from(progress, 1)
                else:
                    failure.update_from(progress, 1)

            status_bar.update(stage='Post-processing')
            progress = manager.counter(total=len(self._post), desc="Post-processing", unit="steps", format='{desc}{desc_pad}{count:d} {unit}{unit_pad}{elapsed}]{fill}')
            logger.info("Running export...")
            for fn in self._post:
                fn()
                progress.update()

            status_bar.update(stage='Done')
            logger.info("Database build complete.")

    def discover(self) -> Iterator[str]:
        for root, dirs, files in os.walk(self.input_path):
            logger.info("Scanning directory: %s", root)
            if "annotation.yaml" in files:
                # don't recurse into subdirs
                dirs.clear()
                yield os.path.relpath(root, self.input_path)

    @abc.abstractmethod
    def process_entity(self, entity_path: str) -> Annotation | None:
        ...

    @abc.abstractmethod
    def procthor(self):
        ...


@DatabaseBuilder.register(AssetType.OBJECT)
class ObjectDatabaseBuilder(DatabaseBuilder):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        def store_db(annotation: Annotation) -> None:
            self._db.store(
                AssetType.OBJECT.value,
                annotation
            )
        self._pipeline.append(store_db)

    def process_entity(self, entity_path: str) -> ObjectAnnotation | None:
        try:
            entity_name = os.path.basename(entity_path)
            realpath = os.path.join(self.input_path, entity_path)

            from arena_models.utils.ModelConverter import ModelConverter
            annotation = self.read_annotation_file(realpath)

            main_file = next((f for f in next(os.walk(realpath))[2] if f.endswith(ModelConverter.exts())), None)
            if main_file is None:
                logger.warning("No supported model file found in %s", realpath)
                return None

            target_path = os.path.join(self.output_path, entity_path, "usd", f"{entity_name}.usdz")
            with ModelConverter() as model_converter:
                model_converter.load(os.path.join(realpath, main_file))
                bounding_box = model_converter.bounding_box().round(4)
                os.makedirs(os.path.dirname(target_path), exist_ok=True)
                model_converter.save(target_path)

            logger.info("Processed model %s", entity_path)
            for line in model_converter.stdout.splitlines():
                if line.startswith("Warning:"):
                    logger.warning("ModelConverter: %s", line)
                else:
                    logger.info("ModelConverter: %s", line)
            for line in model_converter.stderr.splitlines():
                logger.warning("ModelConverter: %s", line)

            return converter.structure(
                dict(
                    **annotation,
                    name=entity_name,
                    path=entity_path,
                    bounding_box=bounding_box,
                ),
                ObjectAnnotation
            )
        except (KeyboardInterrupt, SystemExit):
            raise
        except Exception as e:
            logger.error("Unexpected error processing entity %s: %s", entity_path, e)
            return None

    def procthor(self):
        self._procthor = {}
        self._pipeline.append(lambda annotation: self._procthor.update({annotation.path.replace(os.sep, "_"): annotation.as_procthor}))

        def export():
            with open(os.path.join(self.output_path, "asset-database.json"), "w") as f:
                json.dump(self._procthor, f, indent=4)
        self._post.append(export)
