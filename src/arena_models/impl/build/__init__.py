from __future__ import annotations

import abc
import os
import typing
from collections.abc import Callable, Iterable, Iterator

import yaml

from arena_models.impl import ANNOTATION_NAME, DATABASE_NAME, Annotation, AssetType, converter
from arena_models.utils.Database import Database
from arena_models.utils.logging import get_logger, get_manager

logger = get_logger('build')
manager = get_manager()


class OptionRegistry:
    def __init__(self):
        self._registry = {}

    def register(self, name):
        def decorator(func):
            self._registry[name] = func
            return func
        return decorator

    def __get__(self, instance, _):
        class BoundRegistry:
            def __init__(self, registry, instance):
                self._registry = registry
                self._instance = instance

            def __call__(self, name):
                if name in self._registry:
                    func = self._registry[name]
                    return func.__get__(self._instance, type(self._instance)).__call__()

        return BoundRegistry(self._registry, instance)


AnnotationT = typing.TypeVar("AnnotationT", bound=Annotation)


class DatabaseBuilder(abc.ABC, typing.Generic[AnnotationT]):
    __registry: typing.ClassVar[dict[AssetType, typing.Callable[[AssetType], typing.Type[DatabaseBuilder]]]] = {}
    _type: typing.ClassVar[AssetType]

    @classmethod
    def register(cls, type_: AssetType, /) -> typing.Callable[[typing.Callable[[AssetType], typing.Type[DatabaseBuilder]]], None]:
        def wrapper(builder_cls: typing.Callable[[AssetType], typing.Type[DatabaseBuilder]]) -> None:
            cls.__registry[type_] = builder_cls
        return wrapper

    @classmethod
    def get_registered(cls) -> Iterable[AssetType]:
        return cls.__registry.keys()

    @classmethod
    def Builder(cls, type_: AssetType) -> typing.Type[DatabaseBuilder]:
        if type_ not in cls.__registry:
            raise ValueError(f"No builder registered for type: {type_}")
        builder_cls = cls.__registry[type_](type_)
        builder_cls._type = type_
        return builder_cls

    _annotation_cls: typing.ClassVar[typing.Type[Annotation]]

    @classmethod
    def read_annotation_file(cls, dir_path: str) -> AnnotationT | None:
        try:
            with open(os.path.join(dir_path, ANNOTATION_NAME), "r") as yaml_file:
                return typing.cast(
                    AnnotationT,
                    converter.structure(
                        {
                            **yaml.safe_load(yaml_file),
                            'name': os.path.basename(dir_path),
                            'path': dir_path,
                        },
                        cls._annotation_cls
                    ),
                )
        except Exception:
            return None

    def __init__(self, input_path, output_path, overwrite: int = 0, **kwargs):
        self.input_path = input_path
        self.output_path = output_path
        self._overwrite = overwrite

        self._pipeline: list[Callable[[AnnotationT], typing.Any]] = []
        self._post: list[Callable] = []

        self._db = Database(path=os.path.join(self.output_path, DATABASE_NAME))

        def save_annotation(annotation: AnnotationT) -> None:
            annotation_path = os.path.join(self.output_path, annotation.path, ANNOTATION_NAME)
            os.makedirs(os.path.dirname(annotation_path), exist_ok=True)
            with open(annotation_path, "w") as f:
                yaml.safe_dump(converter.unstructure(annotation), f)
        self._pipeline.append(save_annotation)

    enable: OptionRegistry

    def build(self):
        logger.info("Starting database build...")

        status_bar = manager.status_bar(
            status_format='Building {build_type} database from {input_path}: {stage}{fill}',
            input_path=self.input_path,
            build_type=self._type.name,
            stage='Initializing',
        )

        status_bar.update(stage='Discovery')
        queue = list(self.discover())

        if not queue:
            logger.info("Skipped %s build: No entities found.", self._type.name)
            status_bar.update(stage='Done (skipped)')
        else:
            status_bar.update(stage='Processing')
            progress = manager.counter(total=len(queue), desc="Entities", unit="entities", leave=False)

            with progress:
                success = progress.add_subcounter('green')
                failure = progress.add_subcounter('red')
                skipped = progress.add_subcounter('blue')

                for annotation in queue:
                    progress.update()
                    status_bar.update(stage=f'Processing {annotation.path}')

                    target_path = os.path.relpath(annotation.path, self.input_path)
                    dest_path = os.path.join(self.output_path, target_path)

                    if os.path.exists(os.path.join(dest_path, ANNOTATION_NAME)) and not self._overwrite:
                        logger.info("Skipping existing entity at %s", annotation.path)
                        skipped.update_from(progress, 1)
                        continue

                    logger.info("Processing entity at %s", annotation.path)
                    os.makedirs(dest_path, exist_ok=True)

                    if self._overwrite >= 0:
                        annotation = self.process_entity(annotation, dest_path)
                        if annotation is None:
                            os.rmdir(dest_path)
                            failure.update_from(progress, 1)
                            continue

                    annotation.path = target_path
                    for fn in self._pipeline:
                        fn(annotation)
                    success.update_from(progress, 1)

            if self._post:
                status_bar.update(stage='Post-processing')
                progress = manager.counter(total=len(self._post), desc="Post-processing", unit="steps", format='{desc}{desc_pad}{count:d} {unit}{unit_pad}{elapsed}]{fill}', leave=False)
                with progress:
                    logger.info("Running export...")
                    for fn in progress(self._post):
                        fn()

            # actually looked these up by hand, LLMs won't replace me just yet
            status_bar.update(stage=f'Done ✓{success.count} ✗{failure.count} ➤{skipped.count}')

        status_bar.close()
        logger.info("Database build complete.")

    _DISCOVER_PATH: typing.ClassVar[str] = ''

    def discover(self, *, base_path: str | None = None, filter_: typing.Callable[[str], bool] | None = None) -> Iterator[AnnotationT]:

        if filter_ is None:
            def filter__(path):
                del path
                return True
            filter_ = filter__

        if base_path is None:
            base_path = self._DISCOVER_PATH

        for root, dirs, files in os.walk(os.path.join(self.input_path, base_path)):
            if not filter_(root):
                continue
            logger.info("Scanning directory: %s", root)
            if ANNOTATION_NAME in files:
                # don't recurse into subdirs
                dirs.clear()
                if (annotation := self.read_annotation_file(root)) is not None:
                    yield annotation

    @abc.abstractmethod
    def process_entity(self, annotation: AnnotationT, dest: str) -> AnnotationT | None:
        ...


@DatabaseBuilder.register(AssetType.MATERIAL)
def lazy_material(type_: AssetType) -> typing.Type[DatabaseBuilder]:
    from .MaterialDatabaseBuilder import MaterialDatabaseBuilder
    return MaterialDatabaseBuilder


@DatabaseBuilder.register(AssetType.OBJECT)
def lazy_object(type_: AssetType) -> typing.Type[DatabaseBuilder]:
    from .ObjectDatabaseBuilder import ObjectDatabaseBuilder
    return ObjectDatabaseBuilder
