from __future__ import annotations

import abc
import json
import logging
import os
import typing
from collections.abc import Callable, Iterator

import attrs
import cattrs
import chromadb
import yaml
from text_processing.language_processing import (
    embed_text_with_weight,
    load_spacy_model,
    store_embedding,
)

from arena_models.utils.enums import AssetType
from arena_models.utils.geom import BoundingBox

logger = logging.getLogger()

converter = cattrs.Converter()
converter.register_structure_hook(BoundingBox, lambda d, t: d if isinstance(d, BoundingBox) else None)


@attrs.define
class Annotation:
    name: str
    path: str
    bounding_box: BoundingBox
    desc: str = ""
    material: list[str] = attrs.field(factory=list)
    color: list[str] = attrs.field(factory=list)
    tags: list[str] = attrs.field(factory=list)
    hoi: list[str] = attrs.field(factory=list)

    @property
    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "path": self.path,
            "desc": self.desc,
            "material": ",".join(self.material),
            "color": ",".join(self.color),
            "tags": ",".join(self.tags),
            "hoi": ",".join(self.hoi),
        }

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

    @property
    def as_processed(self) -> str:
        res = []
        for color in self.color:
            res.append(color.lower())
        for material in self.material:
            res.append(material.lower())
        for tag in self.tags:
            res.append(tag.lower())
        res.append(self.desc.lower())
        return " ".join(res)


class DatabaseBuilder(abc.ABC):
    __registry: typing.ClassVar[dict[AssetType, typing.Type[DatabaseBuilder]]] = {}

    @classmethod
    def register(cls, type_: AssetType, /) -> typing.Callable[[typing.Type[DatabaseBuilder]], None]:
        def wrapper(builder_cls: typing.Type[DatabaseBuilder]) -> None:
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
            return {k: v for k, v in yaml.safe_load(yaml_file).items() if k in attrs.fields(Annotation)}

    def __init__(self, input_path, output_path, **kwargs):
        self.input_path = input_path
        self.output_path = output_path

        self._pipeline: list[Callable[[Annotation], typing.Any]] = []
        self._post: list[Callable] = []

        self._spacy_model = load_spacy_model()
        self._client = chromadb.PersistentClient(path=os.path.join(self.output_path, "arena_models_database"))
        self._save_annotations()

    def build(self):
        logger.info("Starting database build...")
        for entity_path in self.discover():
            logger.info("Processing entity at %s", entity_path)
            if (annotation := self.process_entity(entity_path)) is not None:
                for fn in self._pipeline:
                    fn(annotation)
        logger.info("Running export...")
        for fn in self._post:
            fn()
        logger.info("Database build complete.")

    def discover(self) -> Iterator[str]:
        for root, dirs, files in os.walk(self.input_path):
            if "annotation.yaml" in files:
                # don't recurse into subdirs
                dirs.clear()
                yield os.path.relpath(root, self.input_path)

    @abc.abstractmethod
    def _save_annotations(self) -> None:
        ...

    @abc.abstractmethod
    def process_entity(self, entity_path: str) -> Annotation | None:
        ...

    @abc.abstractmethod
    def procthor(self):
        ...


@DatabaseBuilder.register(AssetType.OBJECT)
class ObjectDatabaseBuilder(DatabaseBuilder):
    def process_entity(self, entity_path: str):
        entity_name = os.path.basename(entity_path)
        realpath = os.path.join(self.input_path, entity_path)

        from arena_models.utils.ModelConverter import ModelConverter
        annotation = self.read_annotation_file(realpath)

        main_file = next((f for f in next(os.walk(realpath))[2] if f.endswith(ModelConverter.exts())), None)
        if main_file is None:
            logger.warning("No supported model file found in %s", realpath)
            return

        target_path = os.path.join(self.output_path, entity_path, "usd", f"{entity_name}.usdz")
        with ModelConverter() as model_converter:
            model_converter.load(os.path.join(realpath, main_file))
            bounding_box = model_converter.bounding_box()
            os.makedirs(os.path.dirname(target_path), exist_ok=True)
            model_converter.save(target_path)

        logger.info("Processed model %s", entity_path)
        return converter.structure(
            dict(
                **annotation,
                name=entity_name,
                path=entity_path,
                bounding_box=bounding_box,
            ),
            Annotation
        )

    def procthor(self):
        self._procthor = {}
        self._pipeline.append(lambda annotation: self._procthor.update({annotation.path.replace(os.sep, "_"): annotation.as_procthor}))

        def export():
            with open(os.path.join(self.output_path, "asset-database.json"), "w") as f:
                json.dump(self._procthor, f, indent=4)
        self._post.append(export)

    def _save_annotations(self) -> None:
        guid = 0

        def process(annotation: Annotation) -> None:
            nonlocal guid
            guid += 1
            model_str = annotation.as_processed
            new_embedding = embed_text_with_weight(model_str, self._spacy_model)
            store_embedding(
                model_str,
                new_embedding,
                annotation.as_dict,
                guid,
                self._client,
                "models",
            )
        self._pipeline.append(process)
