from __future__ import annotations

import enum
import io
import tarfile

import attrs
import cattrs
import text_processing.language_processing as language_processing

from .utils.geom import BoundingBox

_converter = cattrs.Converter()


class ModelType(enum.Enum):
    Usd = "usd"


class Database:
    AssetId = str

    class Kind(enum.Enum):
        Objects = "objects"
        Materials = "materials"
        Pedestrians = "pedestrians"

    @attrs.define
    class Manifest:
        version: str
        kind: Database.Kind
        database: str

    @attrs.define
    class Entry:
        asset_id: str
        bounding_box: BoundingBox
        materials: list[str]
        primary_property: str
        scenes: str
        secondary_properties: list[str]
        split: str
        states: dict[str, str]

    _manifest: Manifest
    _database: object
    _files: tarfile.TarFile

    @property
    def kind(self) -> Kind:
        return self._manifest.kind

    @classmethod
    def open(cls, path: str) -> Database:
        with open(path, "rb") as f:
            files = tarfile.open(fileobj=f, mode="r|*")
        manifest_f = files.extractfile("manifest.yaml")
        if manifest_f is None:
            raise ValueError("Invalid database: missing manifest.yaml")

        manifest = _converter.structure_yaml(manifest_f.read(), cls.Manifest)

        database = files.extractfile(manifest.database)
        if database is None:
            raise ValueError(f"Invalid database: missing {manifest.database}")

        db = cls()
        db._manifest = manifest
        db._database = database
        return db

    def save(self, path: str) -> None:
        with open(path, "wb") as f:
            files = tarfile.open(fileobj=f, mode="w|gz")
            manifest_data = _converter.unstructure_yaml(self._manifest)
            manifest_info = tarfile.TarInfo(name="manifest.yaml")
            manifest_info.size = len(manifest_data)
            files.addfile(manifest_info, fileobj=io.BytesIO(manifest_data))

    def query(self, text: str) -> list[Database.Entry]:
        ...

    def get(self, asset_id: AssetId) -> Database.Entry | None:
        ...

    def file(self, asset_id: AssetId) -> bytes | None:
        ...
