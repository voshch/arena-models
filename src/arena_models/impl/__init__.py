"""Implementation modules for arena_models CLI commands."""

import enum
import json
import typing

import attrs
import cattrs

from arena_models.utils.geom import BoundingBox


class AssetType(enum.Enum):
    OBJECT = "object"
    MATERIAL = "material"


DATABASE_NAME = '.db'


converter = cattrs.Converter(prefer_attrib_converters=True)
converter.register_structure_hook(BoundingBox, lambda d, t: d if isinstance(d, BoundingBox) else BoundingBox(*d))
converter.register_unstructure_hook(BoundingBox, list)


@attrs.define
class Annotation(typing.Protocol):
    name: str
    path: str
    desc: str = ""

    @property
    def as_text(self) -> str:
        ...

    @property
    def as_metadata(self) -> dict:
        ...

    @classmethod
    def from_metadata(cls, metadata: dict) -> "Annotation":
        ...

    @property
    def as_procthor(self) -> dict:
        ...


@attrs.define
class ObjectAnnotation(Annotation):
    bounding_box: BoundingBox = attrs.field(factory=BoundingBox.empty)
    material: list[str] = attrs.field(factory=list, converter=lambda x: list() if x is None else list(x))
    color: list[str] = attrs.field(factory=list, converter=lambda x: list() if x is None else list(x))
    tags: list[str] = attrs.field(factory=list, converter=lambda x: list() if x is None else list(x))
    hoi: list[str] = attrs.field(factory=list, converter=lambda x: list() if x is None else list(x))

    @property
    def as_text(self) -> str:
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
    def as_metadata(self) -> dict:
        return {
            "name": self.name,
            "path": self.path,
            "desc": self.desc,
            "material": ",".join(self.material),
            "color": ",".join(self.color),
            "tags": ",".join(self.tags),
            "hoi": ",".join(self.hoi),
            "bounding_box": json.dumps(list(self.bounding_box)),
        }

    @classmethod
    def from_metadata(cls, metadata: dict) -> "ObjectAnnotation":
        return cls(
            name=metadata.get("name", ""),
            path=metadata.get("path", ""),
            desc=metadata.get("desc", ""),
            material=material.split(",") if (material := metadata.get("material")) else [],
            color=color.split(",") if (color := metadata.get("color")) else [],
            tags=tags.split(",") if (tags := metadata.get("tags")) else [],
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
