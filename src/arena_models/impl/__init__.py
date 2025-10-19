"""Implementation modules for arena_models CLI commands."""

import enum
import typing

import attrs
import cattrs.preconf.pyyaml

from arena_models.utils.geom import BoundingBox
from arena_models.utils.logging import get_logger

logger = get_logger('arena_models.impl')

converter = cattrs.preconf.pyyaml.make_converter(prefer_attrib_converters=True)

converter.register_structure_hook(BoundingBox, lambda d, t: d if isinstance(d, BoundingBox) else BoundingBox(d))


def _unstructure_bounding_box(box: BoundingBox) -> list:
    if (volume := box.volume) < 1e-9:
        if volume < 0:
            logger.warning("Serializing Negative-volume (%d) BoundingBox: %s", volume, box)
        else:
            logger.warning("Serializing Zero-volume (%d) BoundingBox: %s", volume, box)
    return list(box)


converter.register_unstructure_hook(BoundingBox, _unstructure_bounding_box)


class AssetType(enum.Enum):
    OBJECT = "object"
    MATERIAL = "material"


DATABASE_NAME = '.db'
ANNOTATION_NAME = 'annotation.yaml'


@attrs.define
class Annotation(typing.Protocol):
    name: str
    path: str
    desc: str = ""
    tags: list[str] = attrs.field(factory=list, converter=lambda x: list() if x is None else list(x))

    @property
    def as_text(self) -> str:
        ...

    @property
    def as_metadata(self) -> dict:
        ...

    @classmethod
    def from_metadata(cls, metadata: dict) -> typing.Self:
        ...
