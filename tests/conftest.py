import pytest

from arena_models.impl import DATABASE_NAME
from arena_models.impl.build.MaterialDatabaseBuilder import MaterialAnnotation
from arena_models.utils.Database import Database

MATERIALS = [
    MaterialAnnotation(
        name="OakWood",
        path="materials/oak",
        desc="light brown oak wood planks",
        tags=["wood"],
    ),
    MaterialAnnotation(
        name="RedBrick", path="materials/brick", desc="red clay brick wall"
    ),
    MaterialAnnotation(
        name="SteelPlate", path="materials/steel", desc="brushed steel metal plate"
    ),
]


@pytest.fixture(scope="session")
def database_path(tmp_path_factory):
    path = tmp_path_factory.mktemp("db")
    db = Database(path / DATABASE_NAME)
    for annotation in MATERIALS:
        db.store("material", annotation)
    return path
