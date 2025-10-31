import os

from arena_models.utils.Database import Database, Annotation

from . import DATABASE_NAME, AssetType


def list_database(database_path: str, asset_type: AssetType) -> list[Annotation]:
    db = Database(os.path.join(database_path, DATABASE_NAME))
    result = db.list_all(asset_type.value)

    return [Annotation.from_metadata(dict(item)) for item in result['metadatas'] if item]
