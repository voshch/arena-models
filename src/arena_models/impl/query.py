import os

from arena_models.utils.Database import Database

from . import DATABASE_NAME, AssetType, Annotation
from arena_models.impl.build import DatabaseBuilder


def query_database(database_path: str, asset_type: AssetType, query_target: str) -> Annotation:
    db = Database(os.path.join(database_path, DATABASE_NAME))
    result = db.query(asset_type.value, query_target, 5)

    data = result['metadatas']
    if not data or not data[0]:
        raise ValueError("No results found in the database.")

    annotation_t = DatabaseBuilder.Builder(asset_type)._annotation_cls

    annotation = annotation_t.from_metadata(dict(data[0][0]))

    return annotation
