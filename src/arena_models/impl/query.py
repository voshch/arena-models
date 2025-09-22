import os

import chromadb
from arena_models.utils.Database import Database

from . import DATABASE_NAME, ObjectAnnotation, AssetType


def query_database(database_path: str, asset_type: AssetType, query_target: str) -> ObjectAnnotation:
    db = Database(os.path.join(database_path, DATABASE_NAME))
    result = db.query(asset_type.value, query_target, 5)

    data = result['metadatas']
    if not data or not data[0]:
        raise ValueError("No results found in the database.")

    annotation = ObjectAnnotation.from_metadata(dict(data[0][0]))
    annotation.path = os.path.abspath(os.path.join(database_path, annotation.path))

    return annotation
