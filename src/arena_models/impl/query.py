import os

from arena_models.impl.build import DatabaseBuilder
from arena_models.utils.Database import Database
from arena_models.utils.logging import get_logger

from . import DATABASE_NAME, Annotation, AssetType

logger = get_logger("query")


def query_database(database_path: str, asset_type: AssetType, query_target: str, n: int = 1, where: dict | None = None) -> list[tuple[Annotation, float]]:
    logger.info("Querying database at %s for %s '%s'", database_path, asset_type.value, query_target)

    db = Database(os.path.join(database_path, DATABASE_NAME))
    result = db.query(asset_type.value, query_target, n, where=where)
    logger.debug("Query result: %s", result)

    metadatas = result["metadatas"]
    if not metadatas or not metadatas[0]:
        raise ValueError("No results found in the database.")

    distances = result["distances"]
    scores = distances[0] if distances else [0.0] * len(metadatas[0])

    annotation_t = DatabaseBuilder.Builder(asset_type)._annotation_cls

    return [(annotation_t.from_metadata(dict(metadata)), score) for metadata, score in zip(metadatas[0], scores, strict=True)]
