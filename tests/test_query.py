import pytest

from arena_models.impl import AssetType
from arena_models.impl.query import query_database


def test_top_n_ranked(database_path):
    results = query_database(str(database_path), AssetType.MATERIAL, "wooden floor", n=2)
    assert len(results) == 2
    assert results[0][0].path == "materials/oak"
    scores = [score for _, score in results]
    assert scores == sorted(scores)


def test_default_returns_single(database_path):
    results = query_database(str(database_path), AssetType.MATERIAL, "metal plate")
    assert len(results) == 1
    assert results[0][0].path == "materials/steel"


def test_empty_database_raises(tmp_path):
    with pytest.raises(ValueError):
        query_database(str(tmp_path), AssetType.MATERIAL, "anything")
