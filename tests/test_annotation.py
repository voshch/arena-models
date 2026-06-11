from arena_models.impl.build.MaterialDatabaseBuilder import MaterialAnnotation


def test_metadata_roundtrip():
    annotation = MaterialAnnotation(
        name="OakWood_01",
        path="materials/oak",
        desc="oak wood planks",
        tags=["wood", "floor"],
        color=["brown"],
    )
    restored = MaterialAnnotation.from_metadata(annotation.as_metadata)
    assert restored == annotation


def test_as_text_splits_name():
    annotation = MaterialAnnotation(
        name="OakWood_01", path="materials/oak", tags=["floor"], color=["brown"]
    )
    text = annotation.as_text
    assert "Oak Wood 01" in text
    assert "brown" in text
    assert "floor" in text


def test_as_text_skips_empty_parts():
    annotation = MaterialAnnotation(name="Brick", path="materials/brick")
    assert annotation.as_text == "Brick"
