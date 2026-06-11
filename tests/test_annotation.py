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


def test_name_text_splits_and_strips_counter():
    annotation = MaterialAnnotation(name="OakWood_01", path="materials/oak")
    assert annotation.name_text == "Oak Wood"


def test_name_text_keeps_meaningful_digits():
    assert MaterialAnnotation(name="Table_200cm", path="m/t").name_text == "Table 200cm"
    assert MaterialAnnotation(name="01", path="m/01").name_text == "01"


def test_as_text_includes_name_and_fields():
    annotation = MaterialAnnotation(
        name="OakWood_01", path="materials/oak", tags=["floor"], color=["brown"]
    )
    text = annotation.as_text
    assert "Oak Wood" in text
    assert "brown" in text
    assert "floor" in text


def test_as_text_skips_empty_parts():
    annotation = MaterialAnnotation(name="Brick", path="materials/brick")
    assert annotation.as_text == "Brick"
