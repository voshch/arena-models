from arena_models.impl.build.ObjectDatabaseBuilder import ObjectAnnotation
from arena_models.utils.geom import BoundingBox


def make_annotation():
    return ObjectAnnotation(
        name="OfficeChair",
        path="objects/chair",
        desc="ergonomic office chair",
        tags=["chair", "office"],
        material=["plastic", "fabric"],
        color=["black"],
        hoi=["sit"],
        face=ObjectAnnotation.Face.POS_X,
        note="wheels included",
        bounding_box=BoundingBox(((-0.5, 0.5), (-0.4, 0.4), (0.0, 1.2))),
    )


def test_metadata_roundtrip():
    annotation = make_annotation()
    restored = ObjectAnnotation.from_metadata(annotation.as_metadata)
    assert restored == annotation
    assert restored.as_metadata == annotation.as_metadata


def test_parse_face():
    parse = ObjectAnnotation._parse_face
    assert parse(None) is ObjectAnnotation.Face.NEG_Y
    assert parse("") is ObjectAnnotation.Face.NEG_Y
    assert parse("+x") is ObjectAnnotation.Face.POS_X
    assert parse("garbage") is ObjectAnnotation.Face.NEG_Y


def test_parse_face_legacy_angles():
    for face in ObjectAnnotation.Face:
        assert ObjectAnnotation._parse_face(face.angle) is face
        assert ObjectAnnotation._parse_face(str(face.angle)) is face


def test_as_text():
    text = make_annotation().as_text
    for part in ("plastic", "black", "office", "ergonomic office chair", "note: wheels included"):
        assert part in text
