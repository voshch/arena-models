from arena_models.impl.fetch import Bucket

API = "https://storage.googleapis.com/storage/v1/b/bkt/o"


def test_object_url_bare():
    assert Bucket("bkt")._object_url("") == API


def test_object_url_params():
    assert Bucket("bkt")._object_url("", prefix="a/") == f"{API}?prefix=a%2F"


def test_object_url_encodes_object():
    assert Bucket("bkt")._object_url("dir/file.yaml", fields="name") == f"{API}/dir%2Ffile.yaml?fields=name"
