from typer.testing import CliRunner

from arena_models.__main__ import app

runner = CliRunner()


def test_version():
    result = runner.invoke(app, ["version"])
    assert result.exit_code == 0
    assert result.output.startswith("Arena Models v")


def test_invalid_log_level():
    result = runner.invoke(app, ["--log-level", "bogus", "version"])
    assert result.exit_code == 1


def test_db_query_outputs_path(database_path):
    result = runner.invoke(app, ["-s", "db", str(database_path), "query", "material", "wooden floor"])
    assert result.exit_code == 0
    assert result.output.strip() == str(database_path / "materials/oak")


def test_db_query_scores(database_path):
    result = runner.invoke(app, ["-s", "db", str(database_path), "query", "material", "wooden floor", "-n", "2", "--scores"])
    assert result.exit_code == 0
    lines = result.output.strip().splitlines()
    assert len(lines) == 2
    for line in lines:
        path, score = line.split("\t")
        float(score)
