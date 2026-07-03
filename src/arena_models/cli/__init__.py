"""CLI module for arena_models."""

import typer

from arena_models.cli.db import add_to_cmd as add_db
from arena_models.cli.net import add_to_cmd as add_net
from arena_models.cli.semantic import add_to_cmd as add_semantic


def add_cmd(app: typer.Typer):
    add_db(app)
    add_net(app)
    add_semantic(app)
