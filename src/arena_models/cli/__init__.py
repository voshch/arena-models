"""CLI module for arena_models."""

import typer

from arena_models.cli.db import add_to_cmd as add_db
from arena_models.cli.net import add_to_cmd as add_net


def add_cmd(app: typer.Typer):
    add_db(app)
    add_net(app)
