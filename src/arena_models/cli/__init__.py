"""CLI module for arena_models."""

# Import command functions
from arena_models.cli.build import add_to_cmd as add_build
from arena_models.cli.merge import add_to_cmd as add_merge
from arena_models.cli.db import add_to_cmd as add_db
from arena_models.cli.down import add_to_cmd as add_download


# Add all commands to the main app
def add_cmd(app):
    add_build(app)
    add_merge(app)
    add_db(app)
    add_download(app)
