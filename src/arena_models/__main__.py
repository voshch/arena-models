import typer

# Import command functions
from arena_models.cli.build import add_to_cmd as add_build
from arena_models.cli.merge import add_to_cmd as add_merge
from arena_models.cli.query import add_to_cmd as add_query
from arena_models.cli.get import add_to_cmd as add_get
from arena_models.cli.down import add_to_cmd as add_download

app = typer.Typer(
    name="arena-models",
    help="Arena Models - Build, query, and manage 3D model databases",
    rich_markup_mode="rich",
    no_args_is_help=True,
    add_completion=False
)

# Add all commands to the main app
add_build(app)
add_merge(app)
add_query(app)
add_get(app)
add_download(app)


@app.command()
def version():
    """Show version information."""
    typer.echo("Arena Models v0.1.0")
    typer.echo("Build, query, and manage 3D model databases")


@app.callback()
def main():
    """
    Arena Models - A toolkit for building and managing 3D model databases.

    Use 'arena-models --help' to see available commands.
    """
    pass


if __name__ == "__main__":
    app()
