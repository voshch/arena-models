import typer
from .cli import add_cmd

app = typer.Typer(
    name="arena-models",
    help="Arena Models - Build, query, and manage 3D model databases",
    rich_markup_mode="rich",
    no_args_is_help=True,
    add_completion=False
)

add_cmd(app)


@app.command()
def version(ctx: typer.Context):
    """Show version information."""
    from .cli.utils import safe_echo
    safe_echo("Arena Models v0.1.0", ctx)
    safe_echo("Build, query, and manage 3D model databases", ctx)


@app.callback()
def main(
    ctx: typer.Context,
    silent: bool = typer.Option(
        False,
        "-s", "--silent",
        help="Suppress all output messages"
    ),
):
    """
    Arena Models - A toolkit for building and managing 3D model databases.

    Use 'arena-models --help' to see available commands.
    """
    # Store silent flag in context for subcommands
    ctx.ensure_object(dict)
    ctx.obj['silent'] = silent


if __name__ == "__main__":
    app()
