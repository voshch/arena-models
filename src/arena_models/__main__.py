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
    log_level: str = typer.Option(
        "WARNING",
        "-l", "--log-level",
        help="Set logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
        case_sensitive=False
    ),
):
    """
    Arena Models - A toolkit for building and managing 3D model databases.

    Use 'arena-models --help' to see available commands.
    """
    # Validate log level
    valid_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
    if log_level.upper() not in valid_levels:
        typer.echo(f"Invalid log level '{log_level}'. Valid options: {', '.join(valid_levels)}")
        raise typer.Exit(1)

    # Initialize global logger and manager
    from .utils.logging import initialize
    initialize(silent=silent, log_level=log_level)

    # Store flags in context for subcommands (for backward compatibility)
    ctx.ensure_object(dict)
    ctx.obj['silent'] = silent
    ctx.obj['log_level'] = log_level


if __name__ == "__main__":
    app()
