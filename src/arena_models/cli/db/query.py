"""Query command for database operations."""

import os

import typer

from ...impl import AssetType
from ..utils import safe_echo


def query_command(
    ctx: typer.Context,
    asset_type: str = typer.Argument(
        ...,
        help=f"Type of asset to query. Options: {', '.join([e.name.lower() for e in AssetType])}",
    ),
    query_text: str = typer.Argument(..., help="Text description to search for"),
    count: int = typer.Option(1, "--count", "-n", min=1, help="Number of results to return"),
    scores: bool = typer.Option(False, "--scores", help="Append the distance score to each result"),
):
    """Search for models in the database using natural language."""
    database_path = ctx.obj.get("database_path") if ctx.obj else None
    if not database_path:
        safe_echo("Error: Database path not found in context", ctx)
        raise typer.Exit(1)

    try:
        asset_type_enum = AssetType[asset_type.upper()]
    except KeyError:
        valid_types = [e.name for e in AssetType]
        safe_echo(
            f"Invalid asset_type '{asset_type}'. Valid options: {', '.join(valid_types)}",
            ctx,
        )
        raise typer.Exit(1) from None

    safe_echo(
        f"Searching for {asset_type_enum.name} '{query_text}' in database {database_path}",
        ctx,
    )

    from arena_models.impl.query import query_database

    results = query_database(database_path=database_path, asset_type=asset_type_enum, query_target=query_text, n=count)
    for annotation, score in results:
        line = os.path.join(database_path, annotation.path)
        if scores:
            line += f"\t{score}"
        typer.echo(line)


def add_to_cmd(db_cmd: typer.Typer):
    db_cmd.command("query")(query_command)


if __name__ == "__main__":
    typer.run(query_command)
