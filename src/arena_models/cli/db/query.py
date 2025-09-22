"""Query command for database operations."""

import typer
from typing import Optional
from ...impl import AssetType
from ..utils import safe_echo


def query_command(
    asset_type: str,
    query_target: str,
    path_database: str,
    ctx: Optional[typer.Context] = None,
):
    """Query the database for models."""
    try:
        asset_type_enum = AssetType[asset_type.upper()]
    except KeyError:
        valid_types = [e.name for e in AssetType]
        safe_echo(f"Invalid asset_type '{asset_type}'. Valid options: {', '.join(valid_types)}", ctx)
        raise typer.Exit(1)

    from ...impl.query import query_database
    return query_database(database_path=path_database, asset_type=asset_type_enum, query_target=query_target)


def add_to_db_cmd(db_cmd):
    """Add query command to the database command group."""
    @db_cmd.command("query")
    def query_model(
        ctx: typer.Context,
        asset_type: str = typer.Argument(
            ...,
            help=f"Type of asset to query. Options: {', '.join([e.name.lower() for e in AssetType])}"
        ),
        query_text: str = typer.Argument(..., help="Text description to search for"),
    ):
        """Search for models in the database using natural language."""
        # Get database path from parent context
        database_path = ctx.obj.get('database_path') if ctx.obj else None
        if not database_path:
            safe_echo("Error: Database path not found in context", ctx)
            raise typer.Exit(1)

        safe_echo(f"Searching for: '{query_text}' in database {database_path}", ctx)

        typer.echo(query_command(asset_type, query_text, database_path, ctx))


if __name__ == '__main__':
    # For direct execution
    def standalone_query(
        asset_type: str = typer.Argument(..., help="Type of asset to query"),
        query_target: str = typer.Argument(..., help="Text to search for in the database"),
        path_database: str = typer.Option(..., "--path-database", help="Path to the database file"),
    ):
        query_command(asset_type, query_target, path_database)

    typer.run(standalone_query)
