import typer
from pathlib import Path

from arena_models.cli.utils import safe_echo
from arena_models.impl import AssetType


def build_command(
    ctx: typer.Context,
    asset_type: str = typer.Option(
        ...,
        "--buildtype",
        "-t",
        help=f"Type of asset to build. Options: {', '.join([e.name.lower() for e in AssetType])}"
    ),
    input_path: Path = typer.Option(
        ...,
        "--input",
        "-i",
        help="Input dataset directory path",
        file_okay=False,
        dir_okay=True,
        readable=True,
        exists=True,
    ),
    procthor: bool = typer.Option(
        False,
        "--procthor",
        help="Enable ProcTHOR export format"
    ),
    formats: list[str] = typer.Option(
        [],
        "--format",
        "-f",
        help="Output formats for the built database"
    ),
    overwrite: bool = typer.Option(
        False,
        "--overwrite",
        help="Overwrite existing files",
        is_flag=True
    ),
):
    """Build a database from source model files."""
    from arena_models.impl.build import DatabaseBuilder

    # Get output path from context
    output_path = ctx.obj.get('database_path') if ctx.obj else None

    # Validate asset type
    try:
        asset_type_enum = AssetType[asset_type.upper()]
    except KeyError:
        valid_types = [e.name for e in AssetType]
        safe_echo(f"Invalid asset_type '{asset_type}'. Valid options: {', '.join(valid_types)}", ctx)
        raise typer.Exit(1)

    # Log build start
    safe_echo(f"Building {asset_type} database from {input_path} to {output_path}", ctx)
    if procthor:
        safe_echo("ProcTHOR export enabled", ctx)

    # Build the database
    builder = DatabaseBuilder.Builder(asset_type_enum)(input_path=str(input_path), output_path=str(output_path), formats=formats, overwrite=overwrite)
    if procthor:
        builder.procthor()
    builder.build()

    safe_echo("Database build completed successfully!", ctx)


if __name__ == '__main__':
    typer.run(build_command)
