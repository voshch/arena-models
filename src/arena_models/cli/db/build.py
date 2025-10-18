import typer
from pathlib import Path

from arena_models.impl import AssetType
from arena_models.utils.logging import get_logger, get_manager

logger = get_logger('cli.db.build')


def build_command(
    ctx: typer.Context,
    asset_type: list[str] = typer.Option(
        [],
        "--buildtype",
        "-t",
        help="Type of asset to build. Optional, defaults to all types. Specify multiple times for multiple types.",
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
    options: list[str] = typer.Option(
        [],
        "--option",
        "-o",
        help="Options for the build process",
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
    if asset_type:
        try:
            asset_type_enum = [AssetType[t.upper()] for t in asset_type]
        except KeyError:
            valid_types = [e.name for e in AssetType]
            logger.fatal(f"Invalid asset_type '{asset_type}'. Valid options: {', '.join(valid_types)}")
            raise typer.Exit(1)
    else:
        asset_type_enum = list(DatabaseBuilder.get_registered())

    logger.info(f"Starting database build from {input_path} to {output_path} with types: {', '.join([t.name for t in asset_type_enum])}")
    manager = get_manager()
    _ = manager.status_bar()  # bug in display removes the progress bar at the end otherwise
    global_progress = manager.counter(
        total=len(asset_type_enum),
        desc="Databases",
        bar_format='{desc}{desc_pad}{count:{len_total}d}/{total:d} [{elapsed}]',
        autorefresh=True,
    )
    with global_progress:
        for t in global_progress(asset_type_enum):
            logger.info(f"Building {t} database from {input_path} to {output_path}")

            # Build the database
            builder = DatabaseBuilder.Builder(t)(input_path=str(input_path), output_path=str(output_path), overwrite=overwrite)
            for extra in options:
                builder.enable(extra)
            builder.build()

    logger.info(f"Database build completed in {global_progress.elapsed}s")


def add_to_cmd(db_cmd):
    db_cmd.command("build")(build_command)


if __name__ == '__main__':
    typer.run(build_command)
