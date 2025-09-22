import typer
from typing import Optional
from ..impl import AssetType
from .utils import safe_echo


def build_command(
    asset_type: str = typer.Argument(..., help="Type of asset to build"),
    input_path: str = typer.Option(..., "-i", "--input-path", help="Input dataset path"),
    output_path: str = typer.Option(..., "-o", "--output-path", help="Output database path"),
    procthor: bool = typer.Option(False, "--procthor", help="Enable procthor export"),
    ctx: Optional[typer.Context] = None
):
    """Build a database from source files."""
    from ..impl.build import DatabaseBuilder
    try:
        asset_type_enum = AssetType[asset_type.upper()]
    except KeyError:
        valid_types = [e.name for e in AssetType]
        safe_echo(f"Invalid asset_type '{asset_type}'. Valid options: {', '.join(valid_types)}", ctx)
        raise typer.Exit(1)

    builder = DatabaseBuilder.Builder(asset_type_enum)(input_path, output_path)
    if procthor:
        builder.procthor()
    builder.build()


def add_to_cmd(cmd):
    """Add build command to the main app."""
    @cmd.command("build")
    def build_db(
        ctx: typer.Context,
        asset_type: str = typer.Argument(
            ...,
            help=f"Type of asset to build. Options: {', '.join([e.name.lower() for e in AssetType])}"
        ),
        input_path: str = typer.Option(
            ...,
            "--input",
            "-i",
            help="Input dataset directory path",
            file_okay=False,
            dir_okay=True,
            readable=True
        ),
        output_path: str = typer.Option(
            ...,
            "--output",
            "-o",
            help="Output database directory path"
        ),
        procthor: bool = typer.Option(
            False,
            "--procthor",
            help="Enable ProcTHOR export format"
        )
    ):
        """Build a database from source model files."""
        safe_echo(f"Building {asset_type} database from {input_path} to {output_path}", ctx)
        if procthor:
            safe_echo("ProcTHOR export enabled", ctx)

        build_command(asset_type, input_path, output_path, procthor, ctx)
        safe_echo("Database build completed successfully!", ctx)


if __name__ == '__main__':
    typer.run(build_command)
