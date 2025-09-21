import typer
from arena_models.utils.enums import AssetType


def build_command(
    buildtype: str = typer.Argument(..., help="Type of asset to build"),
    input_path: str = typer.Option(..., "-i", "--input-path", help="Input dataset path"),
    output_path: str = typer.Option(..., "-o", "--output-path", help="Output database path"),
    procthor: bool = typer.Option(False, "--procthor", help="Enable procthor export")
):
    """Build a database from source files."""
    from arena_models.impl.build import DatabaseBuilder
    try:
        buildtype_enum = AssetType[buildtype.upper()]
    except KeyError:
        valid_types = [e.name for e in AssetType]
        typer.echo(f"Invalid buildtype '{buildtype}'. Valid options: {', '.join(valid_types)}")
        raise typer.Exit(1)

    builder = DatabaseBuilder.Builder(buildtype_enum)(input_path, output_path)
    if procthor:
        builder.procthor()
    builder.build()


def add_to_cmd(cmd):
    """Add build command to the main app."""
    @cmd.command("build")
    def build_db(
        buildtype: str = typer.Argument(
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
        typer.echo(f"Building {buildtype} database from {input_path} to {output_path}")
        if procthor:
            typer.echo("ProcTHOR export enabled")

        build_command(buildtype, input_path, output_path, procthor)
        typer.echo("Database build completed successfully!")


if __name__ == '__main__':
    typer.run(build_command)
