import typer
from pathlib import Path


def merge_command(
    path_database1: str = typer.Option(..., "--path-database1", help="Path to first database"),
    path_database2: str = typer.Option(..., "--path-database2", help="Path to second database"),
    path_outputdb: str = typer.Option(..., "--path-outputdb", help="Path to output database"),
):
    """Merge multiple databases into one."""
    from arena_models.impl.merge import merge_database
    merge_database(path_database1, path_database2, path_outputdb)


def add_to_cmd(cmd):
    """Add merge command to the database command group."""
    @cmd.command("merge")
    def merge_db(
        database1: Path = typer.Option(
            ...,
            "--db1",
            help="Path to first database directory",
            exists=True,
            file_okay=False,
            dir_okay=True
        ),
        database2: Path = typer.Option(
            ...,
            "--db2",
            help="Path to second database directory",
            exists=True,
            file_okay=False,
            dir_okay=True
        ),
        output: Path = typer.Option(
            ...,
            "--output",
            "-o",
            help="Output directory for merged database"
        ),
    ):
        """Merge two databases into one."""
        typer.echo(f"Merging databases {database1} and {database2} into {output}")

        merge_command(str(database1), str(database2), str(output))
        typer.echo("Database merge completed successfully!")


if __name__ == '__main__':
    typer.run(merge_command)
