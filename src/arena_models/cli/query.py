import typer
from pathlib import Path
from typing import Optional


def query_command(
    query_target: str = typer.Argument(..., help="Text to search for in the database"),
    path_database: str = typer.Option(..., "--path-database", help="Path to the database file"),
    name_database: str = typer.Option(..., "--name-database", help="Name of the database"),
    target: str = typer.Option(..., "--target", help="Path to save the configuration file")
):
    """Query the database for models."""
    from arena_models.impl.query import query_database
    query_database(query_target, path_database, name_database, target)


def add_to_cmd(cmd):
    """Add query command to the database command group."""
    @cmd.command("query")
    def query_model(
        query_text: str = typer.Argument(..., help="Text description to search for"),
        database_path: Path = typer.Option(
            ...,
            "--database",
            "-d",
            help="Path to database directory",
            exists=True,
            file_okay=False,
            dir_okay=True
        ),
        database_name: str = typer.Option("models", "--name", "-n", help="Name of the database collection"),
        output_file: Optional[Path] = typer.Option(
            None,
            "--output",
            "-o",
            help="File to save query results (default: config_id_file.txt in current dir)"
        )
    ):
        """Search for models in the database using natural language."""
        typer.echo(f"Searching for: '{query_text}' in database {database_path}")

        target_path = str(output_file.parent if output_file else Path.cwd())

        query_command(query_text, str(database_path), database_name, target_path)
        typer.echo("Query completed successfully!")


if __name__ == '__main__':
    typer.run(query_command)
