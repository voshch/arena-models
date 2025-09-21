import typer
from pathlib import Path
from typing import Optional


def get_command(
    item_id: str = typer.Option(..., "--id", help="The specific id to get from the database"),
    path_database: str = typer.Option(..., "--path-database", help="Path to the database file"),
    name_database: str = typer.Option(..., "--name-database", help="Name of the database"),
    target_path: str = typer.Option(..., "--target-path", help="Path to save the configuration file")
):
    """Get a model from the database by ID."""
    from arena_models.impl.get import get_database
    get_database(path_database, name_database, target_path, item_id)


def add_to_cmd(cmd):
    """Add get command to the database command group."""
    @cmd.command("get")
    def get_model(
        model_id: str = typer.Option(..., "--id", help="The specific model ID to retrieve"),
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
            help="File to save model information (default: config_result_file.txt in current dir)"
        )
    ):
        """Retrieve a specific model by ID from the database."""
        typer.echo(f"Retrieving model ID: {model_id} from database {database_path}")

        target_path = str(output_file.parent if output_file else Path.cwd())

        get_command(model_id, str(database_path), database_name, target_path)
        typer.echo("Model retrieved successfully!")


if __name__ == '__main__':
    typer.run(get_command)
