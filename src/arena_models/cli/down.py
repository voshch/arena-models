from pathlib import Path
from typing import Optional

import typer


def download_command():
    """Download the database from the remote source."""
    from ..impl.down import download_database
    download_database()


def add_to_cmd(cmd):
    """Add download command to the main app."""
    @cmd.command()
    def download(
        output_dir: Optional[Path] = typer.Option(
            Path.cwd(),
            "--output",
            "-o",
            help="Directory to download the dataset to"
        )
    ):
        """Download the pre-built model database."""
        typer.echo("Downloading arena models database...")
        download_command()


if __name__ == '__main__':
    typer.run(download_command)
