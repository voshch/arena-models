import os

import typer

from arena_models.cli.utils import safe_echo


def fetch_command(
    ctx: typer.Context,
    relative_path: str = typer.Argument("", help="Path inside the bucket to download"),
    output_dir: str = typer.Option(
        os.getcwd(),
        "--output",
        "-o",
        help="Directory to download the files to"
    )
):
    """Fetch specific files from a path within the bucket."""
    source = ctx.obj.get('source') if ctx.obj else "TODO"

    safe_echo(f"Fetching '{relative_path}' from bucket: {source}", ctx)
    safe_echo(f"Output directory: {output_dir}", ctx)

    from arena_models.impl.fetch import fetch_database
    fetch_database(bucket=source, bucket_path=relative_path, destination=output_dir, relative_path=relative_path)

    safe_echo("Fetch completed successfully!", ctx)


def add_to_cmd(cmd):
    """Add fetch command to the network command group."""
    cmd.command("fetch")(fetch_command)


if __name__ == '__main__':
    typer.run(fetch_command)
