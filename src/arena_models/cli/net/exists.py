import os

import typer

from arena_models.cli.utils import safe_echo


def exists_command(
    ctx: typer.Context,
    assets: list[str] = typer.Argument([], help="Path inside the bucket to download"),
):
    """Test if files from a path within the bucket exist."""
    source = ctx.obj['source']

    from arena_models.impl.fetch import Bucket
    safe_echo(f"Checking existence of {len(assets)} assets in bucket: {source}", ctx)
    results = Bucket(source).assets_exist(assets)
    for asset, exists in results:
        safe_echo(f"Checking existence of '{asset}' in bucket: {source}", ctx)
        print(1 if exists else 0)
        safe_echo(f"Asset {'exists' if exists else 'does not exist'} in {source}!", ctx)
    safe_echo("Existence check completed successfully!", ctx)


def add_to_cmd(cmd):
    """Add exists command to the network command group."""
    cmd.command("exists")(exists_command)


if __name__ == '__main__':
    typer.run(exists_command)
