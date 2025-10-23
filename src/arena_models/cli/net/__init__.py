"""Network command group for arena_models CLI."""

import typer

from .fetch import add_to_cmd as add_fetch_cmd
from .exists import add_to_cmd as add_exists_cmd


def add_to_cmd(cmd):
    """Add network command group to the main CLI."""

    net_app = typer.Typer(
        name="net",
        help="Network operations for arena models",
        rich_markup_mode="rich",
        no_args_is_help=True,
    )

    @net_app.callback()
    def net_callback(
        ctx: typer.Context,
        source: str = typer.Argument(help="Network source to use"),
    ):
        """Network operations with specified source."""
        if ctx:
            ctx.ensure_object(dict)
            if source == 'default':
                source = "arena-models-prod-public"
            ctx.obj['source'] = source
            # Inherit silent flag from parent context
            if ctx.parent and ctx.parent.obj:
                ctx.obj['silent'] = ctx.parent.obj.get('silent', False)

    add_fetch_cmd(net_app)
    add_exists_cmd(net_app)

    cmd.add_typer(net_app, name="net")
