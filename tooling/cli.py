import sys

import typer

from tooling.commands import (
    benchmark,
    build,
    gen_nmpc_lib,
    lint,
    test,
)

app = typer.Typer(no_args_is_help=True, help="AP-PnC tailsitter project CLI.")

app.add_typer(
    gen_nmpc_lib.app, name="gen-nmpc-lib", help="Generate acados NMPC code and bundle lib"
)
app.add_typer(benchmark.app, name="benchmark", help="Planning benchmark experiments")
app.add_typer(test.app, name="test", help="Test runners (pytest, colcon)")
app.add_typer(build.app, name="build", help="C++ colcon builds")
app.add_typer(lint.app, name="lint", help="Lint and format")


def main() -> None:
    from tooling.observability import operation, session

    commands = {
        "start",
        "gen-nmpc-lib",
        "benchmark",
        "test",
        "build",
        "lint",
    }
    command = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in commands else "ap-pnc"
    with session(), operation("cli.command", {"ap.pnc.command": command}):
        app()
