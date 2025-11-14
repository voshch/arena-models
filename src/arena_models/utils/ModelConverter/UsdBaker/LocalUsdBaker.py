from pathlib import Path

from . import UsdBaker

import subprocess


class LocalUsdBaker(UsdBaker):
    """Handle USD baking locally."""

    def _open_subprocess(self) -> subprocess.Popen:
        return subprocess.Popen(
            [
                str(self._isaacsim_path),
                str(self.converter_script()),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def __init__(
        self,
        input_dir: Path,
        output_dir: Path,
        isaacsim_path: Path,
    ):
        super().__init__(input_dir, output_dir)
        self._isaacsim_path = isaacsim_path.expanduser().resolve()
        self._process = None
        self._process = self._open_subprocess()
        self.wait_for_ready()

    def convert(self, input_file: str, output_file: str) -> bool:
        input_full_path = self.input_dir / input_file
        if not input_full_path.exists():
            raise FileNotFoundError(f"Input file does not exist: {input_full_path}")

        output_full_path = self.output_dir / output_file
        output_full_path.parent.mkdir(parents=True, exist_ok=True)

        self.logger.info("Converting %s to %s...", input_full_path, output_full_path)

        cmd = f"{input_full_path}:{output_full_path}"
        output = self.command(cmd)

        if "error:" in output.lower():
            self.logger.error("Conversion failed: %s", output)
            return False

        self.logger.info("Conversion succeeded.")
        return True
