# converter.py
import sys
import asyncio

# fmt: off
if __name__ == "__main__":
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "no_window": True})

import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.kit.asset_converter", True)
ext_manager.set_extension_enabled_immediate("omni.mdl.distill_and_bake", True)

import omni.usd
import omni.kit.commands
import omni.mdl.distill_and_bake
import omni.kit.asset_converter
from pxr import Usd, UsdShade, Sdf
# fmt: on


async def distill(input_path, output_path):
    context = omni.usd.get_context()

    success = context.open_stage(input_path)
    if not success:
        raise RuntimeError(f"Failed to open stage: {input_path}")
    stage = context.get_stage()

    materials_to_process = []
    for prim in stage.Traverse():
        if prim.IsA(UsdShade.Material):
            materials_to_process.append(prim)

    for mat_prim in materials_to_process:
        mat_path = mat_prim.GetPath().pathString

        try:
            distiller = omni.mdl.distill_and_bake.MdlDistillAndBake(mat_prim)
            await distiller.distill_async()
        except Exception as e:
            print(f"[Warning] Failed to distill material at {mat_path}: {e}", file=sys.stderr)

    context.save_stage()

    converter_manager = omni.kit.asset_converter.get_instance()
    context_options = omni.kit.asset_converter.AssetConverterContext()

    input_stage_path = stage.GetRootLayer().identifier

    task = converter_manager.create_converter_task(
        input_stage_path, output_path, None, context_options
    )

    # Wait for the export task to finish
    success = await task.wait_until_finished()

    if not success:
        raise RuntimeError(f"Failed to convert: {input_path} -> {output_path}")

if __name__ == "__main__":
    print('ready: Converter CLI. Type "exit" to quit.')
    while True:
        inp = input('')
        if inp.strip().lower() == 'exit':
            break
        args = inp.strip().split(':')
        if len(args) != 2:
            print("error: usage <input_file>:<output_file>")
            continue
        try:
            loop = asyncio.get_event_loop()
            r = loop.run_until_complete(distill(args[0], args[1]))
            print(f"success: {r}")
        except Exception as e:
            print(f"error: {e}")
