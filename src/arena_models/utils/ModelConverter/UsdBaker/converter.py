# converter.py
# use with isaacsim 5.x
import asyncio
import os
import shutil


# fmt: off
if __name__ == "__main__":
    from isaacsim import SimulationApp
    app = SimulationApp(
        {"headless": True, "no_window": True}
        # {"headless": False, "no_window": False}
    )

import omni.kit.app
# Enable the required extensions for 5.x
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.kit.asset_converter", True)
ext_manager.set_extension_enabled_immediate("omni.mdl.distill_and_bake", True)
ext_manager.set_extension_enabled_immediate("omni.kit.usd.collect", True)

import omni.usd
from pxr import Usd, UsdShade, UsdUtils, UsdGeom, Sdf
from omni.kit.asset_converter import AssetConverterContext, get_instance
from omni.mdl.distill_and_bake import MdlDistillAndBake
from omni.kit.usd.collect import Collector
# fmt: on


def sanitize_broken_ue4_materials(stage):
    """
    Forces the GLB exporter to respect Roughness by setting explicit constants.
    This fixes the 'Mirror Bed' issue in external viewers.
    """
    new_shader_paths = []

    def _clamp01(value):
        if value is None:
            return None
        return max(0.0, min(1.0, float(value)))

    def _get_scalar_input(shader, names):
        for name in names:
            inp = shader.GetInput(name)
            if not inp or not inp.HasAuthoredValueOpinion():
                continue
            val = inp.Get()
            if isinstance(val, (int, float)):
                return float(val)
        return None

    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Material):
            continue
        material = UsdShade.Material(prim)
        mdl_output = material.GetSurfaceOutput("mdl")
        if not mdl_output:
            continue
        source = mdl_output.GetConnectedSource()
        if not source:
            continue
        old_shader = UsdShade.Shader(source[0])

        if old_shader.GetInput("RMA"):
            albedo_val = old_shader.GetInput("Albedo").Get()
            normal_val = old_shader.GetInput("Normal").Get()
            rma_val = old_shader.GetInput("RMA").Get()

            # Try to extract original constants
            roughness_const = _clamp01(
                _get_scalar_input(
                    old_shader,
                    ["Roughness", "roughness", "RoughnessFactor", "reflection_roughness_constant"],
                )
            )
            metallic_const = _clamp01(
                _get_scalar_input(
                    old_shader,
                    ["Metallic", "metallic", "MetallicFactor", "metallic_constant"],
                )
            )
            if roughness_const is None:
                roughness_const = 0.8
            if metallic_const is None:
                metallic_const = 0.0

            new_shader_path = prim.GetPath().AppendChild("OmniPBR_Fix")
            new_shader = UsdShade.Shader.Define(stage, new_shader_path)
            new_shader.CreateIdAttr("OmniPBR.mdl")
            new_shader.SetSourceAsset("OmniPBR.mdl", "mdl")
            new_shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")

            # Albedo & Normal
            if albedo_val:
                new_shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(albedo_val)
            if normal_val:
                new_shader.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(normal_val)

            if rma_val:
                # KEY FIX: Don't provide texture inputs for roughness/metallic
                # Only set constants, forcing the exporter to bake flat values
                new_shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(roughness_const)
                new_shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(metallic_const)

                # Still use AO channel if present (Red channel in RMA)
                new_shader.CreateInput("ao_texture", Sdf.ValueTypeNames.Asset).Set(rma_val)

            mdl_output.ConnectToSource(new_shader.ConnectableAPI(), "surface")
            new_shader_paths.append(new_shader_path)

    if new_shader_paths:
        stage.GetRootLayer().Save()
    return new_shader_paths


async def convert_asset(input_path, output_path):
    import tempfile
    context = omni.usd.get_context()

    base_name = os.path.splitext(os.path.basename(input_path))[0]
    tmp_dir = tempfile.mkdtemp(prefix=f"usdbaker_{base_name}_")

    try:
        # USDZ files fail with collector - open directly instead
        if input_path.lower().endswith('.usdz'):
            context.open_stage(input_path)
            stage = context.get_stage()
            tmp_usd_path = os.path.join(tmp_dir, f"{base_name}.usd")
            stage.Export(tmp_usd_path)
        else:
            collector = Collector(
                usd_path=input_path,
                collect_dir=tmp_dir,
                flat_collection=True,
            )
            success, result = await collector.collect()
            if not success:
                raise RuntimeError(f"Failed to collect assets: {result}")
            tmp_usd_path = result

        context.open_stage(tmp_usd_path)
        stage = context.get_stage()

        # Repair the materials before distilling
        sanitize_broken_ue4_materials(stage)

        for prim in stage.Traverse():
            if UsdShade.Material(prim):
                distiller = MdlDistillAndBake(prim, ouput_folder=tmp_dir)
                distiller.distill()

        stage.Export(tmp_usd_path)

        converter_manager = get_instance()

        glb_options = AssetConverterContext()
        glb_options.embed_textures = True
        glb_options.export_preview_surface = True
        glb_options.use_meter_per_unit = True
        glb_options.fmerge_all_meshes = False

        glb_task = converter_manager.create_converter_task(
            tmp_usd_path, output_path, None, glb_options
        )

        success = await glb_task.wait_until_finished()
        if not success:
            raise RuntimeError(f"Failed to export glb: {glb_task.get_error_message()}")

        return True
    finally:
        shutil.rmtree(tmp_dir)

if __name__ == "__main__":
    app = omni.kit.app.get_app()
    loop = asyncio.get_event_loop()
    print('ready: Converter CLI (Isaac Sim 5.x). Type "exit" to quit.')
    while True:
        inp = input('')
        if inp.strip().lower() == 'exit':
            break
        args = inp.strip().split(':')
        if len(args) != 2:
            print("error: usage <input_file>:<output_file>")
            continue
        try:
            task = loop.create_task(convert_asset(args[0], args[1]))
            while not task.done():
                app.update()
                loop.run_until_complete(asyncio.sleep(0))
            if (exc := task.exception()):
                raise exc
            print(f"success: {task.result()}")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"error: {repr(e)}")
