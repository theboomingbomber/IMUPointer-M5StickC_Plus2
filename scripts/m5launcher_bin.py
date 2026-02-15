from pathlib import Path
import shutil
import subprocess

Import("env")


def _package_file(package_name, *relative_path):
    package_dir = env.PioPlatform().get_package_dir(package_name)
    if not package_dir:
        raise RuntimeError(f"Missing PlatformIO package: {package_name}")
    return Path(package_dir, *relative_path)


def export_launcher_bins(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    project_dir = Path(env.subst("$PROJECT_DIR"))
    pioenv = env.subst("$PIOENV")

    dist_dir = project_dir / "dist"
    dist_dir.mkdir(parents=True, exist_ok=True)
    for legacy_file in (
        dist_dir / f"{pioenv}-app.bin",
        dist_dir / f"{pioenv}-m5launcher-merged.bin",
    ):
        if legacy_file.exists():
            legacy_file.unlink()

    app_bin = build_dir / "firmware.bin"
    if not app_bin.exists():
        raise RuntimeError(f"Missing app firmware: {app_bin}")

    launcher_app = dist_dir / f"{pioenv}-for-m5launcher.bin"
    shutil.copy2(app_bin, launcher_app)

    bootloader_bin = build_dir / "bootloader.bin"
    partitions_bin = build_dir / "partitions.bin"
    boot_app0_bin = _package_file(
        "framework-arduinoespressif32", "tools", "partitions", "boot_app0.bin"
    )
    python_exe = env.subst("$PYTHONEXE")

    merged_bin = dist_dir / f"{pioenv}-full-flash-0x0000.bin"
    subprocess.run(
        [
            python_exe,
            "-m",
            "esptool",
            "--chip",
            "esp32",
            "merge-bin",
            "-o",
            str(merged_bin),
            "0x1000",
            str(bootloader_bin),
            "0x8000",
            str(partitions_bin),
            "0xe000",
            str(boot_app0_bin),
            "0x10000",
            str(app_bin),
        ],
        check=True,
    )

    print(f"[m5launcher] app bin: {launcher_app}")
    print(f"[m5launcher] merged bin: {merged_bin}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", export_launcher_bins)
