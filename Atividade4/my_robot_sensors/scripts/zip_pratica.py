#!/usr/bin/env python3
import os
import shutil
import zipfile
from pathlib import Path

def find_desktop():
    user = "andre"
    candidates = [
        f"/mnt/c/Users/{user}/Desktop",
        f"/mnt/c/Users/{user}/OneDrive/Desktop",
        f"/mnt/c/Users/{user}/OneDrive/Área de Trabalho",
        f"/mnt/c/Users/{user}/OneDrive - Personal/Desktop",
        f"/mnt/c/Users/{user}",
    ]
    for c in candidates:
        if os.path.isdir(c):
            return c
    return os.path.expanduser("~")

def make_zip():
    desktop = find_desktop()
    target_zip = os.path.join(desktop, "Pratica04_Robotica.zip")
    local_zip = "/home/andre/codigos/robotica/Pratica04_Robotica.zip"

    print(f"Compactando projeto para: {target_zip}")

    exclude_dirs = {".git", "build", "install", "log", "__pycache__", ".pytest_cache"}
    exclude_exts = {".pyc", ".pyo"}

    with zipfile.ZipFile(local_zip, "w", zipfile.ZIP_DEFLATED) as zf:
        # 1. Adicionar pacote my_robot_sensors
        sensors_root = Path("/home/andre/codigos/robotica")
        for root, dirs, files in os.walk(sensors_root):
            dirs[:] = [d for d in dirs if d not in exclude_dirs]
            for f in files:
                if f == "Pratica04_Robotica.zip" or any(f.endswith(ext) for ext in exclude_exts):
                    continue
                full_path = Path(root) / f
                arcname = Path("my_robot_sensors") / full_path.relative_to(sensors_root)
                zf.write(full_path, arcname)

        # 2. Adicionar pacote my_robot_control se existir
        control_root = Path("/home/andre/ros2_ws/src/my_robot_control")
        if control_root.exists() and control_root.is_dir():
            for root, dirs, files in os.walk(control_root):
                dirs[:] = [d for d in dirs if d not in exclude_dirs]
                for f in files:
                    if any(f.endswith(ext) for ext in exclude_exts):
                        continue
                    full_path = Path(root) / f
                    arcname = Path("my_robot_control") / full_path.relative_to(control_root)
                    zf.write(full_path, arcname)

    # Copiar para o Desktop do Windows
    if desktop != "/home/andre/codigos/robotica":
        shutil.copy2(local_zip, target_zip)
        print(f"Arquivo salvo com sucesso na Área de Trabalho do Windows: {target_zip}")
    print(f"Cópia local disponível em: {local_zip}")

if __name__ == "__main__":
    make_zip()
