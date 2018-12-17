import subprocess
import os


if __name__ == "__main__":
    SCENES_DIR = "../examples/scenes"
    EXEC = "./src/GooBallsApp/gooBalls"
    SCENES_DIR = os.path.abspath(SCENES_DIR)
    EXEC = os.path.abspath(EXEC)
    scenes = [f for f in os.listdir(SCENES_DIR) if os.path.isfile(os.path.join(SCENES_DIR, f))]
    scenes.sort()
    for scene in scenes:
        print(scene)
        subprocess.call([EXEC, os.path.join(SCENES_DIR, scene), "--scene-max-seconds", "3"])

