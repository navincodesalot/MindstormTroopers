import os
import subprocess
from glob import glob
from shutil import which, copyfile, rmtree

def compileVideo(folder: str, name: str):
    ffmpeg_path = which('ffmpeg')
    if ffmpeg_path is None:
        raise Exception("Error: 'ffmpeg' is not installed or not in the system's PATH.")
    if not os.path.exists(folder):
        raise Exception("Target folder not be found")

    img_ext = ['png', 'jpg', 'jpeg']
    images = []
    for extension in img_ext:
        print(glob(folder + f'/**/*.{extension}', recursive=True))
        images += glob(folder + f'/**/*.{extension}', recursive=True)
    os.mkdir('temp')

    i = 0
    for imgName in images:
        copyfile(imgName, os.path.join('temp', f'img_{i:06d}.jpg'))
        i += 1

    process = subprocess.Popen(f'ffmpeg -framerate 60 -i img_%06d.jpg {name}',
                               shell=True, cwd=os.path.join(os.getcwd(), 'temp'))
    process.wait()
    rmtree('temp')

if __name__ == '__main__':
    print(__file__)
    if os.path.exists('temp'):
        rmtree('temp')

    compileVideo('.', "../RedProp.mp4")