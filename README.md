# Pose and camera geometry

Welcome to this lab in the computer vision course [TEK5030] at the University of Oslo.

In this lab we will experiment with poses in different coordinate frames together with the perspective camera model, using real data taken from a helicopter flying around Holmenkollen.

![Holmenkollen in 3D with camera trajectory](lab-guide/img/holmenkollen-3d.png)

Start by cloning this repository on your machine.
Then, open the lab project in your editor.

The lab is carried out by following these steps:

1. [Get an overview](lab-guide/1-get-an-overview.md)
2. [From geographical coordinates to pixels](lab-guide/2-from-geographical-coordinates-to-pixels.md)

Please start the lab by going to the [first step](lab-guide/1-get-an-overview.md).

## Prerequisites

Here is a quick reference if you need to set up a Python virtual environment manually:

```bash
python3.8 -m venv venv  # any python version > 3.8 is OK
source venv/bin/activate.
# expect to see (venv) at the beginning of your prompt.
pip install -U pip  # <-- Important step for Ubuntu 18.04!
pip install -r requirements.txt
```

Please consult the [resource pages] if you need more help with the setup.

[TEK5030]: https://www.uio.no/studier/emner/matnat/its/TEK5030/
[resource pages]: https://tek5030.github.io
