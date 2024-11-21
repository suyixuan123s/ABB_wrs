'''
该文件库集成了YOLOv9和WRS的要求库。

(wrs) C:\Users\21344>conda list
# packages in environment at D:\Anaconda3\envs\wrs:
#
# Name                    Version                   Build  Channel
asttokens                 2.4.1                    pypi_0    pypi
attrs                     23.2.0                   pypi_0    pypi
backcall                  0.2.0                    pypi_0    pypi
blinker                   1.8.2                    pypi_0    pypi
ca-certificates           2024.7.2             haa95532_0    defaults
certifi                   2024.7.4                 pypi_0    pypi
charset-normalizer        3.3.2                    pypi_0    pypi
click                     8.1.7                    pypi_0    pypi
colorama                  0.4.6                    pypi_0    pypi
comm                      0.2.2                    pypi_0    pypi
configargparse            1.7                      pypi_0    pypi
contourpy                 1.1.1                    pypi_0    pypi
cycler                    0.12.1                   pypi_0    pypi
cython                    3.0.10                   pypi_0    pypi
dash                      2.17.1                   pypi_0    pypi
dash-core-components      2.0.0                    pypi_0    pypi
dash-html-components      2.0.0                    pypi_0    pypi
dash-table                5.0.0                    pypi_0    pypi
decorator                 5.1.1                    pypi_0    pypi
executing                 2.0.1                    pypi_0    pypi
fastjsonschema            2.20.0                   pypi_0    pypi
flask                     3.0.3                    pypi_0    pypi
fonttools                 4.53.1                   pypi_0    pypi
grpcio                    1.65.1                   pypi_0    pypi
grpcio-tools              1.65.1                   pypi_0    pypi
idna                      3.7                      pypi_0    pypi
importlib-metadata        8.2.0                    pypi_0    pypi
importlib-resources       6.4.0                    pypi_0    pypi
ipython                   8.12.3                   pypi_0    pypi
ipywidgets                8.1.3                    pypi_0    pypi
itsdangerous              2.2.0                    pypi_0    pypi
jedi                      0.19.1                   pypi_0    pypi
jinja2                    3.1.4                    pypi_0    pypi
joblib                    1.4.2                    pypi_0    pypi
jsonschema                4.23.0                   pypi_0    pypi
jsonschema-specifications 2023.12.1                pypi_0    pypi
jupyter-core              5.7.2                    pypi_0    pypi
jupyterlab-widgets        3.0.11                   pypi_0    pypi
kiwisolver                1.4.5                    pypi_0    pypi
libffi                    3.4.4                hd77b12b_1    defaults
markupsafe                2.1.5                    pypi_0    pypi
matplotlib                3.7.5                    pypi_0    pypi
matplotlib-inline         0.1.7                    pypi_0    pypi
nbformat                  5.10.4                   pypi_0    pypi
nest-asyncio              1.6.0                    pypi_0    pypi
networkx                  3.1                      pypi_0    pypi
numpy                     1.24.4                   pypi_0    pypi
open3d                    0.18.0                   pypi_0    pypi
opencv-contrib-python     4.10.0.84                pypi_0    pypi
opencv-python             4.10.0.84                pypi_0    pypi
openssl                   3.0.14               h827c3e9_0    defaults
packaging                 24.1                     pypi_0    pypi
panda3d                   1.10.14                  pypi_0    pypi
pandas                    2.0.3                    pypi_0    pypi
parso                     0.8.4                    pypi_0    pypi
pickleshare               0.7.5                    pypi_0    pypi
pillow                    10.4.0                   pypi_0    pypi
pip                       24.0             py38haa95532_0    defaults
pkgutil-resolve-name      1.3.10                   pypi_0    pypi
platformdirs              4.2.2                    pypi_0    pypi
plotly                    5.23.0                   pypi_0    pypi
prompt-toolkit            3.0.47                   pypi_0    pypi
protobuf                  5.27.2                   pypi_0    pypi
pure-eval                 0.2.3                    pypi_0    pypi
pygments                  2.18.0                   pypi_0    pypi
pyparsing                 3.1.2                    pypi_0    pypi
pyrealsense2              2.55.1.6486              pypi_0    pypi
pyserial                  3.5                      pypi_0    pypi
python                    3.8.19               h1aa4202_0    defaults
python-dateutil           2.9.0.post0              pypi_0    pypi
pytz                      2024.1                   pypi_0    pypi
pywin32                   306                      pypi_0    pypi
pyyaml                    6.0.1                    pypi_0    pypi
referencing               0.35.1                   pypi_0    pypi
requests                  2.32.3                   pypi_0    pypi
retrying                  1.3.4                    pypi_0    pypi
rpds-py                   0.19.1                   pypi_0    pypi
rtree                     1.3.0                    pypi_0    pypi
scikit-learn              1.3.2                    pypi_0    pypi
scipy                     1.10.1                   pypi_0    pypi
setuptools                69.5.1           py38haa95532_0    defaults
shapely                   2.0.5                    pypi_0    pypi
six                       1.16.0                   pypi_0    pypi
sqlite                    3.45.3               h2bbff1b_0    defaults
stack-data                0.6.3                    pypi_0    pypi
tenacity                  8.5.0                    pypi_0    pypi
threadpoolctl             3.5.0                    pypi_0    pypi
traitlets                 5.14.3                   pypi_0    pypi
typing-extensions         4.12.2                   pypi_0    pypi
tzdata                    2024.1                   pypi_0    pypi
urllib3                   2.2.2                    pypi_0    pypi
vc                        14.2                 h2eaa2aa_4    defaults
vs2015_runtime            14.29.30133          h43f2093_4    defaults
wcwidth                   0.2.13                   pypi_0    pypi
werkzeug                  3.0.3                    pypi_0    pypi
wheel                     0.43.0           py38haa95532_0    defaults
widgetsnbextension        4.0.11                   pypi_0    pypi
zipp                      3.19.2                   pypi_0    pypi




'''