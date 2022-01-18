import setuptools
from itertools import chain


# The basic requirements needed for each type of machine
install_requires = [
    "mysql.connector",
    "robomaster",
    "opencv-python>=4.2",
    "numpy",
]

# Requirements per options
extras = {
    "experiments": [
        "imutils==0.5.4",
        "Pillow",
        "matplotlib",
        "keyboard",
        "MyQR",
    ],
    "ai_trainer": [
        "pettingzoo[butterfly]<=1.5.1",
        "stable-baselines3",
        "gym",
        "supersuit==2.4.0",
        "shapely",
        "wandb",
        "tensorboard",
    ],
    "game_leader": [
        "flask",
        "flask-sqlalchemy"
    ],
    "robot": []
}
# Overarching requirements
extras['all'] = list(chain.from_iterable(extras.values()))
extras['dev'] = extras['all'] + [
    "flake8",
    "flake8-blind-except",
    "flake8-builtins",
    "flake8-docstrings",
    "flake8-logging-format",
    "pytest"
]

setuptools.setup(
    name='robotwars',
    description='Robomaster S1 autonomous lasergaming for HU',
    version='0.0.1',
    package_dir={'': 'src'},
    packages=setuptools.find_packages('src'),
    install_requires=install_requires,
    extras_require=extras,
    python_requires='>=3.6.6, <=3.8.9',
    keywords=['robomaster', 'robomaster s1', 'lasergame', 's1', 'autonomous', 'reinforcement learning', 'dji'],
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    include_package_data=True,
)
