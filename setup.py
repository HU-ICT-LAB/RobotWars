import setuptools

install_requires = [
    "flask",
    "flask-sqlalchemy"
]

extras = {
    "dev": [
        'flake8',
        'flake8-blind-except',
        "flake8-builtins",
        "flake8-docstrings",
        "flake8-logging-format",
        "mypy",
        "pytest"]
}

setuptools.setup(
    name='robotwars',
    description='Robomaster S1 autonomous lasergaming for HU',
    version='0.0.1',
    packages=setuptools.find_packages(),
    install_requires=install_requires,
    extras_require=extras,
    python_requires='>=3.6',
    keyword=['robomaster', 'robomaster s1', 'lasergame', 's1', 'autonomous', 'reinforcement learning', 'dji'],
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    include_package_data=True,
)
