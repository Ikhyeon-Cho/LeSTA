# setup.py

from setuptools import setup, find_packages

setup(
    name="pylesta",
    version="0.2.0",
    author="Ikhyeon Cho",
    author_email="tre0430@korea.ac.kr",
    url="https://github.com/Ikhyeon-Cho/LeSTA",
    packages=find_packages(),
    install_requires=[],  # requirements.txt
    description="Training a risk-aware self-supervised traversability model with navigation experiences of mobile robots",
    python_requires='>=3.6',
    include_package_data=True,
)
