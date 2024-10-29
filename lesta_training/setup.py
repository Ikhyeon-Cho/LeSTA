# setup.py

from setuptools import setup, find_packages

setup(
    name="LeSTA",
    version="0.1",
    author="Ikhyeon Cho",
    author_email="tre0430@korea.ac.kr",
    url="https://github.com/Ikhyeon-Cho/LeSTA",
    packages=find_packages(),
    install_requires=[],  # requirements.txt
    description="Learning Self-supervised Traversability with Navigation Experiences of Mobile Robots: A Risk-aware Self-training Approach",
    python_requires='>=3.6',
    include_package_data=True,
)
