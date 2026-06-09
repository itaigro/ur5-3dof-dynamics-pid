from setuptools import setup

setup(
    name="ur5_3dof",
    version="0.1.0",
    package_dir={"": "src"},
    packages=["ur5_3dof"],
    install_requires=["numpy>=1.24", "scipy>=1.10", "sympy>=1.12", "matplotlib>=3.7"],
)
