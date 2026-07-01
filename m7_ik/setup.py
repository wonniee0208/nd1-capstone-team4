from setuptools import setup, find_packages

setup(
    name="nd1_m7_ik",
    version="1.1.0",
    description="ND1 M7 모듈 — 3-DOF 평면 로봇 FK/IK/Jacobian/Manipulability 패키지",
    author="ND1 피지컬 AI 전문가 과정",
    python_requires=">=3.8",
    packages=find_packages(exclude=("tests", "tests.*")),
    install_requires=[
        "numpy>=1.21",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Topic :: Scientific/Engineering",
        "Intended Audience :: Education",
    ],
)
