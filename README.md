# teo-bullet-models

Models and useful scripts for the usage of TEO robot in bullet and pybullet. If you are looking for the source code of the robot itself, visit the [teo-main](https://github.com/roboticslab-uc3m/teo-main) repository.

## Installing PyBullet examples

```
pip install pybullet
```

(optional) Install Jupyter Notebooks to test the examples
```
pip install jupyter
```

Eigen
```bash
cd
mkdir -p repos && cd repos
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build_dir
cd build_dir
cmake ..
sudo make install
```

Pytest
```bash
pip install pytest
```

pybind11
```bash
cd
mkdir -p repos && cd repos
git clone https://github.com/pybind/pybind11.git
cd pybind
mkdir build
cd build
cmake ..
sudo make install
```

## Install

Clone the repo

```bash
mkdir -p repos && cd repos
git clone https://github.com/imontesino/pybullet-tests.git
```

Compile the inverse kinematics python module

```bash
cd source
mkdir build
cd build
cmake ..
sudo make install
```

## Usage

Examples of jupyter notebooks are inside the `/notebooks` folder.

## Posting Issues

1. Read [CONTRIBUTING.md](CONTRIBUTING.md)
2. [Post an issue / Feature request / Specific documentation request](https://github.com/roboticslab-uc3m/teo-bullet-models/issues)
