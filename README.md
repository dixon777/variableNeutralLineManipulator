

# Introduction

This repo provides a user interface for evaluating and visualising the eventual orientation of a [tendon-driven variable neutral line manipulator](https://ieeexplore.ieee.org/document/6661461?arnumber=6661461 "IEEE") given the input tension on each tendon through a proposed static force model.

This project is part of the research, initiated from Dec 2019, subsidised by the [University of New South Wales Taste of Research program](https://www.engineering.unsw.edu.au/taste-of-research-program).
# Recommended installation
- Conda: 4.8.3
- Python: 3.8.5

# Installation

```bash
git clone https://github.com/dixon777/variableNeutralLineManipulator.git `<CUSTOM_DIR_NAME>`
cd `<CUSTOM_DIR_NAME>`
conda env create -f conda_env.yml
```

# Running

## 1. User Interface
```bash
python <REPO_ROOT_PATH>/gui_main.py # <REPO_ROOT_PATH> is the root directory of the repo
```
