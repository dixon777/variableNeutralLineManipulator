# Introduction
This repo provides a user interface for evaluating and visualising the eventual orientation of a [tendon-driven variable neutral line manipulator](https://ieeexplore.ieee.org/document/6661461?arnumber=6661461 "IEEE") given the input tension on each tendon through a proposed static force model.

This project is part of the research, initiated from Dec 2019, subsidised by the [University of New South Wales Taste of Research program](https://www.engineering.unsw.edu.au/taste-of-research-program).
# Prerequisite
- Conda (either Miniconda or Anaconda) (Tested version: 4.8.3)

# Installation
Run the following code in the terminal:
```bash
git clone https://github.com/dixon777/variableNeutralLineManipulator.git vnlm_math_model
cd vnlm_math_model
conda env create -f conda_env.yml
conda activate vnlm_math_model
```

# Run the program
1. To compute the results evaluated from math model, run the following:
    ```bash
    python test_math_model.py
    ```
2. To validate the results from rigid body simulation with Adams View, 
    1. Turn on the Adams View command server listening (Tutorial is available in Adams View Help tab accessible via the question mark button at the top menu bar of the main interface)
    2. Run the following:
    ```bash
    python test_simulation.py
    ```
    (Change the port number if necessary)
