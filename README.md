![MATLAB](https://img.shields.io/badge/MATLAB-R2018b-blue.svg)
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

# Non-Prehensile Manipulation Actions and Visual 6D Pose Estimation for Fruit Grasping Based on Tactile Sensing
This workspace extends the work of [Hogan et al.]() on the design of a Linear Model Predictive Control to perform _pushing_ manouvre under the quasi-static assumption in the Matlab environment through the [Gurobi optimizer](https://www.gurobi.com/).

## Content
- [Requirements](#requirements)
- [Usage](#usage)

## Requirements 
- The code has been tested on **Matlab2018b** (later Matlab versions should be compatible).  

- A **[Gurobi license](https://www.gurobi.com/lp/all/licensing/?utm_source=google&utm_medium=cpc&utm_campaign=2024+na+googleads+request+an+evaluation+license&utm_content=sitelink&campaignid=2027425882&adgroupid=165853131800&creative=702008510760&keyword=gurobi&matchtype=e&_bn=g&gad_source=1&gclid=CjwKCAjw68K4BhAuEiwAylp3koorjOq2qhMndKnlt_b7mGOAtaExZhydcgB1ZZs6827DQq9xLDWXihoCc34QAvD_BwE)** is required to solve the optimization problem set up with the MPC.

- The project uses several external libraries, which are included as submodules (see software/externals, or git submodules).


## Usage
Clone the repository and initialize submodules:  
```
git clone https://github.com/sara9915/uclv_fompush.git
cd uclv_fompush 
git submodule update --init --recursive
```

Just set parameters of your slider in **`Simulation/ClosedLoopPushing/@PusherSlider/PusherSlider.m`** and set up the reference trajectory in **`Simulation/ClosedLoopPushing/@PusherSlider/errorVector.m`** 



## How to cite us
If you use this code in your project, please, cite us as follows:
```
@Article{robotics12040092,
AUTHOR = {Costanzo, Marco and De Simone, Marco and Federico, Sara and Natale, Ciro},
TITLE = {Non-Prehensile Manipulation Actions and Visual 6D Pose Estimation for Fruit Grasping Based on Tactile Sensing},
JOURNAL = {Robotics},
VOLUME = {12},
YEAR = {2023},
NUMBER = {4},
ARTICLE-NUMBER = {92},
URL = {https://www.mdpi.com/2218-6581/12/4/92},
ISSN = {2218-6581},
DOI = {10.3390/robotics12040092}
}
```
