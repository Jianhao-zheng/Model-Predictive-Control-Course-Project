## Model-Predictive-Control-Project

> [JIanhao Zheng](https://github.com/Jianhao-zheng), [Shuhan He](https://github.com/TracyLeee), and [Yujie He](https://github.com/hibetterheyj)

Final Course project for [Model predictive control (ME-425)](https://edu.epfl.ch/coursebook/en/model-predictive-control-ME-425?cb_cycle=bama_cyclemaster&cb_section=mt_ro) supervised by [Prof. Colin Jones](https://people.epfl.ch/colin.jones) at EPFL.

### Report

**:star: Final version of the report: [MPC_Report](./MPC_Report.pdf)**

### Dependance

- MPT3 (Multiparametric Toolbox 3.0) and YALMIP (Yet Another LMI Parser)

  https://www.mpt3.org/Main/Installation

- Gurobi optimizer

  https://www.gurobi.com/academia/academic-program-and-licenses/

- CASADI

  https://web.casadi.org/get/

```matlab
% ###################### Initialization ################################

addpath('your_path_to_casadi\casadi-windows-matlabR2016a-v3.5.5');

import casadi.*;

gurobi_setup;

% then run each deliverables one by one

% #################################################################
```

For more info, please refer to [Software_setup_for_exercises.pdf](./Software_setup_for_exercises.pdf)