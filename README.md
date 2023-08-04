# RBF_Load
This Grasshopper component provides a method for obtaining funicular surfaces that pass through desired points through load control. The method involves generating smooth load distributions via RBF interpolation of nodal loads, which are repeatedly adjusted until the equilibrium shape is close enough to the target points.

![RBF_Load_component_アートボード 1](https://github.com/kazukihayashi/RBF_Load/assets/25089369/555df582-4fa2-4810-8516-65690ab519cf)
<img width="952" alt="ex2_init_shape" src="https://github.com/kazukihayashi/RBF_Load/assets/25089369/b0cd27a0-526b-4963-8a12-c7c3920574df">
<img width="952" alt="ex2_opt_shape" src="https://github.com/kazukihayashi/RBF_Load/assets/25089369/138e9f08-9e01-42f6-b7d2-296b4ed6e2c8">
<img width="952" alt="ex2_load" src="https://github.com/kazukihayashi/RBF_Load/assets/25089369/aa12c98f-08be-420f-b717-7babd7c7fd1c">


## Folder/File description

### RBF_Load(warm)
The source codes (written in C#) of the main contribution. "warm" refers to a warm start, which is a technique where an algorithm is initialized with a solution obtained from a previous run.
The compiled gh file is available at food4Rhino: https://www.food4rhino.com/en/app/rbfload

### RBF_Load(cold)
The source codes (written in C#)  for comparing the computational efficiency between warm and cold starts. The cold start uses the initial shape as an initial solution without prior information about the solutions obtained from previous runs.
Therefore, RBF_Load(cold) is much slower than RBF_Load(warm).

### RBF_Load(alpha)
The source codes (written in C#)  of a component allowing a fixed-value input for stepsize $\alpha$. RBF_Load(warm) automatically adjusts the value of $\alpha$ using a coarse line search. This project is only for verifying the performance of the coarse line search.

### ex1.gh
A simple 20x10-grid model. This includes structural analysis using Karamba 3D, which requires another license to use.
Please see the following website for more details about Karamba 3D: https://karamba3d.com/

### ex2.gh
A complex model (the shape is shown in this README.md file above). This does not include Karambra 3D implementation.
