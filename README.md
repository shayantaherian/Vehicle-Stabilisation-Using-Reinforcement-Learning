# Vehicle-Stabilisation-Using-Reinforcement-Learning
This repository contains the public release of the MATLAB implementation of the reinforcement learning for vehicle stabilisation. The corresponding paper can be found in [Self-adaptive Torque Vectoring Controller Using Reinforcement Learning](https://arxiv.org/pdf/2103.14892.pdf)

## Installation
Clone the repository `
git clone https://github.com/shayantaherian/Vehicle-Stabilisation-Using-Reinforcement-Learning/.git
`

Install ` MATLAB 2019 or later release`

Install Add-On `Reinforcement Learning Toolbox` and `Deep Learning Toolbox` 

Open `RLTunningTV.slx` Simulink file 

## Training
To train the algorithm, run the `train(agent,env,trainingOptions)` m-file

## Testing
To test the trained agent, comment the `Train the agent` section and uncomment `Inference` section
