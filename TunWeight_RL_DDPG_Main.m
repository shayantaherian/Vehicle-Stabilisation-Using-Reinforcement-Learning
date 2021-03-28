% Main loop
Vehicle_parameters;
% Physical limitations

useGPU = true;
% useParallel = true;

Ts = 0.1;
Tf = 15;
observationInfo = rlNumericSpec([11 1]);
observationInfo.Name = 'observations';
% action Info
actionInfo = rlNumericSpec([4 1],'LowerLimit',[40;40;40;40],'UpperLimit', [1e3;1e3;1e3;1e3]);
actionInfo.Name = 'Wdf1;wdf2;wdf3;wdf4';
% define environment
mdl = 'RLTunningTV';
open_system(mdl)
agentblk = [mdl '/RL Agent'];
env = rlSimulinkEnv(mdl,agentblk,observationInfo,actionInfo);
env.ResetFcn = @(in)localResetFcn(in);
rng(0)
%% CREATE NEURAL NETWORKS
TunWeight_RL_DDPGNetworks;

%% CREATE AND TRAIN AGENT
TunWeight_RL_DDPGOptions;
% Train the agent
agent = rlDDPGAgent(actor,critic,agentOptions);
trainingStats = train(agent,env,trainingOptions);
% Inference
% simOptions = rlSimulationOptions('MaxSteps',maxsteps);
% experience = sim(env,agent,simOptions);

% Save Agent
reset(agent); % Clears the experience buffer
curDir = pwd;
saveDir = 'savedAgents';
cd(saveDir)
save(['trainedAgent_Torque_vectoring_' datestr(now,'mm_DD_YYYY_HHMM')],'agent','trainingStats');
cd(curDir)
