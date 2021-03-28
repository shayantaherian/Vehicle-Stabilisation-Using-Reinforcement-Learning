
% agentOptions = rlDDPGAgentOptions(...
%     'SampleTime',Ts,...
%     'TargetSmoothFactor',1e-3,...
%     'ExperienceBufferLength',1e6 ,...
%     'DiscountFactor',0.99,...
%     'MiniBatchSize',250,...
%     '');
% agentOptions.NoiseOptions.Variance = 0.1;

agentOptions = rlDDPGAgentOptions;
agentOptions.SampleTime = Ts;
agentOptions.DiscountFactor = 0.99;
agentOptions.MiniBatchSize = 70;
agentOptions.ExperienceBufferLength = 1e6;
agentOptions.TargetSmoothFactor = 1e-3;
agentOptions.NoiseOptions.MeanAttractionConstant = 1;
agentOptions.NoiseOptions.Variance = 30;
agentOptions.NoiseOptions.VarianceDecayRate = 1e-3;

maxepisodes = 50;
maxsteps = floor(Tf/Ts);
trainingOptions = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes,...
    'MaxStepsPerEpisode',maxsteps,...
    'StopOnError',"on",...
    'Verbose',false,...
    'Plots',"training-progress",...
    'StopTrainingCriteria',"AverageReward",...
    'StopTrainingValue',6000,...
    'ScoreAveragingWindowLength',250,...
    'SaveAgentCriteria',"EpisodeReward",...
    'SaveAgentValue',6000);
%      trainingOptions.Parallelization.Mode = 'async'; 
%      trainingOptions.ParallelizationOptions.StepsUntilDataIsSent = 32;
%      trainingOptions.ParallelizationOptions.DataToSendFromWorkers = "experiences";

