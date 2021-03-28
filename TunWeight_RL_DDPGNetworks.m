
 %% Critic
%load('NewTraied_600'); 
numObs = 11;
numAct = 4;

criticLayerSizes = [100 100]; 

statePath = [
    imageInputLayer([numObs 1 1],'Normalization','none','Name', 'observation')
    fullyConnectedLayer(criticLayerSizes(1), 'Name', 'CriticStateFC1', ... 
            'Weights',criticNetwork.Layers(2,1).Weights, ...
            'Bias',criticNetwork.Layers(2,1).Bias)
    reluLayer('Name','CriticStateRelu1')
    fullyConnectedLayer(criticLayerSizes(2), 'Name', 'CriticStateFC2', ...
            'Weights',criticNetwork.Layers(4,1).Weights, ... 
            'Bias',criticNetwork.Layers(4,1).Bias)
    ];
actionPath = [
    imageInputLayer([numAct 1 1],'Normalization','none', 'Name', 'action')
    fullyConnectedLayer(criticLayerSizes(2), 'Name', 'CriticActionFC1', ...
            'Weights',criticNetwork.Layers(6,1).Weights, ... 
            'Bias',criticNetwork.Layers(6,1).Bias)
    ];
commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(1, 'Name', 'CriticOutput',...
            'Weights',criticNetwork.Layers(9,1).Weights, ...
            'Bias',criticNetwork.Layers(9,1).Bias)
    ];

% Connect the layer graph
criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = addLayers(criticNetwork, commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');

% Create critic representation
criticOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3, ... 
                                        'GradientThreshold',1,'L2RegularizationFactor',2e-4);
if useGPU
   criticOptions.UseDevice = 'gpu'; 
end
critic = rlRepresentation(criticNetwork,criticOptions, ...
                          'Observation',{'observation'},env.getObservationInfo, ...
                          'Action',{'action'},env.getActionInfo);
%  figure
% plot(criticNetwork)

%% ACTOR
% Create the actor network layers
actorLayerSizes = [100 100];
actorNetwork = [
    imageInputLayer([numObs 1 1],'Normalization','none','Name','observation')
    fullyConnectedLayer(actorLayerSizes(1), 'Name', 'ActorFC1', ...
            'Weights',actorNetwork(2,1).Weights, ... 
            'Bias',actorNetwork(2,1).Bias)
    reluLayer('Name', 'ActorRelu1')
    fullyConnectedLayer(actorLayerSizes(2), 'Name', 'ActorFC2', ... 
            'Weights',actorNetwork(4,1).Weights, ... 
            'Bias',actorNetwork(4,1).Bias)
    reluLayer('Name', 'ActorRelu2')
    fullyConnectedLayer(numAct, 'Name', 'ActorFC3', ... 
            'Weights',actorNetwork(6,1).Weights, ... 
            'Bias',actorNetwork(6,1).Bias)                       
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Name','ActorScaling1','Scale',max(actionInfo.UpperLimit))];

% Create actor representation
actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-4, ...
                                       'GradientThreshold',1,'L2RegularizationFactor',1e-5);
if useGPU
   actorOptions.UseDevice = 'gpu'; 
end
actor = rlRepresentation(actorNetwork,actorOptions, ... 
                         'Observation',{'observation'},env.getObservationInfo, ...
                         'Action',{'ActorScaling1'},env.getActionInfo);