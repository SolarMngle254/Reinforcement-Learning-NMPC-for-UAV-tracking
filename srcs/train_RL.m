% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty 
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu) 
% Date modified: 30/08/2026

clear; close all; clc;

env = DroneRLEnv();

obsInfo = getObservationInfo(env);

actInfo = getActionInfo(env);

% Deeper network architecture (128-128-64) to process lookahead features

actorNetwork = [

    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'state')

    fullyConnectedLayer(128, 'Name', 'fc1')

    reluLayer('Name', 'relu1')

    fullyConnectedLayer(128, 'Name', 'fc2')

    reluLayer('Name', 'relu2')

    fullyConnectedLayer(64, 'Name', 'fc3')

    reluLayer('Name', 'relu3')

    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'action')

    tanhLayer('Name', 'tanh')

];

actor = rlContinuousDeterministicActor(actorNetwork, obsInfo, actInfo);

statePath = [

    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'state')

    fullyConnectedLayer(128, 'Name', 'fc1_state')

];

actionPath = [

    featureInputLayer(actInfo.Dimension(1), 'Normalization', 'none', 'Name', 'action')

    fullyConnectedLayer(128, 'Name', 'fc1_action')

];

commonPath = [

    additionLayer(2, 'Name', 'add')

    reluLayer('Name', 'relu1')

    fullyConnectedLayer(128, 'Name', 'fc2')

    reluLayer('Name', 'relu2')

    fullyConnectedLayer(64, 'Name', 'fc3')

    reluLayer('Name', 'relu3')

    fullyConnectedLayer(1, 'Name', 'QValue')

];

criticNetwork = layerGraph(statePath);

criticNetwork = addLayers(criticNetwork, actionPath);

criticNetwork = addLayers(criticNetwork, commonPath);

criticNetwork = connectLayers(criticNetwork, 'fc1_state', 'add/in1');

criticNetwork = connectLayers(criticNetwork, 'fc1_action', 'add/in2');

critic1 = rlQValueFunction(criticNetwork, obsInfo, actInfo, 'ObservationInputNames', 'state', 'ActionInputNames', 'action');

critic2 = rlQValueFunction(criticNetwork, obsInfo, actInfo, 'ObservationInputNames', 'state', 'ActionInputNames', 'action');

% Configure the agent with an appropriate exploration noise level
% for the normalized action space [-1, 1].
%
% Use 'StandardDeviation', 0.22 instead of 'Variance', 0.05.

agentOpts = rlTD3AgentOptions(...

    'SampleTime', env.Ts, ...

    'TargetSmoothFactor', 5e-3, ...

    'ExperienceBufferLength', 1e5, ...

    'MiniBatchSize', 256, ...

    'ExplorationModel', rl.option.GaussianActionNoise('StandardDeviation', 0.22));

agent = rlTD3Agent(actor, [critic1, critic2], agentOpts);

% Train for the full 1000 episodes without using a stopping criterion

trainOpts = rlTrainingOptions(...

    'MaxEpisodes', 1000, ...

    'MaxStepsPerEpisode', env.MaxSteps, ...

    'ScoreAveragingWindowLength', 50, ...

    'Verbose', false, ...

    'Plots', 'training-progress');

trainingStats = train(agent, env, trainOpts);

save('RL_NMPC_Policy.mat', 'agent');
