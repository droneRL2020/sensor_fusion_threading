clear; % Clear variables
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
%% How do you want to do this version
Z = sampledVicon(1:6,:);
% Set initial condition
uPrev = vertcat(sampledVicon(1:6,1),zeros(9,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time5 step in real time
for i = 1:length(sampledTime)
    i
    angVel = sampledData(i).omg;
    acc    = sampledData(i).acc;
    if i == 1
        dt = sampledTime(1) - 0;
    else
        dt = sampledTime(i) - sampledTime(i-1);
    end
    z_t = Z(:,i);
    [covarEst,uEst] = prediction(uPrev,covarPrev,angVel,acc,dt);
    [uCurr,covar_curr] = update(z_t,covarEst,uEst);
    savedStates(:,i) = uCurr;
    uPrev = uCurr;
    covarPrev = covar_curr;
end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);