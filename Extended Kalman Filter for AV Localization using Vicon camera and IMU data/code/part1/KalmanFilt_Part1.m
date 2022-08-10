clc;clear; % Clear variables
datasetNum = 1; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(1:6,:);%all the measurements that you need for the update
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0; %last time step in real time
%write your code here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime)
    t2 = sampledData(1,i).t;
    if (i == 1)
        t1 = 0;
    end
    dt = t2 - t1;
    angVel = sampledData(1,i).omg;
    acc = sampledData(1,i).acc;
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    t1 = t2;
    z_t = Z(:,i);
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    covarPrev = covar_curr;
    uPrev = uCurr;
    savedStates(:,i) = uCurr;

end
plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);