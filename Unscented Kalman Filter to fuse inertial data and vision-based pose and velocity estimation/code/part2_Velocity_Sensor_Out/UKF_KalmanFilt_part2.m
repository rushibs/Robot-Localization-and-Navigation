clear; % Clear variables
clc;
addpath('../data')
datasetNum = 1; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.01*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0;
vel = proj2Data.linearVel;
angVel2 = proj2Data.angVel;

%% Unscented Kalman Filter

for i = 1:length(sampledTime)

     angVel = sampledData(i).omg;
     acc = sampledData(i).acc;
     dt = sampledTime(i)-prevTime;
     [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
     z_t = [transpose(vel(i,:));transpose(angVel2(i,:))];
     [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
     prevTime = sampledTime(i);
     savedStates(:,i) = uCurr;
     uPrev = uCurr;
     covarPrev = covar_curr;
   
end

plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);