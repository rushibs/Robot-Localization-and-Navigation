clear; % Clear variables
clc;
addpath('../data')
datasetNum = 1; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime,proj2Data] = init(datasetNum);

Z = sampledVicon(1:6,:);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.1*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0; %last time step in real time
pos = proj2Data.position;
pose = proj2Data.angle;
for i = 1:length(sampledTime)

%% Fill in the FOR LOOP

     angVel=sampledData(i).omg;
     acc=sampledData(i).acc;
     dt=sampledTime(i)-prevTime;
     [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
     z_t=[transpose(pos(i,:));transpose(pose(i,:))];
     [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
     prevTime=sampledTime(i);
     savedStates(:,i)=uCurr;
     uPrev=uCurr;
     covarPrev=covar_curr;

end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);