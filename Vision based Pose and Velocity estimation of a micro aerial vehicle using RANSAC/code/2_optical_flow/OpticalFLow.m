%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

ransac = 0;   %%%% Flag for Ransac


[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
K = [311.0520        0        201.8724;
            0         311.3885    113.6210;
            0            0           1    ];

t = zeros(length(sampledData),1);
for n = 1:length(sampledData)
    t(n) = sampledData(n).t;
end
t = sgolayfilt(t,1,101);
T_bc = vertcat(horzcat((rotz(-45)*rotx(180)),[0.04; 0; -0.03]),[0, 0, 0, 1]);
R_bc = T_bc(1:3,1:3);
t_bc = T_bc(1:3,4);
for n = 2:length(sampledData)
    %% Initalize Loop load images
    prev_img = sampledData(n-1).img;
    cur_img = sampledData(n).img;
    d_t = t(n) - t(n-1);
    %% Detect good pts
    good_pts = detectMinEigenFeatures(prev_img).selectStrongest(100).Location;
    
    %% Initalize the tracker to the last frame.
    pt_track = vision.PointTracker('MaxBidirectionalError',1);
    
    %% Find the location of the next pts;
    initialize(pt_track,good_pts,prev_img);
    [pts,validity] = pt_track(cur_img);

    %% Calculate velocity
    % Use a for loop
       
    %% Calculate Z
    [position, orientation, R_c2w] = estimatePose(sampledData, n);
    
    R_wb = eul2rotm(orientation);
    T_wb = [R_wb position; 0 0 0 1];
    T_bw = inv(T_wb);
    T_cw = inv(T_bc)*T_bw;
    T_wc = inv(T_cw);
    R_wc = T_wc(1:3,1:3);
    good_normal = [];
    pts_normal = [];
    for m = 1:length(pts)
       %temp_est = position(3)/(dot([pts(m,1);pts(m,2);1],-1*R_wc(3,:)));
       %est_z = [est_z;temp_est];
       temp_pts_normal = [pts(m,1); pts(m,2); 1];
       temp_good_normal = [good_pts(m,1); good_pts(m,2); 1];
       temp_pts_normal = inv(K)*temp_pts_normal;
       temp_good_normal = inv(K)*temp_good_normal;
       pts_normal = [pts_normal; transpose(temp_pts_normal)];
       good_normal = [good_normal; transpose(temp_good_normal)];
    end
    vel = [];
    for i = 1:length(good_pts)
    
    temp = [(pts_normal(i,1)-good_normal(i,1))/d_t; (pts_normal(i,2)-good_normal(i,2))/d_t];
     
    vel = [vel;temp];
    end
    
    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    if ransac == 0
        Ap = [];
        Bp = [];
        P_dot = [];
        for l = 1:length(pts)
         x = pts_normal(l,1);   
         y = pts_normal(l,2);
         Z = position(3)/(dot([x;y;1],-1*R_wc(:,3)));
         temp_Ap = [-1/Z 0 x/Z; 0 -1/Z y/Z];
         temp_Bp = [x*y -(1+x^2) y; 1+y^2 -x*y -x];
         Ap = [Ap; temp_Ap];
         Bp = [Bp; temp_Bp];
       
         
        end
        H = [Ap, Bp];
        temp_vel = pinv(H)*vel;
        Tbc_skew = [   0   -t_bc(3,1)  t_bc(2,1) ;
                    t_bc(3,1)     0  -t_bc(1,1) ;
                   -t_bc(2,1)  t_bc(1,1)     0 ];
        Transformation = [R_wb, zeros(3); zeros(3) R_wb]*[R_bc -R_bc*Tbc_skew; zeros(3) R_bc];
        Vel = Transformation*temp_vel;
    else
      for l = 1:length(pts) 
       x = pts_normal(l,1);   
       y = pts_normal(l,2);
       Z = position(3)/(dot([x;y;1],-1*R_wc(:,3)));
        
      end
       e = 0.8;
       temp_vel = velocityRANSAC(vel,pts_normal,Z,R_c2w,e); 
       Tbc_skew = [   0   -t_bc(3,1)  t_bc(2,1) ;
                    t_bc(3,1)     0  -t_bc(1,1) ;
                   -t_bc(2,1)  t_bc(1,1)     0 ];
        Transformation = [R_wb, zeros(3); zeros(3) R_wb]*[R_bc -R_bc*Tbc_skew; zeros(3) R_bc];
        Vel = Transformation*temp_vel;
    end
           
    %% Thereshold outputs into a range.
    % Not necessary
   
       
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 
    %estimatedV(:,n) = Vel;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
   
    estimatedV(:,n) = Vel;
    % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 47);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 13);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 13);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 13);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 13);

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
