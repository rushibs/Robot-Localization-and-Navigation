function [position, orientation] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX
    K = [311.0520        0        201.8724;
            0         311.3885    113.6210;
            0            0           1    ];

    res = getCorner(data(t).id);
    req_id = data(t).id;
    A = [];
    for i = 1:length(req_id)
        p1_xi = data(t).p1(1,i);
        p1_yi = data(t).p1(2,i);
        p2_xi = data(t).p2(1,i);
        p2_yi = data(t).p2(2,i);
        p3_xi = data(t).p3(1,i);
        p3_yi = data(t).p3(2,i);
        p4_xi = data(t).p4(1,i);
        p4_yi = data(t).p4(2,i);
        p1_x = res(1,i);
        p1_y = res(2,i);
        p2_x = res(3,i);
        p2_y = res(4,i);
        p3_x = res(5,i);
        p3_y = res(6,i);
        p4_x = res(7,i);
        p4_y = res(8,i);
        
        temp1 = [p1_x    p1_y   1      0     0     0    -p1_xi*p1_x    -p1_xi*p1_y -p1_xi;
             0         0     0   p1_x   p1_y   1    -p1_yi*p1_x    -p1_yi*p1_y -p1_yi;
             p2_x    p2_y   1      0     0     0    -p2_xi*p2_x    -p2_xi*p2_y -p2_xi;
             0         0     0   p2_x   p2_y   1    -p2_yi*p2_x    -p2_yi*p2_y -p2_yi;
             p3_x    p3_y   1      0     0     0    -p3_xi*p3_x    -p3_xi*p3_y -p3_xi;
             0         0     0   p3_x   p3_y   1    -p3_yi*p3_x    -p3_yi*p3_y -p3_yi;
             p4_x    p4_y   1      0     0     0    -p4_xi*p4_x    -p4_xi*p4_y -p4_xi;
             0         0     0   p4_x   p4_y   1    -p4_yi*p4_x    -p4_yi*p4_y -p4_yi;];
    A = [A; temp1];    %%%%%%%%%%%%%%%
    end
        [~,~,V] = svd(A);
        h = [V(1,9) V(2,9) V(3,9);
             V(4,9) V(5,9) V(6,9);
             V(7,9) V(8,9) V(9,9)];
       
        h = h/V(9,9);
        temp = inv(K)*h;
        R1 = [temp(1,1); temp(2,1); temp(3,1)];
        R2 = [temp(1,2); temp(2,2); temp(3,2)];
        T =  [temp(1,3); temp(2,3); temp(3,3)];
        
        R1_skew = [   0   -R1(3,1)  R1(2,1) ;
                    R1(3,1)     0  -R1(1,1) ;
                   -R1(2,1)  R1(1,1)     0 ];
        R1xR2 = R1_skew*R2;
        
        temp2 = [R1 R2 R1xR2];
        [U1,~,V1] = svd(temp2);
        V1 = transpose(V1);
        R = U1*[1 0 0; 0 1 0; 0 0 det(U1*V1)]*(V1);
        den = sqrt(R1(1,1)^2 + R1(2,1)^2 +R1(3,1)^2);     
        T = T/den;
        
        H_cw = [R T; 0 0 0 1]; %world to camera
        r = rotz(-45)*rotx(180);  %body to camera
        trans = [0.04*cos(pi/4); -0.04*cos(pi/4); -0.03];     %body to camera
        H_bc = [r trans; 0 0 0 1];            %body to camera
        H_wb = inv(H_cw)*inv(H_bc);           %world to body
        position = H_wb(1:3,4);
        orientation = rotm2eul(H_wb(1:3,1:3), 'ZYX');     
 
end