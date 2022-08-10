function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
    res = zeros(8, length(id));  %each column of res is [p1_x; p1_y; p2_x; p2_y; p3_x; p3_Y; p4_x; p4_y]
    
    for i= 1:length(id)

        req_id = id(1,i);
        col = fix(req_id/12) + 1;    %column number of the required id
        p_4x = 0.152*2*mod(req_id,12);
        p_3x = p_4x;
        p_2x = p_3x + 0.152;
        p_1x = p_2x;
        
        if (col<4)
            p_4y = (2*col - 2)*0.152;
            p_3y = p_4y + 0.152;
            p_2y = p_3y;
            p_1y = p_4y;
        elseif (col>3 && col<7)
            p_4y = (2*col - 2)*0.152 + 0.026;
            p_3y = p_4y + 0.152;
            p_2y = p_3y;
            p_1y = p_4y;
        else
            p_4y = (2*col - 2)*0.152 + 0.052;
            p_3y = p_4y + 0.152;
            p_2y = p_3y;
            p_1y = p_4y;
        end
        res(1,i) = p_1x; 
        res(2,i) = p_1y;
        res(3,i) = p_2x;
        res(4,i) = p_2y;
        res(5,i) = p_3x; 
        res(6,i) = p_3y;
        res(7,i) = p_4x;
        res(8,i) = p_4y;
         
        
    end