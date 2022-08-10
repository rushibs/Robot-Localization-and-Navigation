req_id = 67;
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
        res(1,1) = p_1x; 
        res(2,1) = p_1y;
        res(3,1) = p_2x;
        res(4,1) = p_2y;
        res(5,1) = p_3x; 
        res(6,1) = p_3y;
        res(7,1) = p_4x;
        res(8,1) = p_4y;
        res