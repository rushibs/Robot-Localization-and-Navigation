function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
   
     I = eye(6);
     zero = zeros(6,9);
     C_t = [I zero];
     
     R = eye(6)*0.00001;
   
     K_t = covarEst*(C_t') * pinv(C_t*covarEst*C_t'+ R);
     
     uCurr = uEst + K_t*(z_t-C_t*uEst) ;
     covar_curr = covarEst - K_t*C_t*covarEst;
     K_t = covarEst*(C_t')*pinv((C_t*covarEst*C_t' + R));
     uCurr=uEst+K_t*(z_t - C_t*uEst);
     covar_curr =covarEst - K_t*C_t*covarEst;
 end



