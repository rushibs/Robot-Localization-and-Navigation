function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t is the measurement
%covarEst and uEst are the predicted covariance and mean respectively
%uCurr and covar_curr are the updated mean and covariance respectively

C_t = zeros(9,15);
C_t(7:9, 7:9) = eye(3); 

R = eye(9)*10e-5;

K_t = covarEst*C_t' / (C_t*covarEst*C_t'+ R);

uCurr = uEst + K_t*(z_t-(C_t*uEst)) ;
covar_curr = covarEst - K_t*C_t*covarEst;


end