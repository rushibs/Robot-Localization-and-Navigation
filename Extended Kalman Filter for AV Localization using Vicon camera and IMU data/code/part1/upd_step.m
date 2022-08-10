function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t is the measurement
%covarEst and uEst are the predicted covariance and mean respectively
%uCurr and covar_curr are the updated mean and covariance respectively

I = eye(6);
zero = zeros(6,9);
C_t = [I zero];

R = eye(6)*10e-5;

K_t = covarEst*C_t'* inv(C_t*covarEst*C_t'+R);

uCurr = uEst + K_t*(z_t-(C_t*uEst)) ;
covar_curr = covarEst - K_t*C_t*covarEst;

end