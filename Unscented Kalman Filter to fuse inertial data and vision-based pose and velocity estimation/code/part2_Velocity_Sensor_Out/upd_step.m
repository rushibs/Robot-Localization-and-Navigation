function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state

%% Variables

   alpha = 0.001;
   k = 1;
   n_prime = 15;
   lambda_prime = alpha^2 * (n_prime+k) - n_prime;

%% Frame Transformation
    R_cb=[0.707 -0.707 0; -0.707 -0.707 0; 0 0 -1];
    R_bc=transpose(R_cb);
    trans_bc=[-0.04; 0;-0.03];
    % trans_bc_skew = [0 -trans_bc(3) trans_bc(2); trans_bc(3) 0 -trans_bc(1); -trans_bc(2) trans_bc(1) 0]; 
    T_bc=[R_bc trans_bc; 0 0 0 1];
    T_cb=inv(T_bc);
    trans_cb = T_cb(1:3,4);
    t_cb_skew = [0 -trans_cb(3) trans_cb(2) ; trans_cb(3) 0 -trans_cb(1) ; -trans_cb(2) trans_cb(1) 0];


%% Calculation for Sigma points

     X_0 = uEst;
     covar_aug = chol(covarEst,"lower");
     X_1 = [];
     X_2 = [];
     for i = 1 : n_prime
         x_1 = uEst + sqrt(n_prime+lambda_prime)*(covar_aug(:,i));
         x_2 = uEst - sqrt(n_prime+lambda_prime)*(covar_aug(:,i));
         X_1 = [X_1 x_1];
         X_2 = [X_2 x_2];
     end

     X_sigma=[X_0 X_1 X_2];

%% Propogating Sigma points throught the non-linear function

     Z_ti=[];
     R_t = normrnd(0,(0.0001),[3,1]);
     for k = 1:(2*n_prime+1)
         rpy = transpose(uEst(4:6));
         R_bw = eul2rotm(rpy);
         R_wb = inv(R_bw);
         Z_ti_temp = [R_bc*R_wb*[X_sigma(7:9,k)]-R_bc*t_cb_skew*R_cb*(z_t(4:6,1))+R_t];
         Z_ti = [Z_ti Z_ti_temp];
     end

%%  Computing the weights 

    Wu_0 = lambda_prime/(lambda_prime+n_prime);
    Wu_i = 1/(2*(lambda_prime+n_prime));
    Wc_0 = lambda_prime/((lambda_prime+n_prime))+(1-alpha^2+2);
    Wc_i = 1/(2*(lambda_prime+n_prime));


    %% Computing the predicted mean 
        Z_ut=[];
        for m=1:(2*n_prime+1)
            if m==1
                Z_ut=Wu_0*Z_ti(:,m);
            else 
                Z_ut=Z_ut+(Wu_i*Z_ti(:,m));
            end
        end


    %%  Computing the predicted cross-covariance C_t and predicted covariance of the measurement S_t
        C_t=[];
        S_t = [];
        v_t = eye(3)*0.4;
        for n=1:(2*n_prime+1)
            if n==1
                C_t=Wc_0*(X_sigma(:,n)-(uEst))*((Z_ti(:,n))-Z_ut)';
                S_t = Wc_0*((Z_ti(:,n))-Z_ut)*((Z_ti(:,n))-Z_ut)';
            else
                C_t=C_t+Wc_i*(X_sigma(:,n)-(uEst))*((Z_ti(:,n))-Z_ut)';
                S_t = S_t+Wc_i*((Z_ti(:,n))-Z_ut)*((Z_ti(:,n))-Z_ut)';
            end
        end
        S_t = S_t + v_t;

 
 %% Computing the Kalman Gain and the filtered state mean and covariance, conditional to the measurement
    K_t = C_t * inv(S_t);
    uCurr = uEst + K_t * (z_t(1:3)-Z_ut);
    covar_curr = covarEst - K_t*S_t*(K_t');

end


    


