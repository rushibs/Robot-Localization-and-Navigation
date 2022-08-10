function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 

 %% Variables

    alpha=0.001;
    k=1;
    n=27;
    Q_t=eye(12)*0.218;
    lambda=alpha^2*(n+k)-n;
%%  Augmented mean and Covariance

    u_aug=[uPrev;zeros(12,1)];
    covar_aug=[covarPrev zeros(15,12);zeros(12,15) Q_t];
    covar_aug=chol(covar_aug);
%% Computing the Sigma Points

    X_0 = u_aug;
    uEst=[];
    covarEst=[];
    
    X_1=[];
    X_2=[];

    for i = 1:n
        x_1 = u_aug+sqrt(n+lambda)*(covar_aug(:,i));
        x_2 = u_aug-sqrt(n+lambda)*(covar_aug(:,i));
        X_1 = [X_1 x_1];
        X_2 = [X_2 x_2];
    end
        
    x_sigma = [X_0 X_1 X_2];

%%  Process Model

    func=[];
    for i=1:(2*n+1)
 
    R = [cos(x_sigma(5,i))*cos(x_sigma(6,i)), cos(x_sigma(6,i))*sin(x_sigma(4,i))*sin(x_sigma(5,i)) - cos(x_sigma(4,i))*sin(x_sigma(6,i)), sin(x_sigma(4,i))*sin(x_sigma(6,i)) + cos(x_sigma(4,i))*cos(x_sigma(6,i))*sin(x_sigma(5,i));
        cos(x_sigma(5,i))*sin(x_sigma(6,i)), cos(x_sigma(4,i))*cos(x_sigma(6,i)) + sin(x_sigma(4,i))*sin(x_sigma(5,i))*sin(x_sigma(6,i)), cos(x_sigma(4,i))*sin(x_sigma(5,i))*sin(x_sigma(6,i)) - cos(x_sigma(6,i))*sin(x_sigma(4,i));
        -sin(x_sigma(5,i)), cos(x_sigma(5,i))*sin(x_sigma(4,i)), cos(x_sigma(4,i))*cos(x_sigma(5,i))];
    
    
    G = pinv([cos(x_sigma(5,i))*cos(x_sigma(6,i)), -sin(x_sigma(6,i)), 0;
             cos(x_sigma(5,i))*sin(x_sigma(6,i)), cos(x_sigma(6,i)), 0;
                -sin(x_sigma(5,i)), 0, 1])*R;

    g=[0;0;-9.81];
         
    temp=[x_sigma(7,i);x_sigma(8,i);x_sigma(9,i); G*(angVel-x_sigma(10:12,i)-x_sigma(16:18,i)); 
          g+R*(acc-x_sigma(13:15,i)-x_sigma(19:21,i)); x_sigma(22:24,i); x_sigma(25:27,i)];

    func(1:15,i)=x_sigma(1:15,i)+dt*temp;
    

    end
%%  Computing the weights

    Wu_0 = lambda/(lambda+n);
    Wu_i = 1/(2*(lambda+n));
    Wc_0 = lambda/((lambda+n)+(1-alpha^2+2));
    Wc_i = 1/(2*(lambda+n));

%% Computing Predicted Mean 'uEst' and Predicted Covariance 'covarEst'
    for t=1:(2*n+1)
        if t == 1
            uEst = Wu_0*func(:,1);

        else 
            uEst = uEst + (Wu_i*func(:,t));
        end
    end
    for m = 1:(2*n+1)
        if m == 1
            covarEst = Wc_0*(func(:,1)-uEst)*(func(:,1)-uEst)';
        else
            covarEst=covarEst+(Wc_i*(func(:,m)-uEst)*(func(:,m)-uEst)');
        end
    end 

end


