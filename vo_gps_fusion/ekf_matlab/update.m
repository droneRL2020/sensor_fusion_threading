function [uCurr,covar_curr] = update(z_t,covarEst,uEst)
    %z_t - is the sensor data at the time step
    %% Update using position and orientation from vicon
    C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;];

    %% Update using linear velocity from vicon
    %C = [0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
    %     0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;
    %     0 0 0 0 0 0 0 0 1 0 0 0 0 0 0];
    R = (1e-3) * eye(6);
    
    
    K = covarEst*C.'*(inv(C*covarEst*C.' + R));
    
    uCurr = uEst + K*(z_t - C*uEst);
    covar_curr = covarEst - (K*C*covarEst);
    
    
end

