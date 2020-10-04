function [covarEst,uEst] = prediction(uPrev,covarPrev,angVel,acc,dt)
    %uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time
    syms x y z q_x q_y q_z v_x v_y v_z bg_x bg_y bg_z ba_x ba_y ba_z 
    syms w_x w_y w_z a_x a_y a_z ng_x ng_y ng_z na_x na_y na_z nbg_x nbg_y nbg_z nba_x nba_y nba_z
    x_vector = [x;y;z;
     q_x;q_y;q_z;
     v_x;v_y;v_z;
     bg_x;bg_y;bg_z;   %?? gyro bias
     ba_x;ba_y;ba_z];  %?? accel bias
    
    noise = [ng_x;ng_y;ng_z;   % ?? sensor error
             na_x;na_y;na_z;
             nba_x;nba_y;nba_z;
             nbg_x;nbg_y;nbg_z];
      
    %angular velocity vector
    G_mat = [cos(q_y) 0 -(cos(q_x)*sin(q_y)); %G matrix 
        0 1 sin(q_x) ; 
        sin(q_y) 0 (cos(q_x)*cos(q_y))];
    omega = inv(G_mat) * [w_x - bg_x - ng_x;
                      w_y - bg_y - ng_y;
                      w_z - bg_z - ng_z];
    omg_x = omega(1);
    omg_y = omega(2);
    omg_z = omega(3);
    
    %linear acceleration vector
    comp_1 = (cos(q_z)*cos(q_y)) - (sin(q_x)*sin(q_z)*sin(q_y));
    comp_2 = -(cos(q_x)*cos(q_z));
    comp_3 = (cos(q_z)*sin(q_y)) + (cos(q_y)*sin(q_x)*sin(q_z));
    comp_4 = (cos(q_y)*sin(q_z)) + (cos(q_z)*sin(q_x)*sin(q_y));
    comp_5 = cos(q_x)*cos(q_z);
    comp_6 = (sin(q_z)*sin(q_y)) - (cos(q_z)*cos(q_y)*sin(q_x));
    comp_7 = -(cos(q_x)*sin(q_y));
    comp_8 = sin(q_x);
    comp_9 = cos(q_x)*cos(q_y);
    %Rotation matrix (Z-X-Y)
    R_mat = [comp_1 comp_2 comp_3 ; comp_4 comp_5 comp_6 ; comp_7 comp_8 comp_9];
    accMatrix = [ 0 ; 0 ; -9.81] + R_mat*[a_x - ba_x - na_x;
                                a_y - ba_y - na_y;
                                a_z - ba_z - na_z;];
    acc_x = accMatrix(1);
    acc_y = accMatrix(2);
    acc_z = accMatrix(3);
    
    x_dot=[v_x;v_y;v_z;
       omg_x;omg_y;omg_z;  % angular velocity w.r.t world
       acc_x;acc_y;acc_z;  % linear acceleration w.r.t world
       nbg_x;nbg_y;nbg_z;
       nba_x;nba_y;nba_z];  
             
    
    A = jacobian(x_dot, x_vector);
    U = jacobian(x_dot, noise);
    
    all = [x;y;z;q_x;q_y;q_z;v_x;v_y;v_z;bg_x;bg_y;bg_z;ba_x;ba_y;ba_z;
        w_x;w_y;w_z;a_x;a_y;a_z;ng_x;ng_y;ng_z;na_x;na_y;na_z;nbg_x;nbg_y;nbg_z;nba_x;nba_y;nba_z];

    x_dot_cal = double(subs(x_dot, all, [uPrev;angVel;acc;zeros(12,1)]));    
    A_cal = double(subs(A, all, [uPrev;angVel;acc;zeros(12,1)]));
    U_cal = double(subs(U, all, [uPrev;angVel;acc;zeros(12,1)]));
    
    F = eye(15) + dt*A_cal;  % A_cal 은 자코비안이니 속도군 state transition 에 B가 들어가나?
    Q = (1e-3)*eye(12);
    %Q(7,7) = (1e-6);
    
    uEst = uPrev + dt*x_dot_cal;
    covarEst = F*covarPrev*F.' + U_cal*dt*Q*U_cal.';
end

