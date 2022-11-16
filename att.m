function tau=att(q_start,q_end,myrobot)
    scale = 1;
    % Get the intermediate H matrices 
    % starting points
    H_10_s = myrobot.A(1:1, q_start);
    H_20_s = myrobot.A(1:2, q_start);
    H_30_s = myrobot.A(1:3, q_start);
    H_40_s = myrobot.A(1:4, q_start);
    H_50_s = myrobot.A(1:5, q_start);
    H_60_s = myrobot.A(1:6, q_start);
    % end points
    H_10_e = myrobot.A(1:1, q_end);
    H_20_e = myrobot.A(1:2, q_end);
    H_30_e = myrobot.A(1:3, q_end);
    H_40_e = myrobot.A(1:4, q_end);
    H_50_e = myrobot.A(1:5, q_end);
    H_60_e = myrobot.A(1:6, q_end);
    
    tau = H_10;
    
end