%% 里程计简单仿真
function odometry_simulate()
    % 最新的里程计位姿
    odomPose = [0, 0, 0]'; 
    % 里程计轨迹
    odom2d = [
        0, odomPose';
    ];
    

    rw = 0.08;  % 车轮半径
    d = 0.2;    % 车轮安装轴距
    vL = 0.3;  % 左轮线速度
    vR = 0.25;  % 右轮线速度
    v = (vR + vL) / 2;      % 旋转中心线速度
    w = (vR - vL) / 2 * d;  % 旋转中心角速度
    dt = 1; % 时间差

    for i = 1:100
        dtheta = w * dt;
        dx = v * dt;
        incPose = [
            dx * cos(dtheta); 
            dx * sin(dtheta); 
            dtheta
        ];

        theta = odomPose(3);
        R = [
            cos(theta), -sin(theta), 0;
            sin(theta), cos(theta), 0;
            0, 0, 1;
        ];
        odomPose =  R * incPose + odomPose;
        odom2d = [odom2d; 1, odomPose';];
    end

    visu.Begin("");
    visu.DrawPose2d(odom2d, 'k');
    visu.End();
end