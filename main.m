function main()
    truth2d = common.LoadCSV("dataset/truth2d.csv");
    odom2d = common.LoadCSV("dataset/odom2d.csv");
    
    correctMatrix = calibration(truth2d(1:10000, :), odom2d(1:10000, :));
    disp(correctMatrix);
    correctedOdom = Correct(odom2d, correctMatrix);
    
    visu.Begin("");
    visu.DrawPose2d(truth2d, 'r');
    visu.DrawPose2d(odom2d, 'g');
    visu.DrawPose2d(correctedOdom, 'k');
    visu.End();
end

%% 使用标定矩阵修正里程计
function correctedPath = Correct(odom2d, correctMatrix)
    % 计算每次的里程计递增的距离和角度，左乘标定的变换矩阵后累加得到新的轨迹
    lastPose = odom2d(1, 2:end)';
    correctedPose = lastPose;
    correctedPath = [];
    
    for i = 1:length(odom2d)
        pose = odom2d(i, 2:end)';
        poseInc = common.PoseDiff(lastPose', pose');
        lastPose = pose;
        
        correctedPose = correctedPose + (correctMatrix * poseInc);
        correctedPath = [correctedPath; odom2d(i, 1), correctedPose'];
    end
end

