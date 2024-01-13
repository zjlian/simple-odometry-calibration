% 求解变换矩阵
function matrix = calibration(truth2d, odom2d)
    A = [];
    b = [];
    lastOdom = odom2d(1, 2:end);
    lastTruth = truth2d(1, 2:end);
    for i = 1:length(odom2d)
        

        o = common.PoseDiff(lastOdom, odom2d(i, 2:end));
        lastOdom = odom2d(i, 2:end);
        t = common.PoseDiff(lastTruth, truth2d(i, 2:end));
        lastTruth = truth2d(i, 2:end);

        %o = odom2d(i, :);
        %t = truth2d(i, :);
        
        A = [
            A;
            o(1), o(2), o(3), 0,    0,    0,      0,    0,    0;
            0,    0,    0,      o(1), o(2), o(3), 0,    0,    0;
            0,    0,    0,      0,    0,    0,      o(1), o(2), o(3);
            ];
        b = [
            b;
            t(1);
            t(2);
            t(3);
            ];
    end
    
    % matrix = inv(A' * A) * A' * b;
    matrix = pinv(A) * b;
    matrix = [
        matrix(1), matrix(2), matrix(3);
        matrix(4), matrix(5), matrix(6);
        matrix(7), matrix(8), matrix(9);
        ];
end


