%% 预处理数据集文件格式到 <时间戳 X Y 弧度>，并且对轨迹点做运动距离降采样和时间同步
function preprocess()
odom = LoadCSV("dataset/odometry_mu_100hz.csv");
truth = LoadCSV("dataset/groundtruth.csv");
odom2d = ToPose2d(odom);
truth2d = ToPose2d(truth);

disp("简单处理...");
odom2d = Run(odom2d);
size(odom2d)
truth2d = Run(truth2d);
size(truth2d)
disp("同步时间...");
[truth2d, odom2d] = SyncTimeFilter(truth2d, odom2d, 20);
size(odom2d)
size(truth2d)

visu.Begin("轨迹预处理");
visu.DrawPose2d(odom2d, "r");
visu.DrawPose2d(truth2d, "g");
visu.End();

writematrix(odom2d, "dataset/odom2d.csv")
writematrix(truth2d, "dataset/truth2d.csv")
end

function result = Run(csvList)
result = [];
% result = ToPose2d(csvList);
result = MoveToZero(csvList);
result = Dispersed(result, 0.05);
end

% 按时间同步数据
function [result1, result2] = SyncTimeFilter(input1, input2, maxdiff)
% 在 list 中查找与 timestamp 时间接近的行
    function index = Find(timestamp, maxdiff, list, begin_index)
        index = -1;
        for i1 = 1:length(list)
            t = list(i1, 1);
            if abs(timestamp - t) < maxdiff
                index = i1;
                return;
            end
        end
    end

result1 = [];
result2 = [];
last_found_line = 0;
for i = 1:length(input1)
    timestamp = input1(i, 1);
    found = Find(timestamp, maxdiff, input2, last_found_line);
    if found == -1
        continue;
    end
    last_found_line = found;
    result1 = [result1; input1(i, :)];
    result2 = [result2; input2(found, :)];
    
    if mod(i, 1000) == 0
        fprintf("filtered %.2f ...\n", (i / length(input1)) * 100);
    end
end

end


function output = ToPose2d(table)
output = zeros(length(table), 4);
for i = 1:length(table)
    output(i, :) = [table(i, 1) / 1000, table(i, 2), table(i, 3), table(i, end)];
end
end

function output = MoveToZero(poseList)
output = poseList;
output(:, 2:end) = output(:, 2:end) - output(1, 2:end);
end

function output = Dispersed(poseList, distance)
output = [poseList(1, :)];
lastPose = [poseList(1, 2), poseList(1, 3), poseList(1, end)]';
for i = 1:length(poseList)
    pose = [poseList(i, 2), poseList(i, 3), poseList(i, end)]';
    inc = pose - lastPose;
    
    d = sqrt(inc(1) * inc(1) + inc(2) * inc(2));
    if d > distance
        output = [output; poseList(i, :)];
        lastPose = pose;
    end
end
end

function dataArray = LoadCSV(filename)
opts = detectImportOptions(filename, 'NumHeaderLines', 0);
data = readtable(filename, opts);
dataArray = table2array(data);
end

