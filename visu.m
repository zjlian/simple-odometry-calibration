%% 可视化方便类
classdef visu
methods(Static)

    function Begin(message)
        figure;
        title(message);
    end

    function End()
        xlabel('X (m)');
        ylabel('Y (m)');
        grid on;
        axis equal;
    end

    function DrawPose2d(list, color)
        x = list(:, 2);
        y = list(:, 3);
        theta = list(:, 4);

        % 箭头参数
        arrowLength = 0.05; % 箭头的长度
        arrowWidth = 0.02; % 箭头宽度
        
        % 计算箭头的末端坐标
        endX = x + arrowLength * cos(theta);
        endY = y + arrowLength * sin(theta);
        quiver(x, y, endX - x, endY - y, 0, 'MaxHeadSize', arrowWidth, 'Color', color);
        hold on;
    end



end
end