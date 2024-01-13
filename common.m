classdef common
    methods(Static)
        
        % 加载csv文件到二维数组
        function dataArray = LoadCSV(filename)
            opts = detectImportOptions(filename, 'NumHeaderLines', 0);
            data = readtable(filename, opts);
            dataArray = table2array(data);
        end
        
        % 计算 form 局部坐标系下 to 的位姿
        % 参数接受的是行向量描述的位姿 <x, y, rad>
        function diff = PoseDiff(form, to)

            diff = [0, 0, 0]';
            % form
            r_form = eul2rotm([0, 0, form(3)], "XYZ");
            t_form = [form(1:2), 0]';
            % to
            r_to = eul2rotm([0, 0, to(3)], "XYZ");
            t_to = [to(1:2), 0]';
            
            r_diff = inv(r_form) * r_to;
            eul_diff = rotm2eul(r_diff, "XYZ");
            diff = t_to - t_form;
            diff(3) = eul_diff(3);

            % 其实直接 to - form 也行
        end
    end
end

