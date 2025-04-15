marker_size = [3, 6]; % [row, col]
desired_mm = 500; % distance between two crosspoints

[temp_1, temp_2] = generateCellTemp();

marker_row1 = [];
marker_row2 = [];
for i=1:marker_size(2)
    if mod(i,2)
        marker_row1 = [marker_row1 temp_2];
        marker_row2 = [marker_row2 temp_1];
    else
        marker_row1 = [marker_row1 temp_1];
        marker_row2 = [marker_row2 temp_2];
    end
end
marker_img=[];
for i=1:marker_size(1)
    if mod(i,2)
        marker_img = [marker_img; marker_row1];
    else
        marker_img = [marker_img; marker_row2];
    end
end

desired_width_inches = desired_mm / 25.4;
desired_height_inches = desired_mm / 25.4;

imshow(marker_img);
imwrite(marker_img,['marker_', num2str(marker_size(1)), 'x', num2str(marker_size(2)), '.bmp']);

screen_dpi = 96;
desired_width_pixels = desired_width_inches * screen_dpi;
desired_height_pixels = desired_height_inches * screen_dpi;
set(gcf, 'Position', [100, 100, desired_width_pixels, desired_height_pixels]);
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [desired_width_inches, desired_height_inches]);
exportgraphics(gcf,['marker_', num2str(marker_size(1)), 'x', num2str(marker_size(2)), '.pdf'],'ContentType','vector');

function [temp, temp_inv] = generateCellTemp()
    temp = ones(900,900);

    center_x = 450;
    center_y = 450;
    theta_circ = linspace(0, pi*2, 500); 
    r_circ = ones(size(theta_circ)) * 300;    
    x = r_circ .* cos(theta_circ) + center_x;       
    y = r_circ .* sin(theta_circ) + center_y; 
    temp = insertShape(temp,'FilledPolygon',[[center_x;x';center_x],[center_y;y';center_y]],'Color','black','Opacity',1,'SmoothEdges', false);

    % 定义扇形的参数
    theta = linspace(0, pi/2, 500); % 扇形的角度范围
    r = ones(size(theta)) * 150;          % 扇形的半径
    x = r .* cos(theta) + center_x + 1;            % 扇形在x轴的坐标
    y = r .* sin(theta) + center_y + 1;            % 扇形在y轴的坐标
    temp = insertShape(temp,'FilledPolygon',[[center_x+1;x';center_x+1],[center_y+1;y';center_y+1]],'Color','white','Opacity',1,'SmoothEdges', false);

    theta = linspace(-pi, -pi/2, 500); % 扇形的角度范围
    r = ones(size(theta)) * 150;          % 扇形的半径
    x = r .* cos(theta) + center_x;            % 扇形在x轴的坐标
    y = r .* sin(theta) + center_y;            % 扇形在y轴的坐标
    temp = insertShape(temp,'FilledPolygon',[[center_x;x';center_x],[center_y;y';center_y]],'Color','white','Opacity',1,'SmoothEdges', false);
    
    temp_inv = imrotate(temp, 90);
end
