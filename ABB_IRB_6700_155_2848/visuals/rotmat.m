function R = rotmat(ax,theta)
switch ax
    case 'x'
        R = [[1 0 0];[0 cos(theta) -sin(theta)];[0 sin(theta) cos(theta)]];
    case 'y'
        R = [[cos(theta) 0 sin(theta)];[0 1 0];[-sin(theta) 0 cos(theta)]];
    case 'z'
        R = [[cos(theta) -sin(theta) 0];[sin(theta) cos(theta) 0];[0 0 1]];
end