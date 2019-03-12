% given a starting and and end position of the poolygon's central axis
% as well as thickness values, returns 8 points defining this shape in
% cartesian corrdingates

function out = rect3D(pts,t,n)
r2 = pts(:,2) - pts(:,1);
n2 = norm(r2);
r2 = r2/n2;
o = [0 0 1].';
R2 = 2*(r2 + o)*(r2 + o).'/((r2 + o).'*(r2 + o)) - eye(3);
out = zeros(3,n+1,2);

for i = 1:n
    for j = 1:2
       T = R2*rotmat('z',(i-1)*pi/n*2)*[[t*sqrt(2)/2];[t*sqrt(2)/2];[0]] + pts(:,j);
       out(:,i,j) = T;
    end
end
out(:,n+1,:) = out(:,1,:);
end