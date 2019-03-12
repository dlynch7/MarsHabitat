% ***
% Given a series of SE matrices T, where T{1} is location of the base, 
% link thivcknesses, link_t [...], simulate(...) creates a 3D representation 
% of the robot

function out = simulate(T,link_t)
%% define visual paramters here
axis([-5 5 -5 5 -.2 5])
base_thickness = .2;
base_rad = 1;
cw = 4;
show_edges = 1;
grid on;
%% functional
[X,Y,Z] = cylinder(base_rad,100);
X = X + T{1}(1,4);
Y = Y + T{1}(2,4);
Z = Z + T{1}(3,4);
hold on
a = surf(X,Y,-Z*base_thickness);
a.EdgeAlpha = 0;
a.FaceColor = [.5 .5 .5];
a = fill(X(:),Y(:),[.5 .5 .5]);
for i = 1:length(T)-1
      pp = [T{i}(1:3,4),T{i+1}(1:3,4)];
      out = rect3D(pp,link_t(i),cw);
      X = squeeze(out(1,:,:));
      Y = squeeze(out(2,:,:));
      Z = squeeze(out(3,:,:));
      a = surf(X,Y,Z);
      a.EdgeAlpha = show_edges;
      a.FaceColor = [.7 .7 .8];
      a = fill3(X(1:cw,1).',Y(1:cw,1).',Z(1:cw,1).',ones(cw,1));
      a.FaceColor = [.7 .7 .8];
      a = fill3(X(1:cw,2).',Y(1:cw,2).',Z(1:cw,2).',ones(cw,1));      
      a.FaceColor = [.7 .7 .8];
end

end