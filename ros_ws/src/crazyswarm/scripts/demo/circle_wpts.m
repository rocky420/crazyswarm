nb_agents = 6;

delta_theta = 2*pi/nb_agents;
theta = 0:pi/3:(2-1/3)*pi;
r = 1.5 * ones(1,nb_agents);
y = r .* sin(theta);
x = r .* cos(theta);

Y = max(y) + y;
X = max(x) + x;

k = 2;
Y = circshift(Y,k);
X = circshift(X,k);

plot(X,Y);
axis equal;

n = 1:1:nb_agents;
data = [n', X', Y'];

%write data to end of file
dlmwrite('target1.csv',data);
