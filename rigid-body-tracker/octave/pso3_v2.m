% ======================================================== %
% The Particle Swarm Optimization
% ======================================================== %
% Usage: pso(number_of_particles,Num_iterations)
% eg:   best=pso(20,10);
% where best=[xbest ybest zbest]  %an n by 3 matrix
%   xbest(i)/ybest(i) are the best at ith iteration

function [best]=pso3_v2(n, Num_iterations, init)
% n=number of particles
% Num_iterations=total number of iterations
if nargin<2,   Num_iterations=20;  end
if nargin<1,   n=25;          end

% range=[xmin xmax ymin ymax zmin zmax];
Rx = 1;
Ry = 1;
Rz = 1;

range=[init(1)-Rx init(1)+Rx init(2)-Ry init(2)+Ry init(3)-Rz init(3)+Rz];

% ----------------------------------------------------
% Setting the parameters: alpha, beta
% Random amplitude of roaming particles alpha=[0,1]
% alpha=gamma^t=0.7^t;
% Speed of convergence (0->1)=(slow->fast)
beta=0.5;
% ----------------------------------------------------
% Grid values of the objective function
% These values are used for visualization only
Ngrid=30;
dx=(range(2)-range(1))/Ngrid;
dy=(range(4)-range(3))/Ngrid;
dz=(range(6)-range(5))/Ngrid;
xgrid=range(1):dx:range(2); ygrid=range(3):dy:range(4); zgrid=range(5):dz:range(6);
[x,y,z]=meshgrid(xgrid,ygrid,zgrid);
%u=f(x,y,z);
if exist("saved_u.mat")
  load saved_u.mat
else
  u=match_cost(x,y,z);
  save saved_u.mat u
endif

% Display the shape of the function to be optimized
figure(1);
%surfc(x,y,u(:,:,1)); % z is omitted
% ---------------------------------------------------
best=zeros(Num_iterations,4);   % initialize history
% ----- Start Particle Swarm Optimization -----------
% generating the initial locations of n particles
[xn,yn,zn]=init_pso(n,range);
% Display the paths of particles in a figure
% with a contour of the objective function
%figure(2);
% Start iterations
for i=1:Num_iterations,
%figure(i);
% Show the contour of the function
  %contour(x,y,u(:,:,1),15); hold on;
% Find the current best location (xo,yo)
un=match_cost(xn,yn,zn);
un_min=min(un);
xo=min(xn(un==un_min));
yo=min(yn(un==un_min));
zo=min(zn(un==un_min));
uo=min(un(un==un_min));
% Trace the paths of all roaming particles
% Display these roaming particles
%plot(xn,yn,'.',xo,yo,'*'); axis(range);
% The accelerated PSO with alpha=gamma^t
gamma=0.7; alpha=gamma.^i;
% Move all the particles to new locations
[xn,yn,zn]=pso_move(xn,yn,zn,xo,yo,zo,alpha,beta,range);
%drawnow;
% Use "hold on" to display paths of particles
%hold off;
% History
best(i,1)=xo; best(i,2)=yo; best(i,3)=zo; best(i,4)=uo;
end   %%%%% end of iterations



% ----- Subfunctions  -----

% Intial locations of n particles
function [xn,yn,zn]=init_pso(n,range)
xrange=range(2)-range(1); yrange=range(4)-range(3); zrange=range(6)-range(5);
xn=rand(1,n)*xrange+range(1);
yn=rand(1,n)*yrange+range(3);
zn=rand(1,n)*zrange+range(5);

% Move all the particles toward (xo,yo)
function [xn,yn,zn]=pso_move(xn,yn,zn,xo,yo,zo,a,b,range)
nn=size(yn,2);  %a=alpha, b=beta
xn=xn.*(1-b)+xo.*b+a.*(rand(1,nn)-0.5);
yn=yn.*(1-b)+yo.*b+a.*(rand(1,nn)-0.5);
zn=zn.*(1-b)+zo.*b+a.*(rand(1,nn)-0.5);
[xn,yn,zn]=findrange(xn,yn,zn,range);

% Make sure the particles are within the range
function [xn,yn,zn]=findrange(xn,yn,zn,range)
nn=length(yn);
for i=1:nn,
   if xn(i)<=range(1), xn(i)=range(1); end
   if xn(i)>=range(2), xn(i)=range(2); end
   if yn(i)<=range(3), yn(i)=range(3); end
   if yn(i)>=range(4), yn(i)=range(4); end
   if zn(i)<=range(5), zn(i)=range(5); end
   if zn(i)>=range(6), zn(i)=range(6); end
end
