function gBest = pso3func(X, Y, R, P, X0)
# ========== PARTICAL SWARM OPTIMIZATION SIMULATION - 3d Yin Yang ==========================
# 
# 	EXAMPLE: 	minimize 	f(x, y) = x * e^-(x^2 + y^2)
#		
#	GROUP MEMBERS:	 Emily Le, Mathew Dohlen, Steven Sairafian, Wynne Tran
#
# =========================================================================================

global gR = R
global gP = P

# ========== DEFINE CONSTANTS & FUNCTIONS =================================================
#
#	n 			= number of particles in the system
#	c1, c2 		= importace weight of personal and global best
#					- usually c1 + c2 = 4 or c1 = c1 = 2
#	vMax		= speed of particles from iteration to iteration
#	iteration 	= number of iterations before ending the program
#	
# ========================================================================================= 
n = 50;
c1 = 0.5;
c2 = 0.5;
vMax = 0.05;
iteration = 65;	 

# ========== INITIALIZATION ===============================================================
#
# 	initialzing intiial positions, vectors, lBest, gBest for the first iteration
#
# =========================================================================================
# vector
vector = zeros(n,3);

# position
position = zeros(n,3);
u = zeros(n,1);

x = (rand(1,n)* 4) - 2; 
y = (rand(1,n)* 4) - 2;
z = (rand(1,n)* 4) - 2;

for i = 1:n
	position(i,1) = x(i) + X0(1);
	position(i,2) = y(i) + X0(2);
  position(i,3) = z(i) + X0(3);
	u(i,1) = f(X,Y,x(i),y(i),z(i));
endfor

# lBest
lBest = position;

# gBest
gBest = zeros(1,3) + 1000;
gBest = findGBest(X, Y, gBest, position);

# ========== PLOT INITIAL POINTS ON GRAPH =====================================================
#  
# `Will plot the initial scatter plot on mesh grapgh for the first iteration
#	`	- note that the velocities are initilzed to be 0
# 
# =============================================================================================
#graphPSO(position(:,1), position(:,2), z, 100);

# ========== PSO ITERATION =====================================================================
#
# 	Based on the formula:
#
#		v[] = v[] + c1 * rand() * (pbest[] - present[]) + c2 * rand() * (gbest[] - present[]) 
#		present[] = persent[] + v[] 
#
#	This essentially will update the velocity that will be used to update the new position after
#	each iteration.
#
# ==============================================================================================
for i = 1:iteration
  fprintf("iteration %d\n", i)
	for j = 1:n
		prevPosition = position;
		r1 = rand(1,1);
		r2 = rand(1,1);
		vector(j,:) = vector(j,:) + c1 * r1 * (lBest(j,:) - position(j,:)) + c2 * r2 * (gBest - position(j,:));

		# Limit x-coordinate velocities
		if (vector(j,1) > vMax)
			vector(j,1) = vMax;
		elseif (vector(j,1) < -vMax)
			vector(j,1) = -vMax;
		endif

		# Limit y-coordinate velocities
		if (vector(j,2) > vMax)
			vector(j,2) = vMax;
		elseif (vector(j,2) < -vMax)
			vector(j,2) = -vMax;
		endif

    # Limit z-coordinate velocities
		if (vector(j,3) > vMax)
			vector(j,3) = vMax;
		elseif (vector(j,3) < -vMax)
			vector(j,3) = -vMax;
		endif
    
		# update x, y and z values
		position(j,:) = position(j,:) + vector(j,:);
		
		# update lBest and gBest values
		if (f(X,Y,position(j,1), position(j,2), position(j,3)) > f(X,Y,prevPosition(j,1),prevPosition(j,2),prevPosition(j,3)))
			lBest(j,:) = prevPosition(j,:);
		endif

		# update z values
		z(j,1) = f(X,Y,position(j,1),position(j,2),position(j,3));
	endfor                                            

	gBest = findGBest(X, Y, gBest, position);

	# draw udated graph
	#graphPSO(position(:,1), position(:,2), z, i+100);
endfor

endfunction

# DEFINED FUNCTION:  will return z values of the graph given x and y values
function retval = f(X, Y, xx, yy, zz)
	#retval = xx * e.^(-(xx.^2 + yy.^2));
  retval = match_cost(X, Y, xx, yy, zz);
  #fprintf("%f\n", retval);
endfunction

# DEFINED FUNCTION: finds gBest
function retval = findGBest(X, Y, gBest, position)
	for i = 1:10
		if (f(X, Y, position(i,1), position(i,2), position(i,3)) < f(X, Y, gBest(1), gBest(2), gBest(3)))
			gBest = position(i,:);
		endif
	endfor
	retval = gBest;
endfunction

function loss = match_cost(X, Y, x, y, z)
  % TODO transform Y by t=[x,y,z]
  % Project Y to camera
  global gR
  global gP
  
  T = gR;
  T(1,4) = x;
  T(2,4) = y;
  T(3,4) = z;

  Yt = (T * [Y ones(size(Y,1),1)]')';
  Xbt = (gP * Yt')';

  logY = 0;
  for i = size(Xbt, 1)
    y = Xbt(i, :);
    logY = logY + log(pdf_point_cloud_2d(X, y));
  endfor
  loss = -logY;
endfunction