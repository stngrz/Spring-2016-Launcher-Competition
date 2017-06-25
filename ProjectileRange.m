function [ maxXLand, maxXLandAngle ] = ProjectileRange( d, v0 )
% computes the maximum diastance and coresponding angle for a given inital
% velocity and lengths of the cannon using vectors
% Inputs: a vector of the lengths of the cannon (m), an inintal velocity(m/s)
% Outputs: maximum distance the cannon can fire (m), angle to achieve the
% maximum distance (degrees)
theta = [0:.01:90];
xLand  = LandingDistance( d, v0, theta );
[maxXLand, maxXLandIndex] = max(xLand);
maxXLandAngle = theta(maxXLandIndex);


end

