%% Vehicle parameters
CF = 1.6*10^5;
CR = 1.6*10^5;
g = 9.8;
k = 0.3; 
hg = 0.58;
% Vehicle parameters 
lf	=1.3265; 
lr	= 1.3685;
L = lr+lf;
c	=(1.596+1.610)/2;
ft = 1.596/2;
rt = 1.610/2;
mf = 55.0; 
mr = 57.0;
Reff = 0.323;
v = 80;
M  =1360;  
Kv = (M*lr)/(L*2*CF)-(M*lf)/(L*2*CR);
J_FR = 20*Reff^2/2;
J_FL = 20*Reff^2/2;
J_RR = 20*Reff^2/2; 
J_RL = 20*Reff^2/2;  
% unsprungJzz 	=2*mf*(lf^2 + ft^2) + 2*mr*(lr^2 + rt^2);
% Jzz 	=1782.0 + unsprungJzz;
Jzz 	=2050;


