function y = smoothing_t0(u)

d = 1.0;
n = 16;
s = (1/2*d)/((n-1)/n)^(1/n);
%u=0:0.01:2.0;
y=	1 - exp(-(u/s).^n);

% figure
% plot(u,y)



