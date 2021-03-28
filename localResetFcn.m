function in = localResetFcn(in)
% reset 
in = setVariable(in,'v', (80)); % random value for initial velocity of the vehicle
in = setVariable(in,'LMUX',  (0.34)); % random value for longitudinal friction
in = setVariable(in,'LMUY',  (0.34)); % random value for lateral friction

end
% lowest 0.34

% function in = localResetFcn(in)
% % reset 
% in = setVariable(in,'v', (80+50*rand)); % random value for initial velocity of the vehicle
% in = setVariable(in,'LMUX',  (0.4+0.25*rand)); % random value for longitudinal friction
% in = setVariable(in,'LMUY',  (0.4+0.25*rand)); % random value for lateral friction
% 
% end