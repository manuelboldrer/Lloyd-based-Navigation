function [v,omega] = controller(theta, h_reference, v_old, dt, cx,cy,x,y,Theta,omega_old,kk,L)
%% Read parameters
k_brake      = 15  ;
k_angular    = 20  ;%5   ;
alpha        = 0.5 ;
k_i          = 10;%5   ;
v_des        = 2.5;%0.5 ;
n_agents     = size(theta,2);
h            = zeros(2,n_agents);
v            = zeros(n_agents,1);
omega        = zeros(n_agents,1);

for j = 1:n_agents
%% Compute vehicle heading
h(:,j) = [cos(theta(j)); sin(theta(j))]; 
%% Brake
if dot(h(:,j),h_reference(:,:,j))< cos(pi/5)%cos(pi/10) %||  norm([x-cx;y-cy]) < 0.3
    v(j) = v_old(j) - k_brake * v_old(j) * dt;
    v(j) = max(v(j),0);
else
    v(j) = v_old(j) - k_i * (v_old(j) - v_des) * dt;
    v(j) = max(v(j),0);
end
%% Steer the vehicle

omega(j) = -k_angular * (1 - dot(h(:,j), h_reference(:,:,j)))^alpha * sign(h_reference(1,:,j) * h(2,j) - h_reference(2,:,j) * h(1,j));...
           %%- L(j,:)*(cx-x)';
%you can add to omega a consensus term here



end
% omega_sign = omega;
% for g = 1:100
%     omega_sign =  (eye(n_agents)-dt*L)*omega_sign;
% end
% segno = sign(omega_sign);
% for j = 1:n_agents
%     if sign(omega(j)) ~= segno(j)
%         omega(j) = omega(j)*-1;
%     end
% end
% % omegaL
% for g = 1:100
%     omega =  (eye(n_agents)-dt*L)*omega;
% end

% omega = omega +dt*omega_dot;
%  end

% omega = max(omega,0);