function jointPose = joint_optimization(Z, Keys, N)
% input: Z is from DVO, twist between key frames
%        Keys is the factor
%        N is window size
% output: poses of the other nodes w.r.t the first node in the window

idx_off = Keys{1}(1)-1;

% measurement noise covariance
Sigma = diag([0.05^2, 0.03^2, 0.03^2, 0.05^2, 0.05^2, 0.05^2]);
% Sigma = eye(6);
% Cholesky factor of covariance for sampling
Lz = chol(Sigma, 'lower');

T_init = cell(N,1);
T_init{1} = eye(4);
for i = 2:N
    T_init{i} = T_init{i-1}*Z{i-1}; 
end

% Jacobian matrix
A = zeros(6 + 6 * length(Z), 6 * length(T_init));
% right hand side (residuals)
dx = zeros(6 + 6 * length(Z),1);
% anchor node covariance; we want to fix the node so the covariance should
% be small. This will result in large weights in the optimization process.
Sigma_init = eye(6) * 0.1^2;
A(1:6,1:6) = chol(Sigma_init, 'lower') \ eye(6);
% A(1:6,1:6) = eye(6);

% Gauss-Newton solver over SE(3)
T_est = T_init;
max_iter = 100;
iter = 0;
eps_Jr = 1e-9;

% SE(3) right Jacobian inverse and adjoint
invJr = @RightJacobianInverse_SE3;
Ad = @Adjoint_SE3;
while iter < max_iter
    iter = iter + 1;
    % compute residual
    r = dx;
    r(1:6,1) = wedge(logm(T_est{1}));
%     b = x_target(1:2);
    for i = 1:length(Keys)
        key = Keys{i};
        res_idx = 6*i+1:6*i+6;
        r(res_idx, 1) =  wedge(logm(Z{i} \ (T_est{key(1)-idx_off} \  T_est{key(2)-idx_off})));
        % fill in Jacobian of the corresponding target position
        idx = 6 * (key(1)-idx_off) - 5;
        A(res_idx, idx:idx+5) = Lz \ -invJr(r(res_idx, 1)) * Ad(T_est{key(2)-idx_off} \  T_est{key(1)-idx_off});
        idx = 6 * (key(2)-idx_off) - 5;
        A(res_idx, idx:idx+5) = Lz \ invJr(r(res_idx, 1));
        r(res_idx, 1) =  Lz \ r(res_idx, 1);
    end
    
    % solve normal equations
    Jr = -A' * r;
    dx = (A' * A) \ Jr;
    % retract and update the estimate
    for i = 1:length(T_est)
        xi = dx(6*i-5:6*i);
        T_est{i} = T_est{i} * expm(hat(xi));
        
    end
    
    % check if converged
    if norm(Jr) < eps_Jr
        break;
    end
end

% pose is not the same as twist!
% pose{1} = eye, pose{2} = tf(1->2), pos{3} = tf(1->3)
jointPose = T_est;

end

