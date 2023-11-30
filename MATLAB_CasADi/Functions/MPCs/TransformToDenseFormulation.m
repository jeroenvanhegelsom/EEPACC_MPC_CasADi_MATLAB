function [H,c,G,g_lb,g_ub,Psi,d] = TransformToDenseFormulation(N_hor, A, B, D, H_sparse, c_sparse, G_sparse, g_lb_sparse, g_ub_sparse, n_x, n_u, s_curr, v_curr)
% TransformToDenseFormulation Transforms the QP matrices, vectors and bounds to the dense forms
%
%   Inputs:
%       N_hor       : number of steps in the horizon
%       A           : state space system matrix
%       B           : state space system matrix
%       D           : state space system matrix
%       H_sparse    : sparse Hessian matrix (matrix of quadratic terms in objective) 
%       c_sparse    : sparse vector defining the linear terms in the objective 
%       G_sparse    : sparse matrix defining the linear constraints
%       g_lb        : sparse lower bound vector for the constraints
%       g_ub        : sparse upper bound vector for the constraints
%       n_x         : number of states in one step
%       n_u         : number of controls in one step
%       s_curr      : current travel distance
%       v_curr      : current velocity
%
%   Outputs:
%       H        : dense matrix defining the quadratic terms in the objective 
%       c        : dense vector defining the linear terms in the objective 
%       G        : dense vector defining the constraints 
%       g_lb     : dense lower bound vector for the constraints
%       g_ub     : dense upper bound vector for the constraints
%       Psi      : transformation matrix (linear part of the transformation)
%       d        : transformation vector (translation part of the transformation)

    %% Initialization

    Psi   = zeros(N_hor*n_u + N_hor*n_x, N_hor*n_u);
    d     = zeros(N_hor*n_u + (N_hor+1)*n_x,1);
    x_0   = [s_curr; v_curr];
 
    %% Construct vector d and calculate dense c and bounds
    
    o = 1;
    for k = 0:N_hor
    
        % counter for indexing in matrices
        kk = k+1;
    
        % column blocks related to states
        if k > 0
            
            % calculate Psi
            for i = 0:k-1
                A_ = eye(n_x);
                for j = 1:(k-i-1)
                    A_ = A_ * squeeze(A(kk-j,:,:));
                end
                Psi(o:o+n_x-1,(i*n_u+1):(i*n_u+n_u)) = A_*squeeze(B(i+1,:,:));
            end

            % contribution of x_0
            A_ = eye(n_x);
            for i = 1:k
                A_ = A_ * squeeze(A(kk-i,:,:));
            end
            d(o:o+n_x-1) = d(o:o+n_x-1) + A_*x_0;

            % contribution of previous 'constant' parts D
            for i = 0:k-1
                A_ = eye(n_x);
                for j = 1:(k-i-1)
                    A_ = A_ * squeeze(A(kk-j,:,:));
                end
                d(o:o+n_x-1) = d(o:o+n_x-1) + A_*D(i+1,:)';
            end

        else
            % k=0
            Psi(o:o+n_x-1,:) = 0;
            d(o:o+n_x-1) = x_0;
        end
        o = o + n_x;
    
        % column blocks related to controls
        if k < N_hor
            Psi(o:o+n_u-1,k*n_u+1:(k+1)*n_u) = eye(n_u);
            d(o:o+n_u-1) = zeros(n_u,1);
            o = o + n_u;
        end
    
    end

    % Calculate dense versions of the matrices and vectors
    H    = Psi'*H_sparse*Psi;
    c    = Psi'*(.5*(H_sparse+H_sparse')*d+c_sparse);
    G    = G_sparse*Psi;
    g_lb = g_lb_sparse - G_sparse*d;
    g_ub = g_ub_sparse - G_sparse*d;

end


