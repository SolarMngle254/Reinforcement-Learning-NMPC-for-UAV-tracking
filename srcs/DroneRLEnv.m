% Author : Le Minh Nhat - K22 HCMUT Electrical & Electronics Faculty 
% Inspiration : https://www.youtube.com/@atvu5238/ (Dat Vu) 
% Date modified: 30/08/2026


classdef DroneRLEnv < rl.env.MATLABEnvironment
    properties
        Ts = 0.1;
        MaxSteps = 150;
        CurrentStep = 0;
        State = zeros(12,1);
        U_prev = [0; 0; 0];
        omega_total = 400;
        y_min; y_max; hz;
        TimeArray; RefTraj;
    end
    
    properties(Access = protected)
        IsDone = false;
    end
    
    methods
        function this = DroneRLEnv()
            % 1. Observation: 10 state errors + 9 lookahead states = 19
            ObservationInfo = rlNumericSpec([19 1]);
            ObservationInfo.Name = 'Normalized_Lookahead_States';
            
            % 2. Action: Neural network action space is constrained to [-1, 1]
            %    to prevent gradient saturation.
            ActionInfo = rlNumericSpec([3 1], 'LowerLimit', -1, 'UpperLimit', 1);
            ActionInfo.Name = 'Neural_Action';
            
            this = this@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
            
            constants = initial_constants();
            this.hz = constants('hz');
            ct = constants('ct'); cq = constants('cq'); l = constants('l');
            omega_min = constants('omega_min'); omega_max = constants('omega_max');
            
            this.y_min = [ct*l*(omega_min^2 - omega_max^2); ct*l*(omega_min^2 - omega_max^2); cq*(-2*omega_max^2 + 2*omega_min^2)];
            this.y_max = [ct*l*(omega_max^2 - omega_min^2); ct*l*(omega_max^2 - omega_min^2); cq*(-2*omega_min^2 + 2*omega_max^2)];
        end
        
        function [Observation, Reward, IsDone, LoggedSignals] = step(this, Action)
            % Scale the physical control signal:
            % Map the neural action from [-1, 1] to a small safe compensation range [-0.01, 0.01].
            dU = double(Action) * 0.01;
            
            idx = this.CurrentStep + 1;
            X = this.RefTraj{1}; Xd = this.RefTraj{2}; Xdd = this.RefTraj{3};
            Y = this.RefTraj{4}; Yd = this.RefTraj{5}; Ydd = this.RefTraj{6};
            Z = this.RefTraj{7}; Zd = this.RefTraj{8}; Zdd = this.RefTraj{9};
            Psi = this.RefTraj{10};
            
            xr = X(idx,2); yr = Y(idx,2); zr = Z(idx,2); psi_r = Psi(idx,2);
            
            [Phi_ref, Theta_ref, U1] = pos_controller(xr, Xd(idx,2), Xdd(idx,2), ...
                yr, Yd(idx,2), Ydd(idx,2), zr, Zd(idx,2), Zdd(idx,2), ...
                psi_r, this.State);
            
            Phi_h = Phi_ref * ones(this.hz, 1);
            Theta_h = Theta_ref * ones(this.hz, 1);
            Psi_h = psi_r * ones(this.hz, 1);
            
            x_att0 = [this.State(10:12); this.State(4:6)];
            lb = repmat(this.y_min, this.hz, 1);
            ub = repmat(this.y_max, this.hz, 1);
            u_seq0 = repmat(this.U_prev, this.hz, 1);
            
            consts_local = initial_constants();
            cost_fun = @(u_seq) attitude_cost_nmpc(u_seq, x_att0, ...
                Phi_h, Theta_h, Psi_h, this.U_prev, this.hz, ...
                this.Ts, consts_local, this.omega_total);
            
            fmin_opts = optimoptions('fmincon', ...
                'Algorithm', 'sqp', ...
                'Display', 'off', ...
                'MaxIterations', 20, ...
                'OptimalityTolerance', 1e-2, ...
                'StepTolerance', 1e-3);
            
            [u_seq_opt, ~, exitflag] = fmincon( ...
                cost_fun, u_seq0, [],[],[],[], lb, ub, [], fmin_opts);
            
            if isempty(u_seq_opt) || exitflag <= 0
                U_nmpc = this.U_prev;
            else
                U_nmpc = u_seq_opt(1:3);
            end
            
            this.U_prev = U_nmpc;
            U_final = U_nmpc + dU;
            U_total = [U1; U_final];
            
            tspan = [0 this.Ts];
            global omega_total;
            omega_total = this.omega_total;
            
            [~, x_traj] = ode45( ...
                @(t,x) nonlinear_drone_model(t,x,U_total), ...
                tspan, this.State);
            
            this.State = x_traj(end,:)';
            
            ct = consts_local('ct');
            U1_sat = max(min(U1, ...
                ct*4*consts_local('omega_max')^2), ...
                ct*4*consts_local('omega_min')^2);
            
            this.omega_total = sqrt(U1_sat / ct);
            
            % --- State normalization and lookahead features ---
            ex = (xr - this.State(7)) / 2.0;
            ey = (yr - this.State(8)) / 2.0;
            ez = (zr - this.State(9)) / 2.0;
            epsi = wrapToPi(psi_r - this.State(12)) / pi;
            
            uvw = this.State(1:3) / 5.0;
            pqr = this.State(4:6) / 5.0;
            
            idx1 = min(idx + 3, length(X));
            idx2 = min(idx + 6, length(X));
            idx3 = min(idx + 10, length(X));
            
            la1 = ([X(idx1,2); Y(idx1,2); Z(idx1,2)] ...
                - this.State(7:9)) / 2.0;
            
            la2 = ([X(idx2,2); Y(idx2,2); Z(idx2,2)] ...
                - this.State(7:9)) / 2.0;
            
            la3 = ([X(idx3,2); Y(idx3,2); Z(idx3,2)] ...
                - this.State(7:9)) / 2.0;
            
            Observation = [ex; ey; ez; epsi; uvw; pqr; la1; la2; la3];
            
            prmR.w_pos = 10;
            prmR.w_yaw = 5;
            prmR.w_att = 2;
            prmR.w_u = 0.1;
            
            Reward = compute_reward_UAV( ...
                this.State, U_final, xr, yr, zr, psi_r, prmR);
            
            this.CurrentStep = this.CurrentStep + 1;
            pos_error = norm(this.State(7:9) - [xr; yr; zr]);
            
            if pos_error > 2.0 || this.CurrentStep >= this.MaxSteps
                IsDone = true;
                
                if pos_error > 2.0
                    Reward = Reward - 1000; % Large penalty for excessive tracking error
                end
            else
                IsDone = false;
            end
            
            this.IsDone = IsDone;
            LoggedSignals = [];
        end
        
        function InitialObservation = reset(this)
            this.CurrentStep = 0;
            this.IsDone = false;
            
            % Use the spiral trajectory as the initial training trajectory
            % to provide a stable and consistent learning task.
            traj.type = 2;
            traj.scale = 0.5;
            traj.R0 = 3;
            traj.kR = 0.05;
            traj.z0 = 0.5;
            traj.kz = 0.02;
            traj.omega = 0.3;
            
            this.TimeArray = ...
                (0:this.Ts:(this.MaxSteps + this.hz + 10)*this.Ts)';
            
            [X, Xd, Xdd, Y, Yd, Ydd, Z, Zd, Zdd, Psi] = ...
                trajectory_generator(this.TimeArray, traj);
            
            this.RefTraj = {X, Xd, Xdd, Y, Yd, Ydd, Z, Zd, Zdd, Psi};
            
            % Domain randomization:
            % Apply small initial position perturbations to reduce overfitting.
            this.State = zeros(12,1);
            this.State(7) = X(1,2) + (rand()-0.5)*0.2;
            this.State(8) = Y(1,2) + (rand()-0.5)*0.2;
            this.State(9) = Z(1,2);
            this.State(12) = Psi(1,2);
            
            this.U_prev = [0;0;0];
            this.omega_total = 400;
            
            ex = (X(1,2) - this.State(7))/2.0;
            ey = (Y(1,2) - this.State(8))/2.0;
            ez = 0;
            epsi = 0;
            
            uvw = zeros(3,1);
            pqr = zeros(3,1);
            
            la1 = ([X(4,2); Y(4,2); Z(4,2)] ...
                - this.State(7:9))/2.0;
            
            la2 = ([X(7,2); Y(7,2); Z(7,2)] ...
                - this.State(7:9))/2.0;
            
            la3 = ([X(11,2); Y(11,2); Z(11,2)] ...
                - this.State(7:9))/2.0;
            
            InitialObservation = ...
                [ex; ey; ez; epsi; uvw; pqr; la1; la2; la3];
        end
    end
end
