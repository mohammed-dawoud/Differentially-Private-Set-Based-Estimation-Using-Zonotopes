classdef Measurement < handle
    %MEASUREMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        LIGHTSPEED = 299792458; % m/s
        % range var is about 0.05 m, so due to timing that's about 1.67e-10
 
%dp         var_dij = 1e-12; % around -20 to converge on type 1 only
        %var_dij = 1e-12;
%dp         var_rij = 1.00;   
        var_Rij =3; %0.30;
        %dp-start %Mohammed Dawoud
        cenDP= [0];
        genDP = 0.1*[1 1 1];
        cenm = [0];
        genm = [0.01 0.02 0.01];
        
        %dp-end

%         var_dij = 1e-12; % around -20 to converge on type 1 only
%         %var_dij = 1e-12;
%         var_rij = 1.00;   
%         var_Rij = 0.30;
        % cross terms
        var_dxr = 0;
        var_dxR = 0;
        var_rxR = 0;
    end
    
    properties
        % measurement type
        type = 0;
        % measurement wall time
        walltime = 0;
        % measurement source (i)
        nodei = 0;
        % measurement destination (j)
        nodej = 0;
        % processed measurements
%dp         d_ij = 0;
%dp         r_ij = 0;
        R_ij = 0;
       
        %dp-start % Mohammed Dawoud
%         nodeIndex=0;
%         % x,y measurements
%         x_meas = 0;
%         y_meas = 0;
%         z_meas = 0;
%         % x,y ground truth
%         x_gnd = 0;
%         y_gnd = 0;
%         z_gnd = 0;
%         x_dp = 0;%dp
%         y_dp = 0;%dp
%         z_dp = 0;
        h = [0 0];
        %dp-end
        
        % processed times
        T_rsp0 = 0;
        T_rsp1 = 0;
        T_rnd0 = 0;
        T_rnd1 = 0;
        % rx quality
        fppwr = 0;
        cirp = 0;
        fploss = 0;
        % measurement filters
        allow_d = false;
        allow_b = false;
        allow_r = false;
        allow_B = false;
        allow_R = false;
        % beaconing raw times
        t_bcn_tx = 0;
        t_bcn_rx = 0;
        t_i = 0;
        t_j = 0;
        % sequence number
        seq = 0;
        % is this a queued message?
        queued = 0;
        t0;
        t1;
        t2;
        t3;
        t4;
        t5;
        attackValue=0;
        attackOffset=0;
    end
    
    methods

        % constructor
        function obj = Measurement( varargin )
            % process raw measurement
            % format: time, dest, src, seq, ts0, ..., ts5, fppwr, cirp, fploss
            
            if nargin >1
                raw =varargin{2};
            obj.type = varargin{1};
            obj.walltime = raw(1);
            obj.nodei = raw(2); % ROS
            obj.nodej = raw(3); % ROS
            obj.seq = raw(4);
            obj.t0 = raw(5);  % i
            obj.t1 = raw(6);  % j
            obj.t2 = raw(7);  % i
            obj.t3 = raw(8);  % j
            obj.t4 = raw(9);  % i
            obj.t5 = raw(10); % j
            obj.t_bcn_tx = obj.t0;
            obj.t_bcn_rx = obj.t1;
            obj.t_j = obj.t5;
            obj.t_i = obj.t4;
            % copy rx quality measurements
            %obj.fppwr = raw(11);
            %obj.cirp = raw(12);
            obj.fploss = raw(11); % ROS
            
            % calculate processed measurements
%dp            obj.d_ij = (obj.t5 - obj.t4);
            %obj.d_ij = (t1 - t0);
            obj.T_rnd0 = obj.t3-obj.t0;
            obj.T_rnd1 = obj.t5-obj.t2;
            obj.T_rsp0 = obj.t2-obj.t1;
            obj.T_rsp1 = obj.t4-obj.t3;
            
%dp             obj.r_ij = (obj.LIGHTSPEED/2)*(obj.T_rnd1 - obj.T_rsp1);
            %obj.R_ij = (obj.LIGHTSPEED/4)*(obj.T_rnd0 - obj.T_rsp1 + obj.T_rnd1 - obj.T_rsp0);
            obj.R_ij = (obj.LIGHTSPEED)*(obj.T_rnd0*obj.T_rnd1 - obj.T_rsp0*obj.T_rsp1)/(obj.T_rnd0 + obj.T_rnd1 + obj.T_rsp0 + obj.T_rsp1);
            %obj.B_ij = ( (t5-t1)/(t4-t0) - 1 );
            end
        end

        % get the vectorized measurements
        function z = vectorize(obj)
            z = [];
%dp start             if obj.allow_d
%                 z = [obj.d_ij];
%             end
%             if obj.type >= 2
%                 if obj.allow_r
%                     z = [z; obj.r_ij];
%                 end
%dp end             end
            if obj.type >= 3
                if obj.allow_R
                    z = [z; obj.R_ij];
                end
            end
        end
        %dp-start
        function z = vectorizedp_optimal_noise_LDP(obj,meascnt)
            z = [];
            
            
            %dp-start-measurement-noise-start
            zm = zonotope([0],[0.01 0.02 0.01]); %dp
            rng(meascnt,'twister'); %dp
            p1 = randPoint(zm); %dp
            %dp-start-measurement-noise-end
            
            %dp-optimal-noise-start
            opt_noise_table = readtable("logs/optimal_noise_f_LDP.csv","ReadVariableNames",true);%12-8-2022 
            opt_noise_tablex=table2array(opt_noise_table(:,1));%12-8-2022 
            opt_noise_tabley=table2array(opt_noise_table(:,2));%12-8-2022
            opt_noise_tabley = opt_noise_tabley/sum(opt_noise_tabley); % Make sure probabilites add up to 1.
            cp_opt = [0, cumsum(opt_noise_tabley.')];
            rng(meascnt,'twister'); %dp
            r1 = rand;
            ind1 = find(r1>cp_opt, 1, 'last');
            resultdp_opt1 = opt_noise_tablex(ind1);
            %r2 = rand;
            %ind2 = find(r2>cp_opt, 1, 'last');
            %resultdp_opt2 = opt_noise_tablex(ind2)
            %figure;
            %plot(opt_noise_tablex,opt_noise_tabley);%12-8-2022
            
           
            %dp-optimal-noise-end
            
            
%dp start             if obj.allow_d
%                 z = [obj.d_ij];
%             end
%             if obj.type >= 2
%                 if obj.allow_r
%                     z = [z; obj.r_ij];
%                 end
%dp end             end
            if obj.type >= 3
                if obj.allow_R
                    z = [z; obj.R_ij+p1+resultdp_opt1];
                end
            end
        end
        
        function z = vectorizedp_laplace_noise_LDP(obj,meascnt)
            z = [];
            
            
            %dp-start-measurement-noise-start
            zm = zonotope([0],[0.01 0.02 0.01]); %dp
            rng(meascnt,'twister'); %dp
            p1 = randPoint(zm); %dp
            %dp-start-measurement-noise-end
            
            %dp-laplace-noise-start
            
            roh = 1;
            m = 1;
            epsilon = 0.3;
            delta = 0.153752652132932;%optimal range=2.5,eps=0.3
            %delta for (optimal range=7,eps=0.3)= 0.0243998192017306;
            lamda = roh/epsilon;
            a_m = (roh/epsilon)*log1p(expm1(epsilon)*((m*(1-expm1(-epsilon/m)))/(2*delta)) );
            phi_m = 1/(2*lamda*(1-expm1(-a_m/lamda)));
            %interval_Laplace = fixed.Interval(-a_m, a_m);
            x_m = -a_m:a_m/5000:a_m;
            p_Laplace = phi_m .* exp(-abs(x_m)/lamda);
            %plot(x_m,p_Laplace);
            p_Laplace = p_Laplace/sum(p_Laplace); % Make sure probabilites add up to 1.
            cp_Laplace = [0, cumsum(p_Laplace)];
            r_Laplace1 = rand;
            ind_Laplace1 = find(r_Laplace1>cp_Laplace, 1, 'last');
            resultdp_Laplace1 = x_m(ind_Laplace1);
            %dp-laplace-noise-end
            
            
%dp start             if obj.allow_d
%                 z = [obj.d_ij];
%             end
%             if obj.type >= 2
%                 if obj.allow_r
%                     z = [z; obj.r_ij];
%                 end
%dp end             end
            if obj.type >= 3
                if obj.allow_R
                    z = [z; obj.R_ij+p1+resultdp_Laplace1];
                end
            end
        end

         function z = vectorizedp_optimal_noise_CDP(obj,meascnt)
            z = [];
            
            
            %dp-start-measurement-noise-start
            zm = zonotope([0],[0.01 0.02 0.01]); %dp
            rng(meascnt,'twister'); %dp
            p1 = randPoint(zm); %dp
            %dp-start-measurement-noise-end
            
            %dp-optimal-noise-start
            opt_noise_table = readtable("logs/optimal_noise_f_CDP.csv","ReadVariableNames",true);%12-8-2022 
            opt_noise_tablex=table2array(opt_noise_table(:,1));%12-8-2022 
            opt_noise_tabley=table2array(opt_noise_table(:,2));%12-8-2022
            opt_noise_tabley = opt_noise_tabley/sum(opt_noise_tabley); % Make sure probabilites add up to 1.
            cp_opt = [0, cumsum(opt_noise_tabley.')];
            rng(meascnt,'twister'); %dp
            r1 = rand;
            ind1 = find(r1>cp_opt, 1, 'last');
            resultdp_opt1 = opt_noise_tablex(ind1);
            %r2 = rand;
            %ind2 = find(r2>cp_opt, 1, 'last');
            %resultdp_opt2 = opt_noise_tablex(ind2)
            %figure;
            %plot(opt_noise_tablex,opt_noise_tabley);%12-8-2022
            
           
            %dp-optimal-noise-end
            
            
%dp start             if obj.allow_d
%                 z = [obj.d_ij];
%             end
%             if obj.type >= 2
%                 if obj.allow_r
%                     z = [z; obj.r_ij];
%                 end
%dp end             end
            if obj.type >= 3
                if obj.allow_R
                    z = [z; obj.R_ij+p1+resultdp_opt1];
                end
            end
        end
        
        function z = vectorizedp_laplace_noise_CDP(obj,meascnt)
            z = [];
            
            
            %dp-start-measurement-noise-start
            zm = zonotope([0],[0.01 0.02 0.01]); %dp
            rng(meascnt,'twister'); %dp
            p1 = randPoint(zm); %dp
            %dp-start-measurement-noise-end
            
            %dp-laplace-noise-start
            
            roh = 1;
            m = 1;
            epsilon = 0.3;
            %delta = 0.153752652132932;%optimal range=2.5,eps=0.3
            delta = 0.5;%optimal range=1,eps=0.3

            %delta for (optimal range=7,eps=0.3)= 0.0243998192017306;
            lamda = roh/epsilon;
            a_m = (roh/epsilon)*log1p(expm1(epsilon)*((m*(1-expm1(-epsilon/m)))/(2*delta)) );
            phi_m = 1/(2*lamda*(1-expm1(-a_m/lamda)));
            %interval_Laplace = fixed.Interval(-a_m, a_m);
            x_m = -a_m:a_m/5000:a_m;
            p_Laplace = phi_m .* exp(-abs(x_m)/lamda);
            %plot(x_m,p_Laplace);
            p_Laplace = p_Laplace/sum(p_Laplace); % Make sure probabilites add up to 1.
            cp_Laplace = [0, cumsum(p_Laplace)];
            r_Laplace1 = rand;
            ind_Laplace1 = find(r_Laplace1>cp_Laplace, 1, 'last');
            resultdp_Laplace1 = x_m(ind_Laplace1);
            %dp-laplace-noise-end
            
            
%dp start             if obj.allow_d
%                 z = [obj.d_ij];
%             end
%             if obj.type >= 2
%                 if obj.allow_r
%                     z = [z; obj.r_ij];
%                 end
%dp end             end
            if obj.type >= 3
                if obj.allow_R
                    z = [z; obj.R_ij+p1+resultdp_Laplace1];
                end
            end
        end

        %dp-end
        % get the covariance matrix, R
        function R = getCovariance(obj)
            r = [];
%dp start             if obj.type == 1 || obj.type == 2 || obj.type == 3
%                 if obj.allow_d
%                     r = [r; obj.var_dij];
%                 end
%             end
%             if obj.type == 2 || obj.type == 3
%                 if obj.allow_r
%                     r = [r; obj.var_rij];
%                 end
% dp end            end
            if obj.type == 3
                if obj.allow_R
                    r = [r; obj.var_Rij];
                end
            end
            R = diag(r);
            
            % add cross-terms
%dp-start             if obj.type == 2
%                 if obj.allow_r && obj.allow_d
%                     R(1,2) = obj.var_dxr;
%                     R(2,1) = obj.var_dxr;
%                 end
%dp-end             end
%dp-start             if obj.type == 3
%                 if obj.allow_d && obj.allow_r
%                     R(1,2) = obj.var_dxr;
%                     R(2,1) = obj.var_dxr;
%                     if obj.allow_R
%                         R(1,3) = obj.var_dxR;
%                         R(2,3) = obj.var_rxR;
%                         R(3,1) = obj.var_dxR;
%                         R(3,2) = obj.var_rxR;
%                     end
%                 end
%                 if obj.allow_d && obj.allow_R
%                     R(1,2) = obj.var_dxR;
%                     R(2,1) = obj.var_dxR;
%                 end
%                 
%                 if obj.allow_r && obj.allow_R
%                     R(1,2) = obj.var_rxR;
%                     R(2,1) = obj.var_rxR;
%                 end
%dp-end             end
        end
                
        % get measurement type
        function t = getType(obj)
            t = obj.type;
        end
        
        function setType(obj, type)
            obj.type = type;
        end
        
        % get measurement time
        function t = getTime(obj)
            t = obj.walltime;
        end
        
        % get source node
        function id = getSourceId(obj)
            id = obj.nodei;
        end
        
        % get destination id
        function id = getDestId(obj)
            id = obj.nodej;
        end
        
        % measurement type filters
        function allowMeasType_d(obj)
            obj.allow_d = true;
        end
        function allowMeasType_r(obj)
            obj.allow_r = true;
        end
        function allowMeasType_R(obj)
            obj.allow_R = true;
        end
        
        % set measurement covariances
        function setCovar_dij(obj, v)
            obj.var_dij = v;
        end
        
        function setCovar_rij(obj, v)
            obj.var_rij = v;
        end
        
        function setCovar_Rij(obj, v)
            obj.var_Rij = v;
        end
        
        %dp-start  %Dawoud
        
        %get differential privacy noise zonotope %dp
         function dpzono= getdpzonotope(obj)
            dpzono = zonotope(obj.cenDP,obj.genDP);
         end
        %get measurement noise zonotope %dp
         function mzono= getmeasurementzonotope(obj)
            mzono = zonotope(obj.cenm,obj.genm);
         end
        
        %dp-end
    end
    
end

