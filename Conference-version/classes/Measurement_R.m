classdef Measurement_R < handle
    %MEASUREMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        LIGHTSPEED = 299792458; % m/s
        % range var is about 0.05 m, so due to timing that's about 1.67e-10
        var_Rij = 5;%1.5;%0.3;
        %2.5 for all
        % 3.5 for four neighb set-meme frobenius and svd
        %4.5 for four neighb set-meme frobenius and svd
        %good mobile
        cenDP= [0;0];
        genDP = 7*[1 0;0 1];
        cenm = [0;0];
        genm = [0.01;0.02];
        
    end
    
    properties
        % time
        time=0;
        % nodeIndex
        nodeIndex=0;
        % x,y measurements
        x_meas = 0;
        y_meas = 0;
        % x,y ground truth
        x_gnd = 0;
        y_gnd =0;
        x_dp = 0;%dp
        y_dp = 0;%dp
        h = [0 0];
    end
    
    methods
        
        % constructor
        function obj = Measurement_R( varargin )
            % process raw measurement
            % format:
            % time,nodeIndex,x_gnd(1),x_gnd(2),x_meas(1),x_meas(2),h(1)
            
            if nargin ==1
                raw =varargin{1};
                obj.time = raw(1);
                obj.nodeIndex = raw(2);
                % x,y ground truth
                obj.x_gnd = raw(3);
                obj.y_gnd =raw(4);
                % x,y measurements
                obj.x_meas = raw(5);
                obj.y_meas = raw(6);
                obj.x_dp = raw(7);
                obj.y_dp = raw(8);
                % h is either 0 or 1 
                %if h(1) =1 h(2) = 0
                %if h(1) =0 h(2) = 1
                obj.h(1,1) = raw(9);
                obj.h(1,2) = 1-obj.h(1,1); 
                
            end
        end
        
        
        % get the covariance matrix, R
        function R = getCovariance(obj)    
            R = obj.var_Rij;
        end
        
        
        % get measurement time
        function t = getTime(obj)
            t = obj.time;
        end
        
        function z= vectorize(obj)
            z = obj.h*[obj.x_meas ; obj.y_meas];
        end
        
        %get measurements with differential privacy noise added %dp
         function zdp= vectorizedp(obj)
            zdp = obj.h*[obj.x_dp ; obj.y_dp];
         end
         
         %get differential privacy noise zonotope %dp
         function dpzono= getdpzonotope(obj)
            dpzono = zonotope(obj.cenDP,obj.genDP);
         end
        %get measurement noise zonotope %dp
         function mzono= getmeasurementzonotope(obj)
            mzono = zonotope(obj.cenm,obj.genm);
         end
         % get node index
        function id = getNodeIdx(obj)
            id = obj.nodeIndex;
        end
        
        function loc = getTruePosition(obj)
            loc = [obj.x_gnd ; obj.y_gnd];
        end
        
        function h = getMeasFunc(obj)
           h =  obj.h;
        end
        
        function setCovar_Rij(obj, v)
            obj.var_Rij = v;
        end
        
    end
    
end

