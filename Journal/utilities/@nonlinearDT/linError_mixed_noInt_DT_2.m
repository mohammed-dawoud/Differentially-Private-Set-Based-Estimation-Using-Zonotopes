function errorZon = linError_mixed_noInt_DT_2(obj, options, R)
% linError_mixed_noInt - computes the linearization error
%
% Syntax:  
%    error = linError_mixed_noInt(obj,options,R)
%
% Inputs:
%    obj - nonlinearSysDT system object
%    options - options struct
%    R - initial set
%
% Outputs:
%    errorZon - zonotope overapproximating the linearization error
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: 

% Author:       Matthias Althoff, Niklas Kochdumper
% Written:      21-August-2012
% Last update:  25-July-2016 (intervalhull replaced by interval)
%               29-January-2018 (NK)
% Last revision:---

%------------- BEGIN CODE --------------

%obtain intervals and combined interval z
dx = interval(R);
du = interval(options.U);
dz = [dx; du];

%compute interval of reachable set
totalInt_x = dx + options.p.x;

%compute intervals of input
totalInt_u = du + options.p.u;

%compute zonotope of state and input
Rred = reduce(R,'girard',options.errorOrder);
Z=cartProd(Rred,options.U);

%obtain hessian tensor
if isfield(options,'lagrangeRem') && isfield(options.lagrangeRem,'method') && ...
   ~strcmp(options.lagrangeRem.method,'interval')

    % create taylor models or zoo-objects
    [objX,objU] = initRangeBoundingObjects(totalInt_x,totalInt_u,options);

    % evaluate the hessian tensor 
    H = obj.hessian(objX,objU);
else
    H = obj.hessian(totalInt_x, totalInt_u);
end


% %%amr remove the u dependancy 
if length(H{2})==0
    H{2} = interval(zeros(length(H{1})));
end
% 
% for i=1:obj.dim
%    H{i} = H{i}(1:2,1:2);   
% end

%%

%obtain absolute values
dz_abs = max(abs(infimum(dz)), abs(supremum(dz)));

%separate evaluation
H_mid = cell(options.dim_h,1);
H_rad = cell(options.dim_h,1);
for i=1:options.dim_h
    H_mid{i} = sparse(center(H{i}));
    H_rad{i} = sparse(rad(H{i}));
end

error_mid = 0.5*quadMap(Z, H_mid);

%interval evaluation
error_rad = zeros(options.dim_h,1);
for i=1:options.dim_h
    error_rad(i,1) = 0.5*dz_abs'*H_rad{i}*dz_abs;
end

%combine results
error_rad_zono = zonotope(interval(-error_rad, error_rad));
errorZon = error_mid + error_rad_zono;

errorZon = reduce(errorZon,options.reductionTechnique,options.zonotopeOrder);

%------------- END OF CODE --------------