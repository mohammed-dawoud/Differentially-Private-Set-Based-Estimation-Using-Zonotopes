function [Rout] = intersectNonLinearH(hDisc,Rinit,options)
%INTERSECTNONLINEARH Summary of this function goes here

% compute symbolic derivatives
derivatives(hDisc,options);
%linearization point p.u of the input is the center of the input u
p.u = center(options.U) ;%+ options.uTrans;

%linearization point p.x and p.y
x0 = center(Rinit);
p.x = x0;

%substitute p into the system equation in order to obtain the constant
%input
 f0 = options.fun(p.x, p.u);
% 
% options.jacobians = jacobian(options.fun,[x,u]);
% %get jacobian matrices
[A_lin,B_lin] = hDisc.jacobian( p.u,p.x);
% [A_lin] = options.jacobians(p.x, p.u);



% uTrans = f0; %B*Ucenter from linOptions.U not added as the system is linearized around center(U)
% Udelta = B_lin*(options.U+(-center(options.U)));
% U = Udelta + uTrans;

%save linearization point
%hDisc.linError.p=p;

%translate Rinit by linearization point
Rdelta = Rinit + (-p.x);
options.p = p;

% if options.tensorOrder > 2
%     Verror = linError_thirdOrder_DT(hDisc, options, Rdelta); 
% else
%     Verror = linError_mixed_noInt_DT(hDisc, options, Rdelta);   
% end
%Verror = linearizeOpt(hDisc, options, Rdelta);
Verror = linError_mixed_noInt_DT(hDisc, options, Rdelta);

H = generators(Rinit);

% initialize lambda
lambda0=zeros(options.dim_x,options.dim_h);
optionsfmin = optimoptions(@fminunc,'Algorithm', 'quasi-newton','Display','off');
%find optimal lambda
 %lambda = fminunc(@fun,lambda0, optionsfmin);


%dp-start
    h_combined=[];
    for i=1:length(options.hl)
        h_combined(i,i) = [options.hl(i)];
    end    
    gamma=eye(length(options.hl));
    num= H*H'*h_combined;
    den = h_combined * H*H' * h_combined' ;
    for i=1:length(options.hl)
        den = den + gamma(:,i) *options.Rl(i)^2* gamma(:,i)';
    end
    
    lambda1 = num * den^-1;
    lambda=lambda1(:,1);
    %dp-end

part1 = (eye(options.dim_x)-lambda*A_lin)*Rinit.Z(:,2:end);
%dp part1 = part1 * options.zm;

% resulting zonotope
%Zres = zonotopeFromLambda(Z,R,h,y,lambda);
newCen = Rinit.Z(:,1) - lambda*f0 - lambda*A_lin*Rinit.Z(:,1) + lambda*A_lin*p.x - lambda1*Verror.Z(:,1);
newGen = [part1, lambda1*options.var, - lambda1*Verror.Z(:,2:end)];
%newCen = Rinit.Z(:,1) - lambda*f0 - lambda*A_lin*Rinit.Z(:,1) + lambda*A_lin*p.x ;
%newGen = [(eye(options.dim_x)-lambda*A_lin)*Rinit.Z(:,2:end),- lambda*a];

%Rout = zonotope(newCen,newGen)  +lambda*Verror ;

Rout = zonotope(newCen,newGen);



% embedded function to be minimized for optimal lambda
function nfro = fun(lambda)
%dp-start
%  part1 = eye(length(z1.center));
%         for ii=1:length(Rl)
%             part1 = part1 - lambda(:,ii)*hl{ii};
%             part2(:,ii) = Rl{ii}*lambda(:,ii);
%         end
part1 = (eye(options.dim_x)-lambda*A_lin)*Rinit.Z(:,2:end);
part1 = part1 * options.zm;
%dp-end
    
    
    newGen = [part1, -lambda*options.meas.getCovariance(), -lambda*Verror.Z(:,2:end) ];
%newGen = [(eye(options.dim_x)-lambda*A_lin)*Rinit.Z(:,2:end), -lambda*a];


if strcmp(options.method,'normGen')
    nfro = norm(newGen,'fro');
elseif strcmp(options.method,'svd')
    nfro = sum(svd(newGen));
elseif strcmp(options.method,'radius')
    nfro = radius(zonotope([zeros(options.dim_x,1) newGen]));  
elseif strcmp(options.method,'volume')
    nfro = volume(zonotope([zeros(options.dim_x,1) newGen]));
end

end
end