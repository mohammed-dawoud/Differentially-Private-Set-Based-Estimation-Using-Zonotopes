function [res_zonotope]=intersectZonoZonoTest(fstate,x,P,hmeas,z,Q,R,szono,method,zm,zdp,varargin)

% Author: Mohammed Dawoud
% Written: 1-November-2023
% Last update: 4-December-2023
%              
% Last revision: ---

%------------- BEGIN CODE --------------
Rl=[R;R;R];%dp
%dp H2=hl;%dp
y2=z;%dp
rlength=length(Rl);
%dp hlength=length(hl)
%dp-start if nargin==6
%     %The optimization function is based on norm of the generators
%     method='normGen';
% elseif nargin==7
%     method =varargin{1};
%dp-end end

%dp-start
%szono2=zonotope(x,P);
           [x1,A]=jaccsd(fstate,x);    %nonlinear update and linearization at current state
            %P=A*P*A'+Q;     
           [z1,h]=jaccsd(hmeas,x1);
           %P12=P*H';                   %cross covariance
           hl=h';
           H = generators(szono);
           %H = H2(:,1:3);
           %H= H2;

%dp-end
    if strcmp(method,'svd') || strcmp(method,'radius') 
    lambda0=zeros(length(szono.center),length(Rl));
    options = optimoptions(@fminunc,'Algorithm', 'quasi-newton','Display','off');
    %find the weights
    lambda = fminunc(@fun,lambda0, options);
elseif strcmp(method,'normGen')
    % Find the analytical solution
    h_combined=zeros(3,3);
    for i=1:length(hl)
        h_combined(i,i) = [ hl(i)];
    end    
    gamma=eye(length(hl));
    num= H*H'*h_combined;
    den = h_combined * H*H' * h_combined' ;
    for i=1:length(hl)
        den = den + gamma(:,i) *Rl(i)^2* gamma(:,i)';
    end
    
    lambda = num * den^-1;
else
    disp('Method is not supported');
    return;
    end





%prepare center
c_new=x1;
for i=1:length(Rl)
    c_new = c_new + lambda(:,i)*(z-z1) - lambda(:,i)*((zm.center+zdp.center) );
end

%prepare generators
part1 = eye(length(szono.center));
%part1 = H;
for ii=1:length(Rl)
    part1 = part1 - lambda(:,ii)*hl(ii);
    part2(:,ii) = Rl(ii)*lambda(:,ii);
end
%part1 = part1 * H;
%part3= [0.01 0 0; 0 0.02 0; 0 0 0.01];
%part4= [.1 0 0; 0 .1 0; 0 0 .1];
part3 = zm.generators';
part4 = zdp.generators';
H_new = [part1 part2 part3 part4];
res_zonotope = zonotope([c_new H_new]);



    function nfro = fun(lambda)
        part1 = eye(length(szono.center));
        for ii=1:length(Rl)
            part1 = part1 - lambda(:,ii)*hl(ii);
            part2(:,ii) = Rl(ii)*lambda(:,ii);
        end
        part1 = part1 * H;
        part3 = zm.generators';
        part4 = zdp.generators';
        H_new = [part1 part2 part3 part4];
        if strcmp(method,'svd')
            nfro = sum(svd(H_new));
        elseif strcmp(method,'radius')
            nfro = radius(zonotope([zeros(length(szono.center),1) H_new]));
        end
        
    end

rlength=length(Rl)
hlength=length(hl)
end


function [z,A]=jaccsd(fun,x)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,x)
% z = f(x)
% J = f'(x)
%
z=fun(x);
n=numel(x);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=x;
    x1(k)=x1(k)+h*1i;
    A(:,k)=imag(fun(x1))/h;
end
end

