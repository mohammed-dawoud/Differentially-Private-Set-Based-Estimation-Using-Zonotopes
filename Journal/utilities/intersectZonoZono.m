function [res_zonotope]=intersectZonoZono(z1,zm,zdp,hl,Rl,yl,varargin)

% Author: Mohammed Dawoud
% Written: 9-August-2022
% Last update: ---
%              
% Last revision: ---

%------------- BEGIN CODE --------------
z=z1;%dp
R2=Rl;%dp
H2=hl;%dp
y2=yl;%dp
rlength=length(Rl)
hlength=length(hl)
if nargin==6
    %The optimization function is based on norm of the generators
    method='normGen';
elseif nargin==7
    method =varargin{1};
end
    H = generators(z1);
if strcmp(method,'svd') || strcmp(method,'radius') 
    lambda0=zeros(length(z1.center),length(Rl));
    options = optimoptions(@fminunc,'Algorithm', 'quasi-newton','Display','off');
    %find the weights
    lambda = fminunc(@fun,lambda0, options);
elseif strcmp(method,'normGen')
    % Find the analytical solution
    h_combined=[];
    for i=1:length(hl)
        h_combined = [ h_combined ; hl(i)];
    end    
    gamma=eye(length(hl));
    num= H*H'*h_combined';
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
c_new=z1.center;
for i=1:length( Rl)
    c_new = c_new + lambda(:,i)* yl(i) - lambda*hl(i)*(z1.center)- lambda(:,i)*(hl(i)*(zm.center+zdp.center) );
end

%prepare generators
part1 = eye(length(z1.center));
for ii=1:length(Rl)
    part1 = part1 - lambda(:,ii)*hl(ii);
    part2(:,ii) = Rl(ii)*lambda(:,ii);
end
part1 = part1 * H;
part3 = zm.generators';
part4 = zdp.generators';
H_new = [part1 part2 part3 part4];
res_zonotope = zonotope([c_new H_new]);



    function nfro = fun(lambda)
        part1 = eye(length(z1.center));
        for ii=1:length(Rl)
            part1 = part1 - lambda(:,ii)*hl{ii};
            part2(:,ii) = Rl{ii}*lambda(:,ii);
        end
        part1 = part1 * H;
        part3 = zm{1,1}.generators;
        part4 = zdp{1,1}.generators;
        H_new = [part1 part2 part3 part4];
        if strcmp(method,'svd')
            nfro = sum(svd(H_new));
        elseif strcmp(method,'radius')
            nfro = radius(zonotope([zeros(length(z1.center),1) H_new]));
        end
        
    end

rlength=length(Rl)
hlength=length(hl)
end
