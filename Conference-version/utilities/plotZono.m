function h=plotZono(varargin)


%------------- BEGIN CODE --------------

%If only one argument is passed
if nargin==1
    Z=varargin{1};
    dims=[1,2];
    type{1}='b';
    
%If two arguments are passed    
elseif nargin==2
    Z=varargin{1};
    dims=varargin{2};
    type{1}='b';
    
%If three or more arguments are passed
elseif nargin>=3
    Z=varargin{1};
    dims=varargin{2};   
    type(1:length(varargin)-2)=varargin(3:end);
end

% project zonotope
Z = project(Z,dims);

% delete zero generators
p = polygon(Z);

%plot and output the handle
h = line(p(1,:),zeros(size(p(1,:))),p(2,:),'Color', [0.5 0.5 0.5]);

%------------- END OF CODE --------------