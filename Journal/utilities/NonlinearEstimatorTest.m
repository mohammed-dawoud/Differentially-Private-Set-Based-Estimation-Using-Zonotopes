% function [outputArg1,outputArg2] = NonlinearEstimatorTest(inputArg1,inputArg2)
% %NONLINEARESTIMATORTEST Summary of this function goes here
% %   Detailed explanation goes here
% outputArg1 = inputArg1;
% outputArg2 = inputArg2;
% end

function [x,P] =  NonlinearEstimatorTest(fstate,x,P,hmeas,z,Q,R,szono,meas,method)
            % has no effect if the h does not have u
            [x1,A]=jaccsd(fstate,x);    %nonlinear update and linearization at current state
             P=A*P*A'+Q; 
             
             
            [z1,H]=jaccsd(hmeas,x1);    %nonlinear measurement and linearization
             
            P12=P*H';                   %cross covariance
             K=P12*inv(H*P12+R);       %Kalman filter gain
             x=x1+K*(z-z1);            %state estimate
             P=P-K*P12';
            


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
