clear all 
close all
rand('state',123)
randn('state',223)
logname = "ntbtiming_Laplace.csv";
quadcopter_timinglogs=[];
d = importdata('logs/ped01/ntbtiming.csv');
    if size(d,1) < 0
        error('NTB log file empty');
    end
    %start-dp
    for i=1:length(d)
    if d(i,2)==10||d(i,3)==10
        quadcopter_timinglogs = [quadcopter_timinglogs; d(i,:)];
    end
    end
    csvwrite('logs/quadcoptermeasurement.csv',quadcopter_timinglogs);
    %end-dp

%numOfNodes = 8;
roh = 1;
m = 1;
epsilon = 0.3;
delta = 0.5;
lamda = roh/epsilon;
a_m = (roh/epsilon)*log1p(expm1(epsilon)*((m*(1-expm1(-epsilon/m)))/(2*delta)) )
phi_m = 1/(2*lamda*(1-expm1(-a_m/lamda)))
%interval_Laplace = fixed.Interval(-a_m, a_m);
x_m = -a_m:a_m/3538:a_m;
p_Laplace = phi_m .* exp(-abs(x_m)/lamda);
plot(x_m,p_Laplace);

s1= size(x_m);

p_Laplace = p_Laplace/sum(p_Laplace); % Make sure probabilites add up to 1.
figure;
plot(x_m,p_Laplace);
cp_Laplace = [0, cumsum(p_Laplace)];
% r_Laplace = rand;
% ind_Laplace = find(r_Laplace>cp_Laplace, 1, 'last');
% resultdp_Laplace = x_m(ind_Laplace);
% r_Laplace1 = rand;
% ind_Laplace1 = find(r_Laplace1>cp_Laplace, 1, 'last');
% resultdp_Laplace1 = x_m(ind_Laplace1);
% r_Laplace2 = rand;
% ind_Laplace2 = find(r_Laplace2>cp_Laplace, 1, 'last');
% resultdp_Laplace2 = x_m(ind_Laplace2);
% figure;
zm = zonotope([0],[.01 0.02]);
% rng(1,'twister');%12-8-2022 
% p1 = randPoint(zm);
% p2 = randPoint(zm);
%x_noisy(:,1) = x(:,1) + [p1;p2];

%x_noisydp(:,1) = x_noisy(:,1)+[resultdp_Laplace1;resultdp_Laplace2];
%nodeIndex(1) = 0;
%dataLength= 1e4;
for i=1:length(quadcopter_timinglogs)
%     if mod(i,9)==0%8 
%         x(:,i) = F*x(:,i-1);
%     else
%         x(:,i) = x(:,i-1);
%     end
    rng(i,'twister');%12-8-2022 
    p1 = randPoint(zm);
%     p2 = randPoint(zm);
    x_noisy(:,i) = x(:,i) + [p1;p2];
    r_Laplace1 = rand;
    ind_Laplace1 = find(r_Laplace1>cp_Laplace, 1, 'last');
    resultdp_Laplace1 = x_m(ind_Laplace1);
%     r_Laplace2 = rand;
%     ind_Laplace2 = find(r_Laplace2>cp_Laplace, 1, 'last');
%     resultdp_Laplace2 = x_m(ind_Laplace2);
    %rand_num = randpdf(pdf_sample, sample, 1);
    x_noisydp(:,i) = x_noisy(:,i)+[resultdp_Laplace1;resultdp_Laplace2];
    % From 1 to numOfNodes
    nodeIndex(i) = mod(nodeIndex(i-1)+1,numOfNodes);
%     if mod(i,2)==0
%         h(i)=1; %[1,0]
%     else
%         h(i)=0; %[0,1]
%     end
end
figure
plot(x_noisy(1,:),x_noisy(2,:),'r*')
hold on
plot(x(1,:),x(2,:),'*')
hold on
plot(x_noisydp(1,:),x_noisydp(2,:),'g*')
legend({'x noisy','x','x noisy differential privacy'}, 'Orientation', 'vertical', 'Location', 'NE');
s2= size(x_noisy)
time=1:dataLength;
Matrix = [ time',nodeIndex'+1,x(1,:)',x(2,:)',x_noisy(1,:)',x_noisy(2,:)',x_noisydp(1,:)',x_noisydp(2,:)',h'];
dlmwrite(strcat('logs/',logname), Matrix, 'delimiter', ',', 'precision', 20);