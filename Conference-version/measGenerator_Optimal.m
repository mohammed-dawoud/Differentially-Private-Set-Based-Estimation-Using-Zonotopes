clear all 
close all
rand('state',123)
randn('state',223)
logname = "new_rotatingTarget_Optimal.csv";

numOfNodes = 8;
% epsilon = 0.3;
% delta = 0.0243998192017306;

F= [0.992 -0.1247; 0.1247 0.992];
x(:,1) = [50;50];
h(1)=0; %[0,1]
opt_noise_table = readtable('logs/optimal_noise_f.csv','ReadVariableNames',true);%12-8-2022 
opt_noise_tablex=table2array(opt_noise_table(:,1));%12-8-2022 
opt_noise_tabley=table2array(opt_noise_table(:,2));%12-8-2022
opt_noise_tabley = opt_noise_tabley/sum(opt_noise_tabley); % Make sure probabilites add up to 1.
cp_opt = [0, cumsum(opt_noise_tabley.')];
r1 = rand;
ind1 = find(r1>cp_opt, 1, 'last');
resultdp_opt1 = opt_noise_tablex(ind1)
r2 = rand;
ind2 = find(r2>cp_opt, 1, 'last');
resultdp_opt2 = opt_noise_tablex(ind2)
figure;
plot(opt_noise_tablex,opt_noise_tabley);%12-8-2022
zm = zonotope([0],[.01 0.02]);
rng(1,'twister');%12-8-2022 
p1 = randPoint(zm);
p2 = randPoint(zm);
x_noisy(:,1) = x(:,1) + [p1;p2];
x_noisydp(:,1) = x_noisy(:,1)+[resultdp_opt1;resultdp_opt2];

nodeIndex(1) = 0;
dataLength= 1e4;
for i=2:dataLength
    if mod(i,8)==0%8 
        x(:,i) = F*x(:,i-1);
    else
        x(:,i) = x(:,i-1);
    end
    rng(i,'twister');%12-8-2022 
    p = randPoint(zm);
   
    x_noisy(:,i) = x(:,i) + p;
   
%     r = rand;
%     ind = find(r>cp_opt, 1, 'last');
%     resultdp_opt = opt_noise_tablex(ind);
    r1 = rand;
ind1 = find(r1>cp_opt, 1, 'last');
resultdp_opt1 = opt_noise_tablex(ind1)
r2 = rand;
ind2 = find(r2>cp_opt, 1, 'last');
resultdp_opt2 = opt_noise_tablex(ind2)
    x_noisydp(:,i) = x_noisy(:,i)+[resultdp_opt1;resultdp_opt2];
    % From 1 to numOfNodes
    nodeIndex(i) = mod(nodeIndex(i-1)+1,numOfNodes);
    if mod(i,2)==0
        h(i)=1; %[1,0]
    else
        h(i)=0; %[0,1]
    end
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