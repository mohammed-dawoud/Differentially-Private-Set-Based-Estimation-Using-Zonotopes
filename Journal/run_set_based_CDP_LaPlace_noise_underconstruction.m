
%%
% This is the main file to run Event-triggered Diffusion Kalman Filter
%%

%loop over the following values of threshold in the paper
% each threshold value (run) will generate mat file with the results under
% the cache folder

clc; close all ;
keepvars = {'ii'};
clearvars('-except', keepvars{:});

addpath('classes');
addpath('utilities');

%% Supress warnings
warning('off', 'MATLAB:nearlySingularMatrix');

%% Raw Data Log Folder
logname='ped01';
%other logs are ped02,ped03,ped04
logfolder = strcat('logs/',logname,'/');

target_location_history=[];
%% Node/Network configuration
configfile = 'config/nodepositions_nesl_mobile_dp';

%% Create Network Manager
% NetworkManager(nodeconfig, datafolder, <owr_corrections>, <twr_corrections>) 
nm = NetworkManager(configfile, logfolder, 'config/antennacorrections_mocap', 'config/twrcorrections_mocap', 'config/twrcorrections_mocap' );
node_ids = nm.getNodeIds();
node_names = nm.getNodeNames();

%% Rigid body IDs
nm.setRigidBodyId('ntb-mobile', 1);

%% Black-listed nodes
%nm.blacklistNode('ntb-charlie');

%% Select reference node
nm.setReferenceNode('Alpha');

%% Bootstrap the node positions and clock params before filter starts
% use first second of measurements to bootstrap node states
nm.skipTime(46);
%ped01 --> skip 46
%ped02 --> skip 15
%ped03 --> skip 15
%ped04 --> skip 15
%nm.setStaticNodesToTruePosition();
nm.bootstrapNodeStates( 4.0 );
%

%dp-start
xlength = 3;
debugEnable=0;
logfilename='log.txt';
%dp-end

%% Measurement type selection
nm.enableMessageType(nm.MSGTYPE3, true);
nm.enableMessageType(nm.MSGTYPE1, true);
nm.enableMessageType(nm.MSGTYPE2, true);
nm.enableMeasurementType(nm.MEASTYPE_d, true);
nm.enableMeasurementType(nm.MEASTYPE_r, true);
nm.enableMeasurementType(nm.MEASTYPE_R, true);
%nm.setMessageMinInterval(nm.MSGTYPE3, 2.00);
nm.enableSLATS(true);


%% Set some known anchor positions if desired
known_anchors = {'Alpha', 'Bravo', 'Charlie', 'Delta', 'Hotel', 'Golf', 'Echo', 'Fox'};
%known_anchors = {};
for i=1:length(known_anchors)
    aid = known_anchors{i};
    aidx = nm.getNodeIdx( aid );
    %dp nm.nodes{ aidx }.var_p = [0.0; 0.0; 0.0];
    %dp nm.nodes{ aidx }.var_p = nm.nodes{ aidx }.true_p;
    %dp-start
    nm.nodes{aidx}.initReach(xlength,debugEnable,logfilename);

    %dp-end
end

%% Process Covariances
% Process and Initial noise and state vals are determined by each node object
Q1 = nm.getProcessVar();
P1 = nm.getInitialVar();
P=P1(25:27,25:27);
Q=Q1(25:27,25:27);
nm.nodes{9}.x_zonotope = zonotope([0;0;0],P);
%dp-start
algorithm = 'set-membership';
method = 'normGen';
diffEnable =0;
sNodp = nm.getStatedp();
szonoNodp=nm.nodes{9}.x_zonotope;
sNodp_history = [];
measurement_history=[];
raw_measurements=[];
noisy_measurements=[];
sensor_manager=[];
%dp-end
%% Save as movie
SAVEMOVIE = false;
if SAVEMOVIE
    vidObj = VideoWriter('video/mocap.avi');
    vidObj.FrameRate=20;
    open(vidObj);
end

%% Position Visualization
% get current true and estimated positions
pTruStatic = nm.getTrueStaticPositions();
pEstAll = nm.getTransformedPositions();

fig = cfigure(25,25); grid on; hold on; axis equal;

htruStatic = plot3(pTruStatic(:,2),pTruStatic(:,3),pTruStatic(:,4),'^k','MarkerSize',10, 'MarkerFaceColor', 'k');
hestAll = plot3(pEstAll(:,1),pEstAll(:,2),pEstAll(:,3),'r+');
herrStatic = zeros(nm.getNumStaticNodes(),1);
hvar = zeros(nm.getNumNodes(),1);
%dp-start
h_reach = zeros(1,1);
%dp-end
varscale = 1.00;
for i=1:nm.getNumNodes()
    nid = node_ids(i);
    % if node is static, plot error line to true position
    if ~nm.nodes{i}.isMobile()
        herrStatic(i) = plot3([pTruStatic(i,2) pEstAll(i,1)],[pTruStatic(i,3) pEstAll(i,2)],[pTruStatic(i,4) pEstAll(i,3)], nm.getPlotStyle(nid), 'Color', nm.getPlotColor(nid));
    end
    % if node is the reference, add text saying so
    if nm.nodes{i}.isReference()
        xyz = nm.nodes{i}.getTruePosition();
        text( xyz(1)-0.50, xyz(2) - 0.30, xyz(3), 'Reference');
    end
    % add text for each node
    if ~nm.nodes{i}.isMobile()
        xyz = nm.nodes{i}.getTruePosition();
        text( xyz(1)+0.45, xyz(2) + 0.50, xyz(3)-0.45, nm.nodes{i}.getName() );
    end
    
    % get a node's position covariance matrix    
    nidx = (i-1)*3 + 1;
     Pi = P1( nidx:(nidx+2), nidx:(nidx+2) ) + 0.01*[1 0 0; 0 0 0; 0 0 1];
%         Pi = P( 1, 1 ) + 0.01*[1 0 0; 0 0 0; 0 0 1];
%dp     if i<9
%dp     hvar(i) = plotEllipse([pEstAll(i,1); pEstAll(i,2); pEstAll(i,3)], Pi, varscale);
%dp     end
    %dp-start
    if nm.nodes{i}.isMobile()
    h_reach(i) = plotZono(nm.nodes{i}.x_zonotope,[1,3],'r');
    end
    %dp-end
end

% rigid bodies in mocap
rigid_bodies = nm.dataparser.getRigidBodyIds();
hrigidbodies = zeros( length(rigid_bodies),1 );
for i=1:length(rigid_bodies)
    hrigidbodies(i) = plot3(0, 0, 0, 'sb', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 2);
end

xlim([-7 7]);
ylim([0 4]);
zlim([-7 7]);
xlabel('X Position (m)', 'FontSize',14);
ylabel('Y Position (m)', 'FontSize',14);
zlabel('Z Position (m)', 'FontSize',14);
htitle = title('NESL Network Localization (t = 0.00s)','FontSize',16);
view(180,0);
%legend(herr, nm.getNodeNames());
drawnow;

%% Replay data and run EKF
% analysis stop time
%t_stop = 200;
t_stop = 10;
% state, time, and transformed position history
s_history = [];
p_history = {};
cov_history = [];
pDKAL_history =[];
P_big_history = [];
t_history = [];
timeUpdateFlag_history = [];
DiffFlag_history = [];
MeasFlag_history = [];

%start-dp
meas11 = nm.getNextMeasurement();
raw_measurements=[meas11.getTime() meas11.nodei meas11.nodej meas11.R_ij meas11.getCovariance()];

while(nm.msgidx < nm.dataparser.getNumMeasurements())
    measq = nm.getNextMeasurement();
    raw_measurements=[raw_measurements; measq.getTime() measq.nodei measq.nodej measq.R_ij measq.getCovariance()];

end
 measzonotope = zonotope([0],[0.01 0.02 0.01]); %dp
 dpzonotope= zonotope([0],0.1*[1 1 1]);
 roh = 1;
    m = 1;
    epsilon = 0.3;
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
for i=1:length(raw_measurements)
    %dp-start-measurement-noise-start
           
            rng(i,'twister'); %dp
            p1 = randPoint(measzonotope); %dp
            %dp-start-measurement-noise-end
            
            %dp-laplace-noise-start
            r_Laplace1 = rand;
            ind_Laplace1 = find(r_Laplace1>cp_Laplace, 1, 'last');
            resultdp_Laplace1 = x_m(ind_Laplace1);
            %dp-laplace-noise-end
            temp = raw_measurements(i,4)+p1+resultdp_Laplace1;
            noisy_measurements = [noisy_measurements; raw_measurements(i,1) raw_measurements(i,2) raw_measurements(i,3) temp raw_measurements(i,5)];
end
diffEnable = 1; % enable diffusion
nm.resetMeasurements();
%end-dp

% last global time update
%start-dp-measurement
%meas1 = nm.getNextMeasurement();

t_start = noisy_measurements(1,1);
t_last = t_start;
k = 1;
%end-dp-measurement
% plotting
plot_delay = 0*0.100; % sec
plot_last = t_start;

tlast_twr = 0;
period_twr = 5.00;


MOBILEID = 10;

if strcmp(logname,'ped03') || strcmp(logname,'ped04')
    MOBILEID = 8;
end


%dp-measurement if meas1.nodei==10||meas1.nodej==10
%dp-measurement measurement_history=[measurement_history;meas1];
%dp-measurement end

while (t_last - t_start) < t_stop
    k = k + 1;
    
    % get next measurement object
    %dp -measurements meas = nm.getNextMeasurement();
    %dp track neighborhod
%dp     if meas.nodei==10||meas.nodej==10
%dp     measurement_history=[measurement_history;meas];
%dp     end
    if isempty(raw_measurements(k))
        break;
    end
    walltime = raw_measurements(k,1);
    z = raw_measurements(k,4);
    R = raw_measurements(k,5);  
    

    % delta-time uses wallclock (desktop timestamp) for now
    dt_ref = raw_measurements(k,1) - t_last;
    t_last = walltime;
    t_history = [t_history; walltime];
    
    % get network state vector
    s = nm.getStatedp();
   
    % configure process and measurement functions
    f = @(s) nm.processFcndp(s, dt_ref);
    h = @(s) nm.measurementFcnCentralized(s, raw_measurements(k,:));
     
    % update state estimate
    s = nm.getStatedp();
% to use centralized EKF
    %dp-start
    [zset,pi,pj] = nm.measurementFcnsetCentralized(s, raw_measurements(k,:));
    zdp = noisy_measurements(k,4);%dp-measurement meas.vectorizedp_laplace_noise(nm.meascnt);
    %dp hdp = @(s) nm.measurementFcndp_laplace_noise(s, meas);
    %dp-start 
    
    %dp-end
    %dp-end
      %dp-measurement nm.setMeasParameters(meas,pi,pj,zm,zdp);%dp
if raw_measurements(k,2)==10 || raw_measurements(k,3)==10
     %[s, P] = lkf(f, s, P, h, z, dt_ref*Q, R,nm,meas);
      %[s, P] = ekf(f, s, P, h, z, dt_ref*Q, R);
      %dp-start
      %[s,P] = NonlinearEstimatorTest(f,s,P,h,z,dt_ref*Q,R,nm.nodes{9}.x_zonotope,meas,method);
      szono = intersectZonoZonoTest(f,s,P,h,zdp,dt_ref*Q,R,nm.nodes{9}.x_zonotope,method,measzonotope,dpzonotope);
      szonoNodp = intersectZonoZonoTest(f,sNodp,P,h,z,dt_ref*Q,R,szonoNodp,method,measzonotope,dpzonotope);
      s = center(szono);
      sNodp = center(szonoNodp);
      P = generators(szono);
      %P = P1(:,1:3);
      nm.nodes{9}.x_zonotope=szono;
      % nm.check_p1(diffEnable,f,algorithm,method,Q);
     %dp-end
end
    nm.setStatedp(s);

    % update position estimates
    pTruStatic = nm.getTrueStaticPositions();
    pEstAll = nm.getTransformedPositions();
    

    % update plots
    set(hestAll,'xdata',pEstAll(:,1),'ydata', pEstAll(:,2),'zdata',pEstAll(:,3));
    for i=1:nm.getNumNodes()
        if ~nm.nodes{i}.isMobile()
            set(herrStatic(i),'xdata',[pTruStatic(i,2) pEstAll(i,1)],'ydata',...
                [pTruStatic(i,3) pEstAll(i,2)],'zdata',[pTruStatic(i,4) pEstAll(i,3)]);
        end
        nidx = (i-1)*3 + 1;
%dp        if i~=9
%dp         updateEllipse( hvar(i), [pEstAll(i,1); pEstAll(i,2); pEstAll(i,3)], Pi + 0.001*[1 0 0; 0 0 0; 0 0 1], varscale);
%dp        end
       %dp-start    
       if nm.nodes{i}.isMobile()
       updatePlotZono(h_reach(i),nm.nodes{i}.x_zonotope,[1,3],'r');
       end
       %dp-end
       if i == 3
            % Pi
        end
    end    
    
    if walltime - plot_last >= plot_delay
        plot_last = walltime;
        % update rigid bodies
        for i=1:length(rigid_bodies)
            rb = rigid_bodies(i);
            [xyz, latency] = nm.dataparser.getMocapPos(rb, walltime);
                  target_location_history=[target_location_history; xyz];

            if latency < 0.250
                set(hrigidbodies(i), 'XData', xyz(1), 'YData', xyz(2), 'ZData', xyz(3));%plot square target according to mocap measurements
            end
        end
        
        % update plot title
        tstr = sprintf('Quadcopter Localization (t = %.2fs)', (t_last - t_start));
        set(htitle, 'String', tstr);
        drawnow;
    end

    % append state estimate & measurement to history
    s_history = [s_history; s'];
    sNodp_history = [sNodp_history; sNodp'];
    p_history = [p_history; pEstAll];
    %start dp
    reference_point=mean(target_location_history);
    % end dp 
    %fprintf('t = %.2f / %.2f \n', meas.getTime()-meas1.getTime(), t_stop);
    
    if SAVEMOVIE
        f = getframe(fig);
        writeVideo(vidObj,f);
    end

    %pause();
end
%start-dp
figure; hold on
x_axis= 0:1:1443;
error=target_location_history - s_history;
errorNodp=target_location_history - sNodp_history;
normerror=zeros(length(error),1);
normerrorNodp=zeros(length(errorNodp),1);
for i=1:length(error)
    normerror(i)=norm(error(i,:));
    normerrorNodp(i)=norm(errorNodp(i,:));
end
T=table(error,normerror);
filename = 'logs/CDP_Estimation_error_laplace_noise--.xlsx';
writetable(T,filename,'Sheet',1,'Range','A1:D1444')
plot(x_axis',normerror);
hold on
plot(x_axis',normerrorNodp);
%dp p_history{1,1};
xlabel('Estimation Iterations - K', 'FontSize',10);
ylabel({'Error in Estimated Location (m)'; '||X_{error},Y_{error},Z_{error}||_{2}'}, 'FontSize',10);
htitle = title('Error in Estimated Location','FontSize',16);
legend({'Error in presence of the differential privacy noise (Laplace noise)','Error without the differential privacy noise'}, 'Orientation', 'vertical', 'Location', 'NE');
% reference_p=mean(p_history{:,1});
%end dp
if SAVEMOVIE
    close(vidObj);
end
savemsgsM=nm.savemsgsM;
sentmsgsD=nm.sentmsgsD;
sentmsgsM=nm.sentmsgsM;
% save data


saveName ='cache/temp';
% save(saveName,'savemsgsM','sentmsgsD','sentmsgsM', 'nm', 'k', 's_history', 'p_history', 'cov_history','pDKAL_history','P_big_history' ,'t_history','PDESIRED','t_stop','timeUpdateFlag_history' ,'DiffFlag_history','MeasFlag_history');
save(saveName,'savemsgsM','sentmsgsD','sentmsgsM', 'nm', 'k', 's_history', 'p_history', 'cov_history','pDKAL_history','P_big_history' ,'t_history','t_stop','timeUpdateFlag_history' ,'DiffFlag_history','MeasFlag_history');




