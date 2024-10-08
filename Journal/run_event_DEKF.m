
%%
% This is the main file to run Event-triggered Diffusion Kalman Filter
%%

%loop over the following values of threshold in the paper
% each threshold value (run) will generate mat file with the results under
% the cache folder
for ii=2.5
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


%% Node/Network configuration
configfile = 'config/nodepositions_nesl_mobile';

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
nm.setReferenceNode('ntb-alpha');

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
%known_anchors = {'ntb-alpha', 'ntb-bravo', 'ntb-charlie', 'ntb-delta', 'ntb-hotel', 'ntb-golf', 'ntb-echo', 'ntb-foxtrot'};
known_anchors = {};
for i=1:length(known_anchors)
    aid = known_anchors{i};
    aidx = nm.getNodeIdx( aid );
    nm.nodes{ aidx }.var_p = [0.0; 0.0; 0.0];
end

%% Process Covariances
% Process and Initial noise and state vals are determined by each node object
Q = nm.getProcessVar();
P = nm.getInitialVar();

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
        text( xyz(1)-0.50, xyz(2) + 0.50, xyz(3), nm.nodes{i}.getName() );
    end
    
    % get a node's position covariance matrix    
    nidx = (i-1)*5 + 1;
    Pi = P( nidx:(nidx+2), nidx:(nidx+2) ) + 0.01*[1 0 0; 0 0 0; 0 0 1];
    hvar(i) = plotEllipse([pEstAll(i,1); pEstAll(i,2); pEstAll(i,3)], Pi, varscale);
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

%Choose network connectivity, first cell sets which nodes are connected to
%node number 1 and so on
nm.network= { [1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8  9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9]};
%                    1           2               3           4               5       6               7           8           9
%nm.network = {[1 2 5 7 8 9],[2 3 6 7 1 9],[3 4 6 8 2 9],[4 5 6 8 3 9],[5 6 7 1 4],[6 2 3 4 5],[7 8 1 2 5],[8 1 3 4 7],[1 2 3 4]};
%  3nieg         1      2        3      4       5         6        7       8        9
%nm.network = {[1 5 9],[2 7 9],[3 8 9],[4 5 6],[5 6 1 4],[6 4 5],[7 8 2],[8 3 7],[1 2 3  9]};

%  4nieg         1          2        3          4       5          6           7       8        9
%nm.network = {[1 2 5 9],[1 2 7 9],[3 4 8 9],[4 3 5 6],[5 6 1 4],[6 4 5 7],[6 7 8 2],[8 3 7 9],[1 2 3 8 9]};

% last global time update
meas1 = nm.getNextMeasurement();
t_start = meas1.getTime();
t_last = t_start;
k = 0;

nm.setneigh_forall();
nm.init_x_P_forall(nm.getState(),P);

% plotting
plot_delay = 0*0.100; % sec
plot_last = t_start;

tlast_twr = 0;
period_twr = 5.00;

PDESIRED =ii;
MOBILEID = 10;

if strcmp(logname,'ped03') || strcmp(logname,'ped04')
    MOBILEID = 8;
end

avg_pDKAL = PDESIRED +1;
diffEnable = 1; % enable diffusion
while (t_last - t_start) < t_stop
    k = k + 1;
    
    % get next measurement object
    meas = nm.getNextMeasurement();
    if isempty(meas)
        break;
    end
    walltime = meas.getTime();
    z = meas.vectorize();
    R = meas.getCovariance();  
    

       
    % delta-time uses wallclock (desktop timestamp) for now
    dt_ref = meas.getTime() - t_last;
    t_last = walltime;
    t_history = [t_history; walltime];
    
    % get network state vector
    %s = nm.getState();
    s= nm.getAndFixStateConc();
    % configure process and measurement functions
    f = @(s) nm.processFcn(s, dt_ref);
    h = @(s) nm.measurementFcn(s, meas);
    
    % update state estimate
%     s = nm.getState();
% to used centralized EKF
%     [s, P] = ekf(f, s, P, h, z, dt_ref*Q, R);


 
 %every node participates with its estimate for position ploting
     nm.setState(nm.getAndFixStateConc());
    

    % update position estimates
    pTruStatic = nm.getTrueStaticPositions();
    pEstAll = nm.getTransformedPositions();
    


  if avg_pDKAL > PDESIRED
        bestNode = -1;
        bestP = 0;
        % pubish y to neighbors
        nm.publishmeas_forneigh(meas,h);
        % if a node got the y's from all its neighbors, then run
        % measurement update step
        RunMeasFlag = nm.checkekf_p1_forallUnconn( );
        % publish eita (epsi) to prepare for the diffusion step
        nm.publisheita_forneigh();
        %[RunDiffFlag,RunTimeFlag]= nm.checkekf_p2_forall(f,dt_ref*Q ,diffEnable,dt_ref);
        %check if the node is ready for diffusion step
        [RunDiffFlag,RunTimeFlag]= nm.checkekf_p2_forallUnconn(f,dt_ref*Q ,diffEnable);

        timeUpdateFlag_history = [timeUpdateFlag_history RunTimeFlag];
        DiffFlag_history = [DiffFlag_history RunDiffFlag];
        MeasFlag_history = [MeasFlag_history RunMeasFlag];
        

        
        %%----------- get p from DKAL
        mobIdx = nm.getNodeIdx( MOBILEID );
        stateIdx = (mobIdx-1)*5 + 1;
        Pi_DKAL = nm.nodes{mobIdx}.P(stateIdx:(stateIdx+2), stateIdx:(stateIdx+2));
        avg_pDKAL = trace(Pi_DKAL)/3
        pDKAL_history = [ pDKAL_history; [walltime avg_pDKAL]];

  else
        srcIndex=meas.getSourceId() +1;
        if srcIndex ==11
            srcIndex =9;
        end
        % the src will send to all his nighbours except him self
        nm.savemsgsM = nm.savemsgsM +length(nm.network{srcIndex}) -1;
      
        %RunTimeFlag=nm.ekfPartThreeOnlyForAll(dt_ref*Q ,f,dt_ref);
        RunTimeFlag=nm.ekfPartThreeOnlyForAllUncon(dt_ref*Q ,f);
        %RunTimeFlag should be always 1 here
        timeUpdateFlag_history = [timeUpdateFlag_history RunTimeFlag];
        DiffFlag_history = [DiffFlag_history 0];
        MeasFlag_history = [MeasFlag_history 0];

        %%----------- get p from DKAL
        mobIdx = nm.getNodeIdx( MOBILEID );
        stateIdx = (mobIdx-1)*5 + 1;
        Pi_DKAL = nm.nodes{mobIdx}.P(stateIdx:(stateIdx+2), stateIdx:(stateIdx+2));
        avg_pDKAL = trace(Pi_DKAL)/3
        pDKAL_history = [ pDKAL_history; [walltime avg_pDKAL]];
        %%-------------

  end
    
    [~,p] = chol(nm.nodes{mobIdx}.P); % p is zero if +ve definite
    if p~=0
        disp('negative definite')
    end
    s= issymmetric(nm.nodes{mobIdx}.P);
    max(max(abs(nm.nodes{mobIdx}.P'-nm.nodes{mobIdx}.P)))
  
    % update plots
    set(hestAll,'xdata',pEstAll(:,1),'ydata', pEstAll(:,2),'zdata',pEstAll(:,3));
    for i=1:nm.getNumNodes()
        if ~nm.nodes{i}.isMobile()
            set(herrStatic(i),'xdata',[pTruStatic(i,2) pEstAll(i,1)],'ydata',...
                [pTruStatic(i,3) pEstAll(i,2)],'zdata',[pTruStatic(i,4) pEstAll(i,3)]);
        end
        nidx = (i-1)*5 + 1;
        Pi = nm.nodes{i}.P( nidx:(nidx+2), nidx:(nidx+2) );
        updateEllipse( hvar(i), [pEstAll(i,1); pEstAll(i,2); pEstAll(i,3)], Pi + 0.001*[1 0 0; 0 0 0; 0 0 1], varscale);
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
            if latency < 0.250
                set(hrigidbodies(i), 'XData', xyz(1), 'YData', xyz(2), 'ZData', xyz(3));
            end
        end
        
        % update plot title
        tstr = sprintf('Network Localization (t = %.2fs)', (t_last - t_start));
        set(htitle, 'String', tstr);
        drawnow;
    end

    % append state estimate & measurement to history
    s_history = [s_history; s'];
    p_history = [p_history; pEstAll];

    %fprintf('t = %.2f / %.2f \n', meas.getTime()-meas1.getTime(), t_stop);
    
    if SAVEMOVIE
        f = getframe(fig);
        writeVideo(vidObj,f);
    end

    %pause();
end

if SAVEMOVIE
    close(vidObj);
end
savemsgsM=nm.savemsgsM;
sentmsgsD=nm.sentmsgsD;
sentmsgsM=nm.sentmsgsM;
% save data

%saveName= strcat('cache\',logname,'_th_',num2str(PDESIRED));
saveName ='cache\temp';
save(saveName,'savemsgsM','sentmsgsD','sentmsgsM', 'nm', 'k', 's_history', 'p_history', 'cov_history','pDKAL_history','P_big_history' ,'t_history','PDESIRED','t_stop','timeUpdateFlag_history' ,'DiffFlag_history','MeasFlag_history');

end


