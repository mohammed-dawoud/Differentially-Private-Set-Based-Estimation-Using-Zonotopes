classdef NetworkManager < handle
    %NETWORKMANAGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        LIGHTSPEED = 299792458; % m/s
        PLTSTYLES = {'-o', '-s', '-^', '-x', '-*', '--o', '--s', '--^', '--x', '--*'};
        PLTCOLORS = [
            1 0 0
            0 0 1
            0 0 0
            0.8 0.8 0
            1 0 1
            0 0.7 0.7
            0 1 0
            0.5 0.5 0.5
            0.25 0.25 1
            1 0.25 0.25
            0.25 1 0.25
            ];
        PASS = 1;
        BLOCK = 0;
        MSGTYPE1 = 1;
        MSGTYPE2 = 2;
        MSGTYPE3 = 3;
        MEASTYPE_d = 1;
        MEASTYPE_r = 2;
        MEASTYPE_b = 3;
        MEASTYPE_B = 4;
        MEASTYPE_R = 5;
        MAXMEASUREMENTS = 1e6;
        MEASBLOCKBKOFF = 0.010;
    end
    
    properties
        cov_big;
        P_big;
        %dp start
 
        %dp end
        % node variables
        nodes = {};
        nodeinfo = {};
        numnodes = 0;
        propdelay2alpha=[];
        % message / data variables
        dataparser = [];
        antennacorrections = [];
        twrcorrections = [];
        owrcorrections = [];
        msgidx = 1;
        measurement_history = {};
        meascnt = 0;
        
        % range bias state
        rangebiases = [];
        var_rangebias = 0.10;  % m/s
        vari_rangebias = 1.0^2; % m
        
        % message filtering (pass=1,block=0)
        filter_type1 = 0;
        filter_type2 = 0;
        filter_type3 = 0;
        filter_meas_d = 0;
        filter_meas_r = 0;
        filter_meas_b = 0;
        filter_meas_B = 0;
        filter_meas_R = 0;
        
        % min period between message types (sec)
        period_type1 = 0;
        period_type2 = 0;
        period_type3 = 0;
        
        % node blacklist
        blacklist = [];
        
        % message queues (src, dest, type)
        measQ = [];
        
        % reference node id
        refNode = [];
        
        % enable SLATS?
        enable_slats = false;
        
        % number of saved messages
        savemsgsM =0;
        %number of sent messages
        sentmsgsM=0;
        sentmsgsD=0;
        %neighbours including myself
        %  network= { [1 2 3 ],[ 1 2 3 4 5],[ 1 2 3 4],[3 4 5],[4 5 6],[5 6 7],[6 7 8],[1 2 3 4 5 6 7 8]};
        % network= { [1 2 3],[ 1 2 3 4 5],[ 1 2 3 4 5],[3 4 5 6],[4 5 6 7],[5 6 7 8],[4 5 6 7 8],[1 2 3 4 5 6 7 8]};
        %network= {[1 2 3 4 5 ],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[2 3 4 5 6 7 8],[ 2 3 4 5 6 7 8],[ 2 3 4 5 6 7 8]};
        %network= { [9 1 2 ],[1 2 3 4 5 6 7 8],[2 3 4],[3 4 5],[4 5 6],[5 6 7],[6 7 8],[7 8 9],[8 9 1]};
        %  network= { [8 1 2 3],[1 2 3 4 5 6 7 8],[1 2 3 4],[2 3 4 5],[3 4 5 6],[4 5 6 7],[4 6 7 8],[6 7 8 1]};
        %fully connected
        %network= { [1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[ 1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[ 1 2 3 4 5 6 7 8]};
        
        %rm 2 connection
        %network= { [1 2 3 4 5 6 7],[1 2 3 4 5 6 7 8],[ 1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[2 3 4 5 6 7 8]};
        %network= { [1 2 3 4 5 6 7 8],[1 2 3 5 6 7 8],[ 1 2 3 4 5 6 7],[1 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8]};
        
        %1 neig
        % network= { [1 2],[2 3],[3 4],[4 5],[5 6],[6 7],[7 8],[8 1]};
        
        %2 neig
        %network= { [8 1 2],[1 2 3],[2 3 4],[3 4 5],[4 5 6],[5 6 7],[6 7 8],[7 8 1]};
        
        %3 neig
        %network={[1 3 4 6],[2 4 5 8],[1 3 5 7],[1 2 4 7],[2 3 5 8],[1 6 7 8],[3 4 6 7],[2 5 6 8]};
        
        %4 nei
        network = {[1 2 5 7 8],[2 3 6 7 1],[3 4 6 8 2],[4 5 6 8 3],[5 6 7 1 4],[6 2 3 4 5],[7 8 1 2 5],[8 1 3 4 7]};
        
        %5 nei
        %network={[1 2 3 5 7 8],[2 3 4 5 8 1],[3 4 6 8 1 2],[4 5 6 7 2 3],[5 6 7 1 2 4],[6 7 8 3 4 5],[7 8 1 4 5 6],[8 1 2 3 6 7]};
        
        %6 nei
        %                 1               2               3               4               5               6               7               8
        %network= { [8 1 2 3 4 5 6],[1 2 3 4 5 6 7],[1 2 3 4 5 7 8],[3 4 6 7 8 1 2],[3 5 6 7 8 1 2],[4 5 6 7 8 1 2],[6 7 8 2 3 4 5],[7 8 1 3 4 5 6]};
        
        %conditional msg
        %network= { [1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8  9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9]};
        %Gps
        %network= { [1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8]};
        % mobile
        %network= { [1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[3 1 2  4 5 6 7 8 9],[4 1 2 3  5 6 7 8 9],[5 1 2 3 4  6 7 8 9],[6 1 2 3 4 5  7 8 9],[7 1 2 3 4 5 6  8 9],[ 1 2 3 4 5 6 7 8 9],[ 1 2 3 4 5 6 7 8 9]};
        
    end
    
    methods
        % =============================================
        %                  CONSTRUCTOR
        % =============================================
        function obj = NetworkManager(configfile, logfolder, varargin)
            % create dataparser
            obj.dataparser = DataParserROS(configfile, logfolder);
            obj.nodeinfo = obj.dataparser.getNodeInfo();
            % create node objects
            for i=1:length(obj.nodeinfo)
                node_names = obj.nodeinfo{1};
                node_ids = obj.nodeinfo{2};
                node_dynamics = obj.nodeinfo{6};
                for i=1:length(node_ids)
                    name = node_names{i};
                    id = node_ids(i);
                    xyz = [obj.nodeinfo{3}(i); obj.nodeinfo{4}(i); obj.nodeinfo{5}(i)];
                    obj.nodes{i} = Node(id, name);
                    obj.nodes{i}.setTruePosition(xyz);
                    % is this a mobile node?
                    if strcmp(node_dynamics{i}, 'mobile')
                        obj.nodes{i}.setToMobile();
                    end
                end
            end
            obj.numnodes = length(obj.nodeinfo{1});
            
            % load rangefile corrections if they exist % TWR, OWR
            if nargin >= 3
                ant_file = varargin{1};
                obj.antennacorrections = csvread(ant_file);
            end
            if nargin >= 4
                owr_file = varargin{2};
                obj.owrcorrections = csvread(owr_file);
            end
            if nargin >= 5
                twr_file = varargin{3};
                obj.twrcorrections = csvread(twr_file);
            end
            
            % pre-allocate measurement memory
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);
            
            % pre-allocate range bias memory
            obj.rangebiases = zeros( obj.getNumNodes(), obj.getNumNodes() );
        end
        
        % =============================================
        %                     SLATS
        % =============================================
        
        % enable SLATS
        function enableSLATS(obj, val)
            obj.enable_slats = val;
        end
        
        
        % =============================================
        %                MESSAGE FILTERING
        % =============================================
        
        % queue a specific message type
        function queueMessage(obj, srcId, dstId, type)
            % add if Q empty
            if isempty(obj.measQ)
                obj.measQ = [obj.measQ; [srcId dstId type]];
                return;
            end
            % otherwise only add if not already queued
            if isempty( find( obj.measQ(:,1) == srcId & obj.measQ(:,2) == dstId & obj.measQ(:,3) == type ) )
                obj.measQ = [obj.measQ; [srcId dstId type]];
            end
        end
        
        % enable / disable specific message types
        function enableMessageType(obj, type, enable)
            switch type
                case obj.MSGTYPE1
                    obj.filter_type1 = enable;
                case obj.MSGTYPE2
                    obj.filter_type2 = enable;
                case obj.MSGTYPE3
                    obj.filter_type3 = enable;
                otherwise
                    error('Message type not recognized');
            end
        end
        
        % downsample specific message types to a given period lower bound
        function setMessageMinInterval(obj, type, interval)
            switch type
                case obj.MSGTYPE1
                    obj.period_type1 = interval;
                case obj.MSGTYPE2
                    obj.period_type2 = interval;
                case obj.MSGTYPE3
                    obj.period_type3 = interval;
                otherwise
                    error('Message type not recognized');
            end
        end
        
        % enable / disable specific message types
        function enableMeasurementType(obj, type, enable)
            switch type
                case obj.MEASTYPE_d
                    obj.filter_meas_d = enable;
                case obj.MEASTYPE_r
                    obj.filter_meas_r = enable;
                case obj.MEASTYPE_b
                    obj.filter_meas_b = enable;
                case obj.MEASTYPE_B
                    obj.filter_meas_B = enable;
                case obj.MEASTYPE_R
                    obj.filter_meas_R = enable;
                otherwise
                    error('Message type not recognized');
            end
        end
        
        % determine whether or not this measurement should be received
        function valid = filterMeasurement(obj, meas)
            srcId = meas.getSourceId();
            dstId = meas.getDestId();
            type = meas.getType();
            tlastSent = obj.nodes{ obj.getNodeIdx(srcId) }.getLastMeasTime( type );
            
            % ignore measurement if src or dst are blacklisted
            if ~isempty( find(obj.blacklist == srcId, 1) ) || ~isempty( find(obj.blacklist == dstId, 1) )
                valid = obj.BLOCK;
                return;
            end
            
            % set allowed measurement types % to know for vectorizing
            if obj.filter_meas_d
                meas.allowMeasType_d();
            end
            if obj.filter_meas_r
                meas.allowMeasType_r();
            end
            if obj.filter_meas_R
                meas.allowMeasType_R();
            end
            
            % next, pass immediately if measurement type has been requested
            if ~isempty( obj.measQ )
                qIdx = find(obj.measQ(:,1) ==  srcId & obj.measQ(:,2) == dstId);
                if ~isempty(qIdx)
                    % force this message to be the right type
                    meas.type = obj.measQ(qIdx,3);
                    meas.queued = 1;
                    % a message of this type has been requested, pass it and remove
                    % from the message queue
                    
                    % block if the range or time is obviously incorrect
                    if meas.r_ij < -1 || meas.r_ij > 20 || meas.R_ij < -1 || meas.R_ij > 20
                        valid = obj.BLOCK;
                        obj.measQ(qIdx,:) = [];
                        return;
                    end
                    valid = obj.PASS;
                    obj.measQ(qIdx,:) = [];
                    return;
                end
            end
            
            % block if no measurements supported
            if ~(obj.filter_meas_d || obj.filter_meas_r || obj.filter_meas_R)
                valid = obj.BLOCK;
                return;
            end
            
            % block if the range or time is obviously incorrect
%dp             if meas.r_ij < -1 || meas.r_ij > 20 || meas.R_ij < -1 || meas.R_ij > 20
            if meas.R_ij < -1 || meas.R_ij > 20

                valid = obj.BLOCK;
                return;
            end
            
            
            % pass as a type 3 message if they're allowed and it's been
            % long enough since the last type 3 message
            if obj.filter_type3 == obj.PASS
                
                meas.setType(obj.MSGTYPE3);
                tlastSent = obj.nodes{ obj.getNodeIdx(srcId) }.getLastMeasTime( obj.MSGTYPE3 );
                dt = meas.getTime() - tlastSent;
                if dt >= obj.period_type3 || dt < obj.MEASBLOCKBKOFF
                    valid = obj.PASS;
                    obj.nodes{ obj.getNodeIdx(srcId) }.setLastMeasTime( obj.MSGTYPE3, meas.getTime() );
                    return;
                end
            end
            % otherwise make this a type 2 message and pass it if they're
            % allowed and if it's been long enough since the last type 2.
            if obj.filter_type2 == obj.PASS
                % downgrade this to a type 2 message
                meas.setType(obj.MSGTYPE2);
                tlastSent = obj.nodes{ obj.getNodeIdx(srcId) }.getLastMeasTime( obj.MSGTYPE2 );
                dt = meas.getTime() - tlastSent;
                if dt >= obj.period_type2 || dt < obj.MEASBLOCKBKOFF
                    valid = obj.PASS;
                    obj.nodes{ obj.getNodeIdx(srcId) }.setLastMeasTime( obj.MSGTYPE2, meas.getTime() );
                    return;
                end
            end
            % finally, we can at the very least make this a type 1 message
            % if the other 2 messages are blocked for this time period.
            % Disallow this too, though, if not enough time has elapsed
            % since the last type 1.
            if obj.filter_type1 == obj.PASS
                % downgrade this to a type 1 message
                meas.setType(obj.MSGTYPE1);
                tlastSent = obj.nodes{ obj.getNodeIdx(srcId) }.getLastMeasTime( obj.MSGTYPE1 );
                dt = meas.getTime() - tlastSent;
                if dt >= obj.period_type1 || dt < obj.MEASBLOCKBKOFF
                    valid = obj.PASS;
                    obj.nodes{ obj.getNodeIdx(srcId) }.setLastMeasTime( obj.MSGTYPE1, meas.getTime() );
                    return;
                end
            end
            
            % otherwise, block it
            valid = obj.BLOCK;
        end
        
        % =============================================
        %                  PLOTTING
        % =============================================
        
        % plot styles and colors
        function style = getPlotStyle(obj, nodeId)
            nidx = obj.dataparser.getNodeIdx(nodeId);
            style = obj.PLTSTYLES{nidx};
        end
        function c = getPlotColor(obj, nodeId)
            nidx = obj.dataparser.getNodeIdx(nodeId);
            c = obj.PLTCOLORS(nidx,:);
        end
        
        % get number of nodes
        function n = getNumNodes(obj)
            n = obj.numnodes;
        end
        
        % get number of static nodes
        function n = getNumStaticNodes(obj)
            n = 0;
            for i=1:obj.getNumNodes()
                dynamics = obj.nodeinfo{6}{i};
                if strcmp(dynamics, 'static')
                    n = n + 1;
                end
            end
        end
        
        % get number of mobile nodes
        function n = getNumMobileNodes(obj)
            n = obj.getNumNodes() - obj.getNumStaticNodes();
        end
        
        % =============================================
        %                  NODE INFO
        % =============================================
        
        % get node ascii names
        function n = getNodeNames(obj)
            n = obj.nodeinfo{1};
        end
        
        % get numeric node is: 0, 1, ...
        function id = getNodeIds(obj)
            id = obj.nodeinfo{2};
        end
        
        % get the index of a specific node
        function idx = getNodeIdx(obj, nodeid)
            idx = obj.dataparser.getNodeIdx( nodeid );
        end
        
        % blacklist a certain node (don't use it)
        function blacklistNode(obj, nodeid)
            idx = obj.getNodeIdx( nodeid );
            nodeids = obj.getNodeIds();
            id = nodeids(idx);
            obj.blacklist = [obj.blacklist; id];
        end
        
        % =============================================
        %               STATE ACCESSORS
        % =============================================
        
        % get network state
        function s = getState(obj)
            s = [];
            for i=1:length(obj.nodes)
                s = [s; obj.nodes{i}.getState()];
            end
        end
        % start-dp
        function s = getStatedp(obj)
            s = [];
            for i=9:length(obj.nodes)
                s = [s; obj.nodes{i}.getState()];
            end
        end
        %end-dp
        % get the vectorized range biases
        function sb = getRangeBiasVec(obj)
            sb = [];
            for i=1:obj.getNumNodes()
                for j=(i+1):obj.getNumNodes()
                    sb = [sb; obj.rangebiases(i,j)];
                end
            end
        end
        
        % range bias vector to matrix
        function SB = rangeBiasVec2Mat(obj, sb)
            SB = [];
            idx = 0;
            for i=1:obj.getNumNodes()
                for j=(i+1):obj.getNumNodes()
                    idx = idx + 1;
                    SB(i,j) = sb(idx);
                    SB(j,i) = sb(idx);
                end
            end
        end
        
        % set the vectorized range biases
        function setRangeBiasState(obj, sb)
            obj.rangebiases = obj.rangeBiasVec2Mat(sb);
        end
        
        % set network state
        function setState(obj, s)
            idx = 1;
            for i=1:length(obj.nodes)
                stateSize = length(obj.nodes{i}.getState());
                si = s(idx:(idx+stateSize-1));
                obj.nodes{i}.setState(si);
                 idx = idx + stateSize;
            end
            
        end
        
        function setStatedp(obj, s)
            for i=1:length(obj.nodes)-1
                %stateSize = length(obj.nodes{i}.getState());
                
            obj.nodes{i}.setState(obj.nodes{i}.getTruePosition);    
            end
            obj.nodes{9}.setState(s);
        end
        
        %every node participate with its state only
        function s=getAndFixStateConc(obj)
            idx = 1;
            s = [];
            for i=1:length(obj.nodes)
                stateSize = length(obj.nodes{i}.getState());
                si = obj.nodes{i}.x(idx:(idx+stateSize-1));
                obj.nodes{i}.setState(si);
                s = [ s ; si];
                idx = idx + stateSize;
            end
            
        end
        % get estimated node positions for only static nodes
        function P = getEstimatedStaticPositions(obj)
            P = [];
            for i=1:length(obj.nodes)
                if ~obj.nodes{i}.isMobile()
                    P = [P; obj.nodes{i}.getId() obj.nodes{i}.getStatePosition()'];
                end
            end
        end
        
        % get estimated node positions for all nodes
        function P = getEstimatedPositions(obj)
            P = [];
            for i=1:length(obj.nodes)
                P = [P; obj.nodes{i}.getStatePosition()'];
            end
        end
        
        % get true node positions
        function P = getTrueStaticPositions(obj)
            P = [];
            for i=1:length(obj.nodes)
                if ~obj.nodes{i}.isMobile()
                    P = [P; obj.nodes{i}.getId() obj.nodes{i}.getTruePosition()'];
                end
            end
        end
        
        % get estimated positions transformed to match true as closely as
        % possible (procrustes)
        function P = getTransformedPositions(obj)
            truStatic = obj.getTrueStaticPositions();
            estStatic = obj.getEstimatedStaticPositions();
            % procrustes ignoring first column (node IDs)
            [~,~,Transform] = procrustes(truStatic(:,2:end), estStatic(:,2:end), 'Scaling', false);
            % transform all points
            estAll = obj.getEstimatedPositions();
            ofst = repmat(Transform.c(1,:), obj.getNumNodes(), 1);
            P = Transform.b*estAll*Transform.T + ofst;
        end
        
        % set reference node (for EKF)
        function setReferenceNode(obj, nodeId)
            nidx = obj.dataparser.getNodeIdx(nodeId);
            obj.nodes{nidx}.setAsReference();
            obj.refNode = obj.nodes{nidx}.getId();
        end
        
        % =============================================
        %               PROCESS & MEASUREMENT
        % =============================================
        
        % get process variance
        function P = getProcessVar(obj)
            p = [];
            for i=1:length(obj.nodes)
                p = [p; obj.nodes{i}.getProcessVar()];
            end
            P = diag(p);
        end
        
        % get initial variance
        function P = getInitialVar(obj)
            p = [];
            for i=1:length(obj.nodes)
                p = [p; obj.nodes{i}.getInitialVar()];
            end
            P = diag(p);
        end
        
        
        % state process function
        function snew = processFcn(obj, s, dt)
            snew = s;
            
            % perform process update over each node separately
            stateSize = 5;
            i = 1;
            for n=1:obj.getNumNodes()
                snew(i+3) = s(i+3) + s(i+4) * 1e-9 * dt;
                i = i + stateSize;
            end
        end
        %start-dp
        function snewdp = processFcndp(obj, s, dt)
            snewdp = s;
            
%             % perform process update over each node separately
%             stateSize = 4;
%             i = 1;
%             for n=1:obj.getNumNodes()
%                 snewdp(i+3) = s(i+3) + s(i+4) * 1e-9 * dt;
%                 i = i + stateSize;
%             end
        end
        %end-dp
        
        
        % state measurement function
        function y = measurementFcn(obj, s, meas)
            type = meas.getType();
            
            srcIdx = obj.getNodeIdx(meas.getSourceId());
            dstIdx = obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            stateSize = 3; %dp 5;
            start_i = (srcIdx-1)*stateSize + 1;
            start_j = (dstIdx-1)*stateSize + 1;
            
            % extract the needed state info
            pi = s(start_i:(start_i+2));
            pj = s(start_j:(start_j+2));
            oi = s(start_i+3);
            oj = s(start_j+3);
            bi = s(start_i+4);
            bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];
            y = [];
            
            c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
            if obj.filter_meas_d
                if obj.enable_slats
                    y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
                else
                    y = [y; (oj - oi)];
                end
            end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
            if obj.filter_meas_r
                if obj.enable_slats
                    y = [y; sqrt(sum((pj-pi).^2)) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
                else
                    y = [y; sqrt(sum((pj-pi).^2))];
                end
            end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
                    err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    y = [y; sqrt(sum((pj-pi).^2)) + err ];
                else
                    y = [y; sqrt(sum((pj-pi).^2))];
                end
            end
        end
        
        %dp-start
        
        function y = measurementFcnrest(obj, s, meas)
            
%dp             zm = zonotope([0],[.01 0.02]); %dp
%dp             rng(obj.meascnt,'twister'); %dp
%dp             p1 = randPoint(zm); %dp
                    
            type = meas.getType();
            
            srcIdx = obj.getNodeIdx(meas.getSourceId());
            dstIdx = obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            %dp start
%dp             stateSize = 3; %dp 5;
%dp             start_i = (srcIdx-1)*stateSize + 1;
%dp             start_j = (dstIdx-1)*stateSize + 1;
            
            pi=[];
            pj=[];
            %dp end
            %pi = s(start_i:(start_i+2));
            % extract the needed state info
            if meas.nodei==10
            pi = s; %dp (start_i:(start_i+2));
            pj = obj.nodes{meas.nodej+1}.true_p;
            elseif meas.nodej==10
                  pi = obj.nodes{meas.nodei+1}.true_p;
                  pj = s; %dp (start_j:(start_j+2));
                
            else
                pi = obj.nodes{meas.nodei+1}.true_p;
                pj = obj.nodes{meas.nodej+1}.true_p;

            end
%dp             %pj = s(start_j:(start_j+2));
%dp             oi = s(start_i+3);
%dp             oj = s(start_j+3);
%dp             bi = s(start_i+4);
%dp             bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];
            y = [];
            c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
%             if obj.filter_meas_d
%                 if obj.enable_slats
%                     y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
%                 else
%                     y = [y; (oj - oi)];
%                 end
%             end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
%             if obj.filter_meas_r
%                 if obj.enable_slats
%                     y = [y; (pj-pi) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
%                 else
%                     y = [y; (pj-pi)];
%                 end
%             end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
 
                    
%dp                     err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    %y = [y; sqrt(sum((pj-pi).^2))+p1]; %+ err ];
                    y = [y; sqrt(sum((pj-pi).^2))];
                else
                    %y = [y; sqrt(sum((pj-pi).^2))+p1];
                    y = [y; sqrt(sum((pj-pi).^2))];
                end
            end
        end

        function y = measurementFcnCentralized(obj, s, meas)
            
%dp             zm = zonotope([0],[.01 0.02]); %dp
%dp             rng(obj.meascnt,'twister'); %dp
%dp             p1 = randPoint(zm); %dp
                    
            type =3;% meas.getType();
            
            srcIdx = meas(2);%obj.getNodeIdx(meas.getSourceId());
            dstIdx = meas(3);%obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            %dp start
%dp             stateSize = 3; %dp 5;
%dp             start_i = (srcIdx-1)*stateSize + 1;
%dp             start_j = (dstIdx-1)*stateSize + 1;
            
            pi=[];
            pj=[];
            %dp end
            %pi = s(start_i:(start_i+2));
            % extract the needed state info
            if meas(2)==10
            pi = s; %dp (start_i:(start_i+2));
            pj = obj.nodes{meas(3)+1}.true_p;
            elseif meas(3)==10
                  pi = obj.nodes{meas(2)+1}.true_p;
                  pj = s; %dp (start_j:(start_j+2));
                
            else
                pi = obj.nodes{meas(2)+1}.true_p;
                pj = obj.nodes{meas(3)+1}.true_p;

            end
%dp             %pj = s(start_j:(start_j+2));
%dp             oi = s(start_i+3);
%dp             oj = s(start_j+3);
%dp             bi = s(start_i+4);
%dp             bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];
            y = [];
            c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
%             if obj.filter_meas_d
%                 if obj.enable_slats
%                     y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
%                 else
%                     y = [y; (oj - oi)];
%                 end
%             end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
%             if obj.filter_meas_r
%                 if obj.enable_slats
%                     y = [y; (pj-pi) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
%                 else
%                     y = [y; (pj-pi)];
%                 end
%             end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
 
                    
%dp                     err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    %y = [y; sqrt(sum((pj-pi).^2))+p1]; %+ err ];
                    y = [y; sqrt(sum((pj-pi).^2))];
                else
                    %y = [y; sqrt(sum((pj-pi).^2))+p1];
                    y = [y; sqrt(sum((pj-pi).^2))];
                end
            end
        end

        function [yset,pi,pj] = measurementFcnset(obj, s, meas)
            
            zm = zonotope([0],[.01 0.02]); %dp
            rng(obj.meascnt,'twister'); %dp
            p1 = randPoint(zm); %dp
                    
            type = meas.getType();
            
            srcIdx = obj.getNodeIdx(meas.getSourceId());
            dstIdx = obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            %dp start
%dp             stateSize = 3; %dp 5;
%dp             start_i = (srcIdx-1)*stateSize + 1;
%dp             start_j = (dstIdx-1)*stateSize + 1;
            
            pi=[];
            pj=[];
            %dp end
            %pi = s(start_i:(start_i+2));
            % extract the needed state info
            if meas.nodei==10
            pi = s; %dp (start_i:(start_i+2));
            pj = obj.nodes{meas.nodej+1}.true_p;
            elseif meas.nodej==10
                  pi = obj.nodes{meas.nodei+1}.true_p;
                  pj = s; %dp (start_j:(start_j+2));
                
            else
                pi = obj.nodes{meas.nodei+1}.true_p;
                pj = obj.nodes{meas.nodej+1}.true_p;

            end
%dp             %pj = s(start_j:(start_j+2));
%dp             oi = s(start_i+3);
%dp             oj = s(start_j+3);
%dp             bi = s(start_i+4);
%dp             bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];
            y = [];
            yset = []; %dp
            c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
%             if obj.filter_meas_d
%                 if obj.enable_slats
%                     y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
%                 else
%                     y = [y; (oj - oi)];
%                 end
%             end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
%             if obj.filter_meas_r
%                 if obj.enable_slats
%                     y = [y; (pj-pi) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
%                 else
%                     y = [y; (pj-pi)];
%                 end
%             end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
 
                    
%dp                     err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    %y = [y; sqrt(sum((pj-pi).^2))+p1]; %+ err ];
                    yset = [yset; sqrt(sum((pj-pi).^2))+p1];
                else
                    %y = [y; sqrt(sum((pj-pi).^2))+p1];
                    yset = [yset; sqrt(sum((pj-pi).^2))+p1];
                end
            end
        end
        function [yset,pi,pj] = measurementFcnsetCentralized(obj, s, meas)
            
            zm = zonotope([0],[.01 0.02]); %dp
            rng(obj.meascnt,'twister'); %dp
            p1 = randPoint(zm); %dp
                    
            type =3; %meas.getType();
            
            srcIdx = meas(2);%obj.getNodeIdx(meas.getSourceId());
            dstIdx = meas(3);%obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            %dp start
%dp             stateSize = 3; %dp 5;
%dp             start_i = (srcIdx-1)*stateSize + 1;
%dp             start_j = (dstIdx-1)*stateSize + 1;
            
            pi=[];
            pj=[];
            %dp end
            %pi = s(start_i:(start_i+2));
            % extract the needed state info
            if meas(2)==10
            pi = s; %dp (start_i:(start_i+2));
            pj = obj.nodes{meas(3)+1}.true_p;
            elseif meas(3)==10
                  pi = obj.nodes{meas(2)+1}.true_p;
                  pj = s; %dp (start_j:(start_j+2));
                
            else
                pi = obj.nodes{meas(2)+1}.true_p;
                pj = obj.nodes{meas(3)+1}.true_p;

            end
%dp             %pj = s(start_j:(start_j+2));
%dp             oi = s(start_i+3);
%dp             oj = s(start_j+3);
%dp             bi = s(start_i+4);
%dp             bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];
            y = [];
            yset = []; %dp
            c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
%             if obj.filter_meas_d
%                 if obj.enable_slats
%                     y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
%                 else
%                     y = [y; (oj - oi)];
%                 end
%             end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
%             if obj.filter_meas_r
%                 if obj.enable_slats
%                     y = [y; (pj-pi) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
%                 else
%                     y = [y; (pj-pi)];
%                 end
%             end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
 
                    
%dp                     err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    %y = [y; sqrt(sum((pj-pi).^2))+p1]; %+ err ];
                    yset = [yset; sqrt(sum((pj-pi).^2))+p1];
                else
                    %y = [y; sqrt(sum((pj-pi).^2))+p1];
                    yset = [yset; sqrt(sum((pj-pi).^2))+p1];
                end
            end
        end

        function [ydp,yset,pi,pj] = measurementFcndp2(obj, s, meas)
            %dp-start-measurement-noise-start
            zm = zonotope([0],[.01 0.02]); %dp
            rng(obj.meascnt,'twister'); %dp
            p1 = randPoint(zm); %dp
            %dp-start-measurement-noise-end
            
            %dp-optimal-noise-start
            opt_noise_table = readtable("logs/optimal_noise_f.csv","ReadVariableNames",true);%12-8-2022 
            opt_noise_tablex=table2array(opt_noise_table(:,1));%12-8-2022 
            opt_noise_tabley=table2array(opt_noise_table(:,2));%12-8-2022
            opt_noise_tabley = opt_noise_tabley/sum(opt_noise_tabley); % Make sure probabilites add up to 1.
            cp_opt = [0, cumsum(opt_noise_tabley.')];
            rng(obj.meascnt,'twister'); %dp
            r1 = rand;
            ind1 = find(r1>cp_opt, 1, 'last');
            resultdp_opt1 = opt_noise_tablex(ind1);
            %r2 = rand;
            %ind2 = find(r2>cp_opt, 1, 'last');
            %resultdp_opt2 = opt_noise_tablex(ind2)
            %figure;
            %plot(opt_noise_tablex,opt_noise_tabley);%12-8-2022
            
           
            %dp-optimal-noise-end
            
            
            
                    
            
%dp             srcIdx = obj.getNodeIdx(meas.getSourceId());
%dp             dstIdx = obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            %dp start
%dp             stateSize = 3; %dp 5;
%dp             start_i = (srcIdx-1)*stateSize + 1;
%dp             start_j = (dstIdx-1)*stateSize + 1;
            type = meas.getType();
            pi=[];
            pj=[];
            yset = [];%dp
            ydp = []; %dp
            %dp end
            %pi = s(start_i:(start_i+2));
            % extract the needed state info
            if meas.nodei==10
            pi = s; %dp (start_i:(start_i+2));
            pj = obj.nodes{meas.nodej+1}.true_p;
            elseif meas.nodej==10
                  pi = obj.nodes{meas.nodei+1}.true_p;
                  pj = s; %dp (start_j:(start_j+2));
                
            else
                pi = obj.nodes{meas.nodei+1}.true_p;
                pj = obj.nodes{meas.nodej+1}.true_p;

            end
%dp             %pj = s(start_j:(start_j+2));
%dp             oi = s(start_i+3);
%dp             oj = s(start_j+3);
%dp             bi = s(start_i+4);
%dp             bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];

%dp             c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
%             if obj.filter_meas_d
%                 if obj.enable_slats
%                     y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
%                 else
%                     y = [y; (oj - oi)];
%                 end
%             end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
%             if obj.filter_meas_r
%                 if obj.enable_slats
%                     y = [y; (pj-pi) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
%                 else
%                     y = [y; (pj-pi)];
%                 end
%             end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
 
                    
%dp                 err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    %y = [y; sqrt(sum((pj-pi).^2))+p1]; %+ err ];
                    ydp = [ydp; sqrt(sum((pj-pi).^2))+p1+resultdp_opt1];
                    yset = [yset; sqrt(sum((pj-pi).^2))+p1];

                else
                    %y = [y; sqrt(sum((pj-pi).^2))+p1];
                    ydp = [ydp; sqrt(sum((pj-pi).^2))+p1+resultdp_opt1];
                    yset = [yset; sqrt(sum((pj-pi).^2))+p1];
                end
            end
        end

        
        
        function ydp = measurementFcndp_Optimal_noise(obj, s, meas)
            %dp-start-measurement-noise-start
            zm = zonotope([0],[.01 0.02]); %dp
            rng(obj.meascnt,'twister'); %dp
            p1 = randPoint(zm); %dp
            %dp-start-measurement-noise-end
            
            %dp-optimal-noise-start
            opt_noise_table = readtable("logs/optimal_noise_f.csv","ReadVariableNames",true);%12-8-2022 
            opt_noise_tablex=table2array(opt_noise_table(:,1));%12-8-2022 
            opt_noise_tabley=table2array(opt_noise_table(:,2));%12-8-2022
            opt_noise_tabley = opt_noise_tabley/sum(opt_noise_tabley); % Make sure probabilites add up to 1.
            cp_opt = [0, cumsum(opt_noise_tabley.')];
            rng(obj.meascnt,'twister'); %dp
            r1 = rand;
            ind1 = find(r1>cp_opt, 1, 'last');
            resultdp_opt1 = opt_noise_tablex(ind1);
            %r2 = rand;
            %ind2 = find(r2>cp_opt, 1, 'last');
            %resultdp_opt2 = opt_noise_tablex(ind2)
            %figure;
            %plot(opt_noise_tablex,opt_noise_tabley);%12-8-2022
            
           
            %dp-optimal-noise-end
            
            
            
                    
            
%dp             srcIdx = obj.getNodeIdx(meas.getSourceId());
%dp             dstIdx = obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            %dp start
%dp             stateSize = 3; %dp 5;
%dp             start_i = (srcIdx-1)*stateSize + 1;
%dp             start_j = (dstIdx-1)*stateSize + 1;
            type = meas.getType();
            pi=[];
            pj=[];
            yset = [];%dp
            ydp = []; %dp
            %dp end
            %pi = s(start_i:(start_i+2));
            % extract the needed state info
            if meas.nodei==10
            pi = s; %dp (start_i:(start_i+2));
            pj = obj.nodes{meas.nodej+1}.true_p;
            elseif meas.nodej==10
                  pi = obj.nodes{meas.nodei+1}.true_p;
                  pj = s; %dp (start_j:(start_j+2));
                
            else
                pi = obj.nodes{meas.nodei+1}.true_p;
                pj = obj.nodes{meas.nodej+1}.true_p;

            end
%dp             %pj = s(start_j:(start_j+2));
%dp             oi = s(start_i+3);
%dp             oj = s(start_j+3);
%dp             bi = s(start_i+4);
%dp             bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];

%dp             c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
%             if obj.filter_meas_d
%                 if obj.enable_slats
%                     y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
%                 else
%                     y = [y; (oj - oi)];
%                 end
%             end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
%             if obj.filter_meas_r
%                 if obj.enable_slats
%                     y = [y; (pj-pi) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
%                 else
%                     y = [y; (pj-pi)];
%                 end
%             end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
 
                    
%dp                 err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    %y = [y; sqrt(sum((pj-pi).^2))+p1]; %+ err ];
                    ydp = [ydp; sqrt(sum((pj-pi).^2))+p1+resultdp_opt1];
                    %yset = [yset; sqrt(sum((pj-pi).^2))+p1];

                else
                    %y = [y; sqrt(sum((pj-pi).^2))+p1];
                    ydp = [ydp; sqrt(sum((pj-pi).^2))+p1+resultdp_opt1];
                    %yset = [yset; sqrt(sum((pj-pi).^2))+p1];
                end
            end
        end
        function ydp = measurementFcndp_laplace_noise(obj, s, meas)
            %dp-start-measurement-noise-start
            zm = zonotope([0],[.01 0.02]); %dp
            rng(obj.meascnt,'twister'); %dp
            p1 = randPoint(zm); %dp
            %dp-start-measurement-noise-end
            
            %dp-laplace-noise-start
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
            r_Laplace1 = rand;
            ind_Laplace1 = find(r_Laplace1>cp_Laplace, 1, 'last');
            resultdp_Laplace1 = x_m(ind_Laplace1);
            
           
            %dp-laplace-noise-end
            
            
            
                    
            
%dp             srcIdx = obj.getNodeIdx(meas.getSourceId());
%dp             dstIdx = obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            %dp start
%dp             stateSize = 3; %dp 5;
%dp             start_i = (srcIdx-1)*stateSize + 1;
%dp             start_j = (dstIdx-1)*stateSize + 1;
            type = meas.getType();
            pi=[];
            pj=[];
            yset = [];%dp
            ydp = []; %dp
            %dp end
            %pi = s(start_i:(start_i+2));
            % extract the needed state info
            if meas.nodei==10
            pi = s; %dp (start_i:(start_i+2));
            pj = obj.nodes{meas.nodej+1}.true_p;
            elseif meas.nodej==10
                  pi = obj.nodes{meas.nodei+1}.true_p;
                  pj = s; %dp (start_j:(start_j+2));
                
            else
                pi = obj.nodes{meas.nodei+1}.true_p;
                pj = obj.nodes{meas.nodej+1}.true_p;

            end
%dp             %pj = s(start_j:(start_j+2));
%dp             oi = s(start_i+3);
%dp             oj = s(start_j+3);
%dp             bi = s(start_i+4);
%dp             bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];

%dp             c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
%             if obj.filter_meas_d
%                 if obj.enable_slats
%                     y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
%                 else
%                     y = [y; (oj - oi)];
%                 end
%             end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
%             if obj.filter_meas_r
%                 if obj.enable_slats
%                     y = [y; (pj-pi) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
%                 else
%                     y = [y; (pj-pi)];
%                 end
%             end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
 
                    
%dp                 err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    %y = [y; sqrt(sum((pj-pi).^2))+p1]; %+ err ];
                    ydp = [ydp; sqrt(sum((pj-pi).^2))+p1+resultdp_Laplace1];
                    %yset = [yset; sqrt(sum((pj-pi).^2))+p1];

                else
                    %y = [y; sqrt(sum((pj-pi).^2))+p1];
                    ydp = [ydp; sqrt(sum((pj-pi).^2))+p1+resultdp_Laplace1];
                    %yset = [yset; sqrt(sum((pj-pi).^2))+p1];
                end
            end
        end

        
       function [sup] = getSupremumPositions(obj)
           for i=9:9%1:obj.numnodes
               % put
               sup(1,[1 2 3])=supremum(interval(obj.nodes{i}.x_zonotope));
           end
       end
       
       function [infi] = getInfimumPositions(obj)
           for i=9:9%1:obj.numnodes
               % put
               infi(1,[1 2 3])=infimum(interval(obj.nodes{i}.x_zonotope));
           end
       end 
        %dp-end
        
        
        % request the next (filtered) measurement
        function m = getNextMeasurement(obj)
            while obj.msgidx < obj.dataparser.getNumMeasurements()
                % get new tentative meas
                raw = obj.dataparser.getMeasurement(obj.msgidx);
                meas = Measurement(obj.MSGTYPE3, raw);%calulate dij,rij,Rij
                obj.msgidx = obj.msgidx + 1;
                % check if we should pass this message
                if obj.filterMeasurement(meas) == obj.PASS
                    % ANT corrections
%dp-start                     if ~isempty(obj.antennacorrections)
%                         idx = find(obj.antennacorrections(:,1) == meas.getSourceId() &...
%                             obj.owrcorrections(:,2) == meas.getDestId());
%                         if ~isempty(idx)
%                             meas.d_ij = meas.d_ij + 0*1e-9*obj.antennacorrections(idx, 3);
%                         end
%dp-end                      end
                   % OWR corrections
%dp-start                     if ~isempty(obj.owrcorrections)
%                         idx = find(obj.owrcorrections(:,1) == meas.getSourceId() &...
%                             obj.owrcorrections(:,2) == meas.getDestId());
%                         if ~isempty(idx)
%                             meas.r_ij = meas.r_ij + obj.owrcorrections(idx, 3);
%                         end
%dp-end                     end
                    % TWR corrections
                    if ~isempty(obj.twrcorrections)
                        idx = find(obj.twrcorrections(:,1) == meas.getSourceId() &...
                            obj.twrcorrections(:,2) == meas.getDestId());
                        if ~isempty(idx)
                            meas.R_ij = meas.R_ij + obj.twrcorrections(idx, 3);
                        end
                    end
                    
                    m = meas;
                    
                    % increment and check message count
                    if obj.meascnt > obj.MAXMEASUREMENTS
                        m = [];
                        return;
                    end
                    obj.meascnt = obj.meascnt + 1;
                    obj.measurement_history{obj.meascnt} = m;
                    return;
                end
            end
            % no more measurements
            m = [];
        end
        
        function killLastMeasurement(obj)
            obj.meascnt = obj.meascnt - 1;
        end
        
        % reset measurement index
        function resetMeasurements(obj)
            obj.msgidx = 1;
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);
            obj.meascnt = 0;
        end
        
        % =============================================
        %                  BOOTSTRAPPING
        % =============================================
        
        % skip some time
        function skipTime(obj, tskip)
            % enable type 3 messages for this
            type3Enabled = obj.filter_type3;
            typeREnabled = obj.filter_meas_R;
            obj.enableMessageType(obj.MSGTYPE3, true);
            obj.enableMeasurementType(obj.MEASTYPE_R, true);
            
            % get the first measurement
            meas1 = obj.getNextMeasurement();
            t1 = meas1.getTime();
            t_now = t1;
            while t_now - t1 < tskip
                meas = obj.getNextMeasurement();
                if isempty(meas)
                    break;
                end
                t_now = meas.getTime();
            end
            
            % reset the measurement history but not the dp index
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);
            obj.meascnt = 0;
            
            % set type 3 filtering to whatever it was before bootstrap
            obj.enableMessageType(obj.MSGTYPE3, type3Enabled);
            obj.enableMeasurementType(obj.MEASTYPE_R, typeREnabled);
        end
        
        % bootstrap node positions using ranges averaged over a duration
        function bootstrapNodeStates(obj, T_boot)
            % enable type 3 messages for this
            type3Enabled = obj.filter_type3;
            typeREnabled = obj.filter_meas_R;
            obj.enableMessageType(obj.MSGTYPE3, true);
            obj.enableMeasurementType(obj.MEASTYPE_R, true);
            
            % get range estimates (distance matrix)
            R_sum = zeros( obj.getNumNodes(), obj.getNumNodes() );
            % get offset estimates
            D_sum = zeros( obj.getNumNodes(), 1 );
            % distance measurement counts
            D_cnt = zeros( obj.getNumNodes(), obj.getNumNodes() );
            % timing measurements
            d_ij_arrays = cell( obj.getNumNodes(), 1 );
            d_ij_times  = cell( obj.getNumNodes(), 1 );
            % get the first measurement
            meas1 = obj.getNextMeasurement();
            t1 = meas1.getTime();
            t_now = t1;
            while t_now - t1 < T_boot
                meas = obj.getNextMeasurement();
                if isempty(meas)
                    break;
                end
                t_now = meas.getTime();
                srcIdx = obj.getNodeIdx( meas.getSourceId() );
                dstIdx = obj.getNodeIdx( meas.getDestId() );
                
                R_sum(srcIdx,dstIdx) = R_sum(srcIdx,dstIdx) + meas.R_ij;
                R_sum(dstIdx,srcIdx) = R_sum(dstIdx,srcIdx) + meas.R_ij;
                D_cnt(srcIdx,dstIdx) = D_cnt(srcIdx,dstIdx) + 1;
                D_cnt(dstIdx,srcIdx) = D_cnt(dstIdx,srcIdx) + 1;
                
                % look for messages from the reference node to bootstrap
                % clock offset and bias
%dp-start                 if ~isempty(obj.refNode) && meas.getSourceId() == obj.refNode
%                     % append d_ij and time
%                     d_ij_arrays{dstIdx} = [d_ij_arrays{dstIdx} meas.d_ij];
%                     d_ij_times{dstIdx} = [d_ij_times{dstIdx} meas.getTime()];
%dp-end                 end
            end
            
            % average range estimates
            D = zeros( obj.getNumNodes(), obj.getNumNodes() );
            W = ones( obj.getNumNodes(), obj.getNumNodes() );
            for i=1:obj.getNumNodes()
                for j=1:obj.getNumNodes()
                    % ignore diagonals
                    if i == j
                        continue;
                    end
                    % assign default or average measurements
                    if D_cnt(i,j) == 0
                        % if we have no measurement, assume it's a bit far
                        % away and give it very low weight
                        D(i,j) = 10;
                        W(i,j) = 0.01;
                    else
                        D(i,j) = R_sum(i,j) / D_cnt(i,j);
                    end
                end
            end
            
            % use MDS to get an estimate of the node positions
            Y = mdscale(D, 3, 'Weights', W);
            
            % assign the estimated positions to each node
            for i=1:obj.getNumNodes()
                obj.nodes{i}.setStatePosition( Y(i,:) );
            end
            
            % perform linear regression over d_ij and time to get estimated
            % clock offset and bias w.r.t. reference node
            for i=1:size(d_ij_arrays,1)
                if length(d_ij_arrays{i}) >= 3
                    % wall time = x
                    x = d_ij_times{i} - d_ij_times{i}(1);
                    % measured d_ij = y
                    y = d_ij_arrays{i};
                    p = polyfit(x, y, 1);
                    est_bias = p(1);
                    est_ofst = polyval(p, x(end));
                    obj.nodes{i}.setStateClockOfst(est_ofst);
                    obj.nodes{i}.setStateClockBias(est_bias*1e9);
                    fprintf('BOOTSTRAP: node %d: offset= %.6f ms, bias = %d ppb\n', obj.nodes{i}.getId(), est_ofst*1e3, est_bias*1e9);
                elseif obj.nodes{i}.getId() ~= obj.refNode
                    fprintf('WARN: No timing bootstrap for %d-->%d (%d measurements)\n', obj.refNode, obj.nodes{i}.getId(), length(d_ij_arrays{i}));
                end
            end
            
            % reset the measurement history but not the dp index
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);
            obj.meascnt = 0;
            
            % set type 3 filtering to whatever it was before bootstrap
            obj.enableMessageType(obj.MSGTYPE3, type3Enabled);
            obj.enableMeasurementType(obj.MEASTYPE_R, typeREnabled);
        end
        
        % set all static nodes to their true positions
        function setStaticNodesToTruePosition(obj)
            for i=1:length(obj.nodes)
                if ~obj.nodes{i}.isMobile()
                    obj.nodes{i}.setPositionToTrue();
                end
            end
        end
        
        % =============================================
        %              MEASUREMENT HISTORY
        % =============================================
        
        % get pairwise TWR range measurements
        function r = getMeasurementTWR(obj, srcId, dstId)
            r = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    r = [r; meas.R_ij];
                end
            end
        end
        
        % get pairwise OWR range measurements
        function r = getMeasurementOWR(obj, srcId, dstId)
            r = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    r = [r; meas.r_ij];
                end
            end
        end
        
        % get pairwise offset measurements
        function [o,t] = getMeasurementOffsets(obj, srcId, dstId)
            o = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    o = [o; meas.d_ij];
                end
            end
        end
        
        % get all measurements between two devices
        function m = getMeasurements(obj, srcId, dstId)
            m = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    m = [m; meas];
                end
            end
        end
        
        % get pairwise measurement times
        function t = getMeasurementTimes(obj, srcId, dstId)
            t = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    t = [t; meas.getTime()];
                end
            end
        end
        
        % get all measurement times
        function t = getAllMeasurementTimes(obj)
            t = zeros(1,obj.meascnt);
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                t(i) = meas.getTime();
            end
        end
        
        % get allan deviation between two nodes
        function [ad,tau] = getAllanDev(obj, srcId, dstId, tau)
            % get non-zero aligned data
            data = obj.dataparser.aligned_logs;
            data = data(data(:,2) == srcId & data(:,3) == dstId, :);
            % get tx and rx
            tx = data(:,5);
            rx = data(:,6);
            % tx time (ignore 1st b/c y is diffed)
            x = tx(2:end);
            % rx dFreq in seconds
            y = (diff(rx) ./ diff(tx) - 1);
            [ad,~,~,tau] = allan(struct('freq',y,'time',x),tau,'',0);
        end
        
        % get all time offsets between two nodes
        function [o,t] = getAllOffsets(obj, srcId, dstId)
            data = obj.dataparser.aligned_logs;
            idxs = find(data(:,2) == srcId & data(:,3) == dstId);
            t = data(idxs,1);
            o = data(idxs,10) - data(idxs,9);
        end
        
        % get all time biases between two nodes
        function [b,t] = getAllBiases(obj, srcId, dstId)
            data = obj.dataparser.aligned_logs;
            idxs = find(data(:,2) == srcId & data(:,3) == dstId);
            t = data(idxs,1);
            o = data(idxs,10) - data(idxs,9);
            b = diff(o)./diff(t);
            t = t(2:end);
        end
        
        % =============================================
        %              POSITION ACCESSORS
        % =============================================
        function xyz = getTruePosition(obj, nodeId, tarray)
            xyz = [];
            for i=1:length(tarray)
                t = tarray(i);
                idx = obj.getNodeIdx( nodeId );
                if obj.nodes{idx}.isMobile()
                    rbid = obj.nodes{idx}.getRigidBodyId();
                    p = obj.dataparser.getMocapPos( rbid, t );
                else
                    p = obj.nodes{ idx }.getTruePosition()';
                end
                xyz = [xyz; p];
            end
        end
        
        function setRigidBodyId(obj, nodeId, rbId)
            nodeIdx = obj.getNodeIdx( nodeId );
            obj.nodes{nodeIdx}.setRigidBodyId( rbId );
        end
        
        function setSizebuffer_forall(obj,size_R)
            for i=1:length(obj.nodes)
                obj.nodes{i}.setSizebuffer(size_R);
            end
        end
        
        function init_x_P_forall(obj,x,P)
            for i=1:length(obj.nodes)
                obj.nodes{i}.init_x_P(x,P)
            end
        end

        function publishmeas_forall(obj,meas,h)
            for i=1:length(obj.nodes)
                obj.nodes{i}.set_meas(meas,h);
            end
        end
        
        
        function RunMeasFlag= checkekf_p1_forall(obj,dt_ref,meas)
            for i=1:length(obj.nodes)
                obj.nodes{i}.checkekf_p1();
            end
            RunMeasFlag = obj.MeasurementPCovUpdate(dt_ref,meas);
        end

        function RunMeasFlag= checkekf_p1_forallUnconn(obj)
            for i=1:length(obj.nodes)
                obj.nodes{i}.checkekf_p1();
            end
            mobileIndex =9;
            RunMeasFlag = obj.nodes{mobileIndex}.measUpdateFlag;
            obj.nodes{mobileIndex}.measUpdateFlag = 0;
        end
        
        
     

        function [RunDiffFlag,RunTimeFlag]= checkekf_p2_forall(obj,fstate,Q,diffEnable,dt_ref)
            for i=1:length(obj.nodes)
                obj.nodes{i}.checkekf_p2(fstate,Q,diffEnable);
            end
            RunDiffFlag = obj.DiffusionPCovUpdate();
            RunTimeFlag = obj.TimePCovUpdate(dt_ref);
        end
        
        
        function [RunDiffFlag,RunTimeFlag]= checkekf_p2_forallUnconn(obj,fstate,Q,diffEnable,dt_ref)
            for i=1:length(obj.nodes)
                obj.nodes{i}.checkekf_p2(fstate,Q,diffEnable);
            end
            mobileIndex =9;
             RunDiffFlag =obj.nodes{mobileIndex}.diffUpdateFlag ;
            RunTimeFlag = obj.nodes{mobileIndex}.timeUpdateFlag;
        end


        function publisheita_forall(obj)
            for i=1:length(obj.nodes)
                if(obj.nodes{i}.ready_to_ekf_p1 == 1) %eita is ready to publish
                    for j=1:length(obj.nodes)
                        obj.nodes{j}.seteital(i,obj.nodes{i}.eita,obj.nodes{i}.P);
                    end
                end
            end
        end
        
        function reseteita(obj)
            for i=1:length(obj.nodes)
                obj.nodes{i}.reseteital();
            end
        end
        %%%%%%%%%%%%%% network topology
        
        
        %set the neighbours for all
        function setneigh_forall(obj)
            for i=1:length(obj.nodes)
                obj.nodes{i}.setmyneigh(sort(obj.network{i}));
                obj.nodes{i}.setSizebuffer(length(obj.network{i}));
            end
        end
        
        function publishmeas_forneigh(obj,meas,h)
            srcIndex=meas.getSourceId() +1;
            desIndex=meas.getDestId() +1;
            
            %id 10 is number 9
            if srcIndex ==11
                srcIndex =9;
            end
            if desIndex ==11
                desIndex =9;
            end
            
%             %if srcIndex ==1
%             if any(obj.network{1}==srcIndex)
%                 ttt =6;
%             end
            %check if the measuremnt is between neighbours
            % if not just discard it
            if( any(obj.network{srcIndex}==desIndex) )
                for i =obj.network{srcIndex}
                    %send the need msg only to the nei
                    % the meas must be between nei of my nei
                    %if(any(obj.network{i}==srcIndex) && any(obj.network{i}==desIndex))
                    obj.nodes{i}.set_meas(meas,h);
                    if(i ~= srcIndex)% if it is to my self, do not count!
                        obj.sentmsgsM = obj.sentmsgsM +1;
                    end
                    %end
                end
            end
        end
        
        

        function calcPropDelaytoAlpha(obj)
            for i=1:length(obj.nodes)-1
                obj.propdelay2alpha(i)= norm( obj.nodes{1}.true_p- obj.nodes{i+1}.true_p )/obj.LIGHTSPEED;
            end
        end
        
        function publisheita_forneigh(obj)
            for i=1:length(obj.nodes)
                if(obj.nodes{i}.ready_to_ekf_p1 == 1 && obj.nodes{i}.eitaIsSent ==0) %eita is ready to publish
                    obj.nodes{i}.eitaIsSent =1;
                    for j = sort(obj.network{i})
                        %if(obj.nodes{j}.ready_to_ekf_p1==1)
                        obj.nodes{j}.seteital(i,obj.nodes{i}.eita,obj.nodes{i}.P);
                       
                        if(j ~= i)% if it is to my self, do not count!
                            obj.sentmsgsD = obj.sentmsgsD +1;
                        end
                        %end
                    end
                end
            end
        end
        
 
        
        function sendP(obj)
            P = obj.nodes{9}.P;
            for i =1:8
                P = P + obj.nodes{i}.P;
            end
            obj.nodes{9}.P = P./9;
        end
        
        function calcThreshold(obj,dt_ref)
            numofnodes=9;
            numofstates=5;
            M=numofnodes*numofstates;
            c_diffusion=1/numofnodes *ones(numofnodes,numofnodes);
            C_mat=kron(c_diffusion,eye(M));
            G = 1;
            P_mat = blkdiag(obj.nodes{1}.P,obj.nodes{2}.P,obj.nodes{3}.P,obj.nodes{4}.P,obj.nodes{5}.P,obj.nodes{6}.P,obj.nodes{7}.P,obj.nodes{8}.P,obj.nodes{9}.P);
            P_minus_mat = blkdiag(obj.nodes{1}.P_minus,obj.nodes{2}.P_minus,obj.nodes{3}.P_minus,obj.nodes{4}.P_minus,obj.nodes{5}.P_minus,obj.nodes{6}.P_minus,obj.nodes{7}.P_minus,obj.nodes{8}.P_minus,obj.nodes{9}.P_minus);
            X = [obj.nodes{1}.x,obj.nodes{2}.x,obj.nodes{3}.x,obj.nodes{4}.x,obj.nodes{5}.x,obj.nodes{6}.x,obj.nodes{7}.x,obj.nodes{8}.x,obj.nodes{9}.x];
            fstate= @(X) processFcndp(s, dt_ref); %dp proecessFcn
            h = @(X) measurementFcndp(s, meas);   %dp measurementFcn
            [f, F_bar]= jaccsd(fstate,X);
            L = ones(numofnodes,numofnodes);
            L_latin=kron(L,eye(M));
            A = C_mat' * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F_bar)),F_bar);
            B = C_mat' * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F_bar)),G);
            % D = C_mat' * P_mat * L_latin' *
            
        end
        
        
        function P=getCovWithXerror(obj,meas,P_old,dt_ref,walltime)
            %If A is an m × n matrix and B is a p × q matrix,
            %then the Kronecker product A ? B is the mp × nq block matrix
            numofnodes=9;
            numofstates=5;
            M=numofnodes*numofstates;
            c_diffusion=1/numofnodes *ones(numofnodes,numofnodes);
            C_mat=kron(c_diffusion,eye(M));
            G = 1;
            P_mat = blkdiag(obj.nodes{1}.P_i_i,obj.nodes{2}.P_i_i,obj.nodes{3}.P_i_i,obj.nodes{4}.P_i_i,obj.nodes{5}.P_i_i,obj.nodes{6}.P_i_i,obj.nodes{7}.P_i_i,obj.nodes{8}.P_i_i,obj.nodes{9}.P_i_i);
            P_minus_mat = blkdiag(obj.nodes{1}.P_minus,obj.nodes{2}.P_minus,obj.nodes{3}.P_minus,obj.nodes{4}.P_minus,obj.nodes{5}.P_minus,obj.nodes{6}.P_minus,obj.nodes{7}.P_minus,obj.nodes{8}.P_minus,obj.nodes{9}.P_minus);
            
            L = ones(numofnodes,numofnodes);
            L_latin=kron(L,eye(M));
            
            % get the true positions X_true
            rb=1;
            [xyz_mobile, latency] = obj.dataparser.getMocapPos(rb, walltime);
            
            x_true = [];
            for i=1:length(obj.nodes)
                if ~obj.nodes{i}.isMobile()
                    x_true = [x_true;  obj.nodes{i}.getTruePosition(); obj.nodes{1}.state_clkofst; obj.nodes{1}.state_clkbias];
                else
                    x_true = [x_true;  xyz_mobile'; obj.nodes{1}.state_clkofst; obj.nodes{1}.state_clkbias];
                end
            end
            % X = [obj.nodes{1}.x_i_i;obj.nodes{2}.x_i_i;obj.nodes{3}.x_i_i;obj.nodes{4}.x_i_i;obj.nodes{5}.x_i_i;obj.nodes{6}.x_i_i;obj.nodes{7}.x_i_i;obj.nodes{8}.x_i_i;obj.nodes{8}.x_i_i];
            X = [x_true-obj.nodes{1}.x_i_i;x_true-obj.nodes{2}.x_i_i;x_true-obj.nodes{3}.x_i_i;x_true-obj.nodes{4}.x_i_i;x_true-obj.nodes{5}.x_i_i;x_true-obj.nodes{6}.x_i_i;x_true-obj.nodes{7}.x_i_i;x_true-obj.nodes{8}.x_i_i;x_true-obj.nodes{9}.x_i_i];
            fstate= @(X) obj.processExtendedFcn(X, dt_ref);
            [f, F]= obj.jaccsd(fstate,X);
            
            
            for i=1:obj.getNumNodes()
                x=x_true-obj.nodes{i}.x_i_i;
                h = @(x) obj.measurementFcndp(x, meas);
                [ h_, H_{i}]= obj.jaccsd(h,x);
            end
            % Convert H_{i} to H_vector
            %H_vector = [ H_{1}', H_{2}', H_{3}', H_{4}', H_{5}', H_{6}', H_{7}', H_{8}', H_{9}' ];
            %H = diag(H_vector);
            H= blkdiag( H_{1}, H_{2}, H_{3}, H_{4}, H_{5}, H_{6}, H_{7}, H_{8}, H_{9});
            
            R_single = meas.getCovariance();
            R= blkdiag(R_single,R_single,R_single,R_single,R_single,R_single,R_single,R_single,R_single);
            A = C_mat' * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F)),F);
            B = C_mat' * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F)),G);
            D = C_mat' * P_mat * L_latin' * H' * R^-1;
            onesVec=ones(numofnodes,1);
            Q = obj.getProcessVar();
            P = A*P_old*A' + B* kron(onesVec*onesVec',Q)*B'+D*R*D';
            
        end
        
        function P=getCovStatic(obj,meas,P_old,dt_ref)
            %If A is an m × n matrix and B is a p × q matrix,
            %then the Kronecker product A ? B is the mp × nq block matrix
            numofnodes=8;
            numofstates=5;
            M=numofnodes*numofstates;
            c_diffusion=1/numofnodes *ones(numofnodes,numofnodes);
            C_mat=kron(c_diffusion,eye(M));
            G = 1;
            P_mat = blkdiag(obj.nodes{1}.P_i_i,obj.nodes{2}.P_i_i,obj.nodes{3}.P_i_i,obj.nodes{4}.P_i_i,obj.nodes{5}.P_i_i,obj.nodes{6}.P_i_i,obj.nodes{7}.P_i_i,obj.nodes{8}.P_i_i);
            P_minus_mat = blkdiag(obj.nodes{1}.P_minus,obj.nodes{2}.P_minus,obj.nodes{3}.P_minus,obj.nodes{4}.P_minus,obj.nodes{5}.P_minus,obj.nodes{6}.P_minus,obj.nodes{7}.P_minus,obj.nodes{8}.P_minus);
            
            L = ones(numofnodes,numofnodes);
            L_latin=kron(L,eye(M));
            
            
            X = [obj.nodes{1}.x_i_i;obj.nodes{2}.x_i_i;obj.nodes{3}.x_i_i;obj.nodes{4}.x_i_i;obj.nodes{5}.x_i_i;obj.nodes{6}.x_i_i;obj.nodes{7}.x_i_i;obj.nodes{8}.x_i_i];
            fstate= @(X) obj.processExtendedFcn(X, dt_ref);
            [f, F]= obj.jaccsd(fstate,X);
            
            for i=1:obj.getNumNodes()
                x=obj.nodes{i}.x_i_i;
                h = @(x) obj.measurementFcndp(x, meas);
                [ h_, H_{i}]= obj.jaccsd(h,x);
            end
            % Convert H_{i} to H_vector
            %H_vector = [ H_{1}', H_{2}', H_{3}', H_{4}', H_{5}', H_{6}', H_{7}', H_{8}', H_{9}' ];
            %H = diag(H_vector);
            H= blkdiag( H_{1}, H_{2}, H_{3}, H_{4}, H_{5}, H_{6}, H_{7}, H_{8});
            
            R_single = meas.getCovariance();
            R= blkdiag(R_single,R_single,R_single,R_single,R_single,R_single,R_single,R_single);
            A = C_mat' * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F)),F);
            B = C_mat' * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F)),G);
            D = C_mat' * P_mat * L_latin' * ctranspose(H) * R^-1;
            onesVec=ones(numofnodes,1);
            Q = obj.getProcessVar();
            P = A*P_old*ctranspose(A) + B* kron(onesVec*ctranspose(onesVec),Q)*ctranspose(B)+D*R*ctranspose(D);
            
        end
        
        function P=getCovStaticTimeUpdate(obj,P_old,dt_ref)
            %If A is an m × n matrix and B is a p × q matrix,
            %then the Kronecker product A ? B is the mp × nq block matrix
            numofnodes=8;
            numofstates=5;
            M=numofnodes*numofstates;
            X = [obj.nodes{1}.x_i_i;obj.nodes{2}.x_i_i;obj.nodes{3}.x_i_i;obj.nodes{4}.x_i_i;obj.nodes{5}.x_i_i;obj.nodes{6}.x_i_i;obj.nodes{7}.x_i_i;obj.nodes{8}.x_i_i];
            fstate= @(X) obj.processExtendedFcn(X, dt_ref);
            [f, F]= obj.jaccsd(fstate,X);
            onesVec=ones(numofnodes,1);
            Q = obj.getProcessVar();
            G=eye(numofnodes*numofstates);
            % FF = kron(eye(numofnodes),F);
            
            
            %GG = kron(onesVec,G);
            GG = kron(onesVec,G);
            P = F*P_old*ctranspose(F) + GG*Q*ctranspose(GG);
            
        end
        

        function P=getCov(obj,meas,P_old,dt_ref)
            %If A is an m × n matrix and B is a p × q matrix,
            %then the Kronecker product A ? B is the mp × nq block matrix
            numofnodes=9;
            numofstates=5;
            M=numofnodes*numofstates;
            c_diffusion=1/numofnodes *ones(numofnodes,numofnodes);
            C_mat=kron(c_diffusion,eye(M));
            G = 1;
            P_mat = blkdiag(obj.nodes{1}.P_i_i,obj.nodes{2}.P_i_i,obj.nodes{3}.P_i_i,obj.nodes{4}.P_i_i,obj.nodes{5}.P_i_i,obj.nodes{6}.P_i_i,obj.nodes{7}.P_i_i,obj.nodes{8}.P_i_i,obj.nodes{9}.P_i_i);
            P_minus_mat = blkdiag(obj.nodes{1}.P_minus,obj.nodes{2}.P_minus,obj.nodes{3}.P_minus,obj.nodes{4}.P_minus,obj.nodes{5}.P_minus,obj.nodes{6}.P_minus,obj.nodes{7}.P_minus,obj.nodes{8}.P_minus,obj.nodes{9}.P_minus);
            
            L = ones(numofnodes,numofnodes);
            L_latin=kron(L,eye(M));
            
            X = [obj.nodes{1}.x_i_i;obj.nodes{2}.x_i_i;obj.nodes{3}.x_i_i;obj.nodes{4}.x_i_i;obj.nodes{5}.x_i_i;obj.nodes{6}.x_i_i;obj.nodes{7}.x_i_i;obj.nodes{8}.x_i_i;obj.nodes{9}.x_i_i];
            fstate= @(X) obj.processExtendedFcn(X, dt_ref);
            [f, F]= obj.jaccsd(fstate,X);
            
            for i=1:obj.getNumNodes()
                x=obj.nodes{i}.x_i_i;
                h = @(x) obj.measurementFcndp(x, meas);
                [ h_, H_{i}]= obj.jaccsd(h,x);
            end
            % Convert H_{i} to H_vector
            %H_vector = [ H_{1}', H_{2}', H_{3}', H_{4}', H_{5}', H_{6}', H_{7}', H_{8}', H_{9}' ];
            %H = diag(H_vector);
            H= blkdiag( H_{1}, H_{2}, H_{3}, H_{4}, H_{5}, H_{6}, H_{7}, H_{8}, H_{9});
            
            R_single = meas.getCovariance();
            R= blkdiag(R_single,R_single,R_single,R_single,R_single,R_single,R_single,R_single,R_single);
            A = ctranspose(C_mat) * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F)),F);
            B = ctranspose(C_mat)  * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F)),G);
            D = ctranspose(C_mat)  * P_mat * L_latin' * H' * R^-1;
            onesVec=ones(numofnodes,1);
            Q = obj.getProcessVar();
            P = A*P_old*ctranspose(A) + B* kron(onesVec*ctranspose(onesVec),Q)*ctranspose(B)+D*R*ctranspose(D);
            
        end
        
        function P=getCovMobileTimeUpdate(obj,P_old,dt_ref)
            %If A is an m × n matrix and B is a p × q matrix,
            %then the Kronecker product A ? B is the mp × nq block matrix
            numofnodes=9;
            numofstates=5;
            M=numofnodes*numofstates;
            X = [obj.nodes{1}.x_i_i;obj.nodes{2}.x_i_i;obj.nodes{3}.x_i_i;obj.nodes{4}.x_i_i;obj.nodes{5}.x_i_i;obj.nodes{6}.x_i_i;obj.nodes{7}.x_i_i;obj.nodes{8}.x_i_i;obj.nodes{9}.x_i_i];
            fstate= @(X) obj.processExtendedFcn(X, dt_ref);
            [f, F]= obj.jaccsd(fstate,X);
            onesVec=ones(numofnodes,1);
            Q = obj.getProcessVar();
            G=eye(numofnodes*numofstates);
            % FF = kron(eye(numofnodes),F);
            
            
            %GG = kron(onesVec,G);
            GG = kron(onesVec,G);
            P = F*P_old*ctranspose(F) + GG*Q*ctranspose(GG);
            
        end
        
        function P=getCovStaticWithXerror(obj,meas,P_old,dt_ref)
            %If A is an m × n matrix and B is a p × q matrix,
            %then the Kronecker product A ? B is the mp × nq block matrix
            numofnodes=8;
            numofstates=5;
            M=numofnodes*numofstates;
            c_diffusion=1/numofnodes *ones(numofnodes,numofnodes);
            C_mat=kron(c_diffusion,eye(M));
            G = 1;
            P_mat = blkdiag(obj.nodes{1}.P_i_i,obj.nodes{2}.P_i_i,obj.nodes{3}.P_i_i,obj.nodes{4}.P_i_i,obj.nodes{5}.P_i_i,obj.nodes{6}.P_i_i,obj.nodes{7}.P_i_i,obj.nodes{8}.P_i_i);
            P_minus_mat = blkdiag(obj.nodes{1}.P_minus,obj.nodes{2}.P_minus,obj.nodes{3}.P_minus,obj.nodes{4}.P_minus,obj.nodes{5}.P_minus,obj.nodes{6}.P_minus,obj.nodes{7}.P_minus,obj.nodes{8}.P_minus);
            
            L = ones(numofnodes,numofnodes);
            L_latin=kron(L,eye(M));
            
            % get the true positions X_true
            x_true = [];
            for i=1:length(obj.nodes)
                if ~obj.nodes{i}.isMobile()
                    x_true = [x_true;  obj.nodes{i}.getTruePosition(); obj.nodes{1}.state_clkofst; obj.nodes{1}.state_clkbias];
                end
            end
            % X = [obj.nodes{1}.x_i_i;obj.nodes{2}.x_i_i;obj.nodes{3}.x_i_i;obj.nodes{4}.x_i_i;obj.nodes{5}.x_i_i;obj.nodes{6}.x_i_i;obj.nodes{7}.x_i_i;obj.nodes{8}.x_i_i];
            X = [x_true-obj.nodes{1}.x_i_i;x_true-obj.nodes{2}.x_i_i;x_true-obj.nodes{3}.x_i_i;x_true-obj.nodes{4}.x_i_i;x_true-obj.nodes{5}.x_i_i;x_true-obj.nodes{6}.x_i_i;x_true-obj.nodes{7}.x_i_i;x_true-obj.nodes{8}.x_i_i];
            fstate= @(X) obj.processExtendedFcn(X, dt_ref);
            [f, F]= obj.jaccsd(fstate,X);
            
            for i=1:obj.getNumNodes()
                x=obj.nodes{i}.x_i_i;
                h = @(x) obj.measurementFcndp(x, meas);
                [ h_, H_{i}]= obj.jaccsd(h,x);
            end
            % Convert H_{i} to H_vector
            %H_vector = [ H_{1}', H_{2}', H_{3}', H_{4}', H_{5}', H_{6}', H_{7}', H_{8}', H_{9}' ];
            %H = diag(H_vector);
            H= blkdiag( H_{1}, H_{2}, H_{3}, H_{4}, H_{5}, H_{6}, H_{7}, H_{8});
            
            R_single = meas.getCovariance();
            R= blkdiag(R_single,R_single,R_single,R_single,R_single,R_single,R_single,R_single);
            A = C_mat' * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F)),F);
            B = C_mat' * P_mat * (P_minus_mat)^-1 * kron(eye(length(P_minus_mat)/length(F)),G);
            D = C_mat' * P_mat * L_latin' * H' * R^-1;
            onesVec=ones(numofnodes,1);
            Q = obj.getProcessVar();
            P = A*P_old*A' + B* kron(onesVec*onesVec',Q)*B'+D*R*D';
            
        end
        
        
        
        
        function snew = processExtendedFcn(obj, s, dt)
            snew = s;
            
            % perform process update over each node separately
%             stateSize = 3;
%             i = 1;
%             for n=1:stateSize:length(s)
%                 snew(i+3) = s(i+3) + s(i+4) * 1e-9 * dt;
%                 i = i + stateSize;
%             end
        end
        % state measurement function
        function y = measurementFcnExtended(obj, s, meas)
            type = meas.getType();
            
            srcIdx = obj.getNodeIdx(meas.getSourceId());
            dstIdx = obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            stateSize = 5;
            start_i = (srcIdx-1)*stateSize + 1;
            start_j = (dstIdx-1)*stateSize + 1;
            
            % extract the needed state info
            pi = s(start_i:(start_i+2));
            pj = s(start_j:(start_j+2));
            oi = s(start_i+3);
            oj = s(start_j+3);
            bi = s(start_i+4);
            bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];
            y = [];
            
            c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
            if obj.filter_meas_d
                if obj.enable_slats
                    y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
                else
                    y = [y; (oj - oi)];
                end
            end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
            if obj.filter_meas_r
                if obj.enable_slats
                    y = [y; sqrt(sum((pj-pi).^2)) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
                else
                    y = [y; sqrt(sum((pj-pi).^2))];
                end
            end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
                    err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    y = [y; sqrt(sum((pj-pi).^2)) + err ];
                else
                    y = [y; sqrt(sum((pj-pi).^2))];
                end
            end
        end
        
        function RunTimeFlag=ekfPartThreeOnlyForAll(obj,Q,fstate,dt_ref)
            for i=1:length(obj.nodes)
                obj.nodes{i}.ekf_part3_Only(Q,fstate);
            end
            RunTimeFlag=obj.TimePCovUpdate(dt_ref);
        end
          
                       
        function RunTimeFlag=ekfPartThreeOnlyForAllUncon(obj,Q,fstate)
            for i=1:length(obj.nodes)
                obj.nodes{i}.ekf_part3_Only(Q,fstate);
            end
            mobileIndex =9;
            RunTimeFlag=obj.nodes{mobileIndex}.timeUpdateFlag ==1;
            %reset the node flag
            obj.nodes{mobileIndex}.timeUpdateFlag =0;
            
        end
        
        function RunFlag= MeasurementPCovUpdate(obj,dt_ref,meas)
            iamReady=1;
            for i=1:length(obj.nodes)
                if obj.nodes{i}.measUpdateFlag ==0
                    iamReady=0;
                end
            end
            RunFlag = 0;
            if iamReady==1
                RunFlag = 1;
                numofnodes=9;
                numofstates=5;
                M=numofnodes*numofstates;
                G = 1;
                P_mat = blkdiag(obj.nodes{1}.P_i_i,obj.nodes{2}.P_i_i,obj.nodes{3}.P_i_i,obj.nodes{4}.P_i_i,obj.nodes{5}.P_i_i,obj.nodes{6}.P_i_i,obj.nodes{7}.P_i_i,obj.nodes{8}.P_i_i,obj.nodes{9}.P_i_i);
                P_minus_mat = blkdiag(obj.nodes{1}.P_minus,obj.nodes{2}.P_minus,obj.nodes{3}.P_minus,obj.nodes{4}.P_minus,obj.nodes{5}.P_minus,obj.nodes{6}.P_minus,obj.nodes{7}.P_minus,obj.nodes{8}.P_minus,obj.nodes{9}.P_minus);
                
                L = ones(numofnodes,numofnodes);
                L_latin=kron(L,eye(M));
                
                X = [obj.nodes{1}.eita;obj.nodes{2}.eita;obj.nodes{3}.eita;obj.nodes{4}.eita;obj.nodes{5}.eita;obj.nodes{6}.eita;obj.nodes{7}.eita;obj.nodes{8}.eita;obj.nodes{9}.eita];
                fstate= @(X) obj.processExtendedFcn(X, dt_ref);
                [f, F]= obj.jaccsd(fstate,X);
                
                for i=1:obj.getNumNodes()
                    x=obj.nodes{i}.eita;
                    h = @(x) obj.measurementFcndp(x, meas);
                    [ h_, H_{i}]= obj.jaccsd(h,x);
                end
                % Convert H_{i} to H_vector
                %H_vector = [ H_{1}', H_{2}', H_{3}', H_{4}', H_{5}', H_{6}', H_{7}', H_{8}', H_{9}' ];
                %H = diag(H_vector);
                H= blkdiag( H_{1}, H_{2}, H_{3}, H_{4}, H_{5}, H_{6}, H_{7}, H_{8}, H_{9});
                
                R_single = meas.getCovariance();
                R= blkdiag(R_single,R_single,R_single,R_single,R_single,R_single,R_single,R_single,R_single);
               
                CovTermOne = P_mat * P_minus_mat^-1 * obj.cov_big * P_minus_mat^-1 *P_mat ;
                CovTermTwo = P_mat  * L_latin' * ctranspose(H) * R ^-1 * H * L_latin * P_mat;
               
                obj.cov_big =  CovTermOne + CovTermTwo;
                
                obj.P_big = P_mat;
                
                
                for i=1:length(obj.nodes)
                    obj.nodes{i}.measUpdateFlag =0;   
                end
                
            end
        end
        
        function RunFlag=DiffusionPCovUpdate(obj)
            iamReady=1;%assume they are ready
            for i=1:length(obj.nodes)
                if obj.nodes{i}.diffUpdateFlag ==0
                    iamReady=0; % some one is not ready :(
                end
            end
            RunFlag=0;
            if iamReady ==1
                RunFlag=1;
                numofnodes=9;
                numofstates=5;
                M=numofnodes*numofstates;
                c_diffusion=1/numofnodes *ones(numofnodes,numofnodes);
                C_mat=kron(c_diffusion,eye(M));
                %% no change in P_big
                %% update the cov
                obj.cov_big = C_mat'*obj.cov_big*C_mat;
                for i=1:length(obj.nodes)
                    obj.nodes{i}.diffUpdateFlag =0;
                end
            end
        
        end
      
        
        function RunFlag=TimePCovUpdate(obj,dt_ref)
            iamReady=1;%assume they are ready
            for i=1:length(obj.nodes)
                if obj.nodes{i}.timeUpdateFlag ==0
                    iamReady=0; % some one is not ready :(
                end
            end
            RunFlag=0;
            if iamReady ==1
                RunFlag=1;
                numofnodes=9;
                numofstates=5;
                M=numofnodes*numofstates;
                X = [obj.nodes{1}.x_i_i;obj.nodes{2}.x_i_i;obj.nodes{3}.x_i_i;obj.nodes{4}.x_i_i;obj.nodes{5}.x_i_i;obj.nodes{6}.x_i_i;obj.nodes{7}.x_i_i;obj.nodes{8}.x_i_i;obj.nodes{9}.x_i_i];
                fstate= @(X) obj.processExtendedFcn(X, dt_ref);
                [f, F]= obj.jaccsd(fstate,X);
                onesVec=ones(numofnodes,1);
                Q = obj.getProcessVar();
                G=eye(numofnodes*numofstates);
                GG = kron(onesVec,G);
                obj.cov_big = F*obj.cov_big * ctranspose(F) + GG*Q*ctranspose(GG);
                obj.P_big = F*obj.P_big * ctranspose(F) + GG*Q*ctranspose(GG);
                for i=1:length(obj.nodes)
                    obj.nodes{i}.timeUpdateFlag =0;
                end
            end
        end
        
        %dp-start %Dawoud
        function setMeasParameters(obj,meas,pi,pj,zm,zdp)
            
            walltime = meas.getTime();
%dp             R = meas.getCovariance();
            [xyz, latency] = obj.dataparser.getMocapPos(1, walltime);
            obj.nodes{9}.true_p = xyz';
%dp-start             if meas.nodei==10
%             obj.nodes{9}.x_zonotope = zonotope(pi,diag([R;R;R]));
%             %obj.nodes{meas.nodej+1}.x_zonotope = zonotope(pj,diag([R;R;R]));
%             else
%             obj.nodes{9}.x_zonotope = zonotope(pj,diag([R;R;R]));
%             %obj.nodes{meas.nodei+1}.x_zonotope = zonotope(pi,diag([R;R;R]));
% 
%dp-end             end
            
            srcIndex=meas.getSourceId();
            if srcIndex==10
%                 for i =unique(obj.network{srcIndex-1}) %obj.network{srcIndex}
%                 j=i;
                %send the need msg only to the nei
                obj.nodes{srcIndex-1}.set_meas(meas,pi,pj,zm,zdp);
%                 end
            
            else 
%                 for i =unique(obj.network{srcIndex+1}) %obj.network{srcIndex}
%                 j=i;
                %send the need msg only to the nei
                obj.nodes{srcIndex+1}.set_meas(meas,pi,pj,zm,zdp);
%                 end
            end
%             for i =unique(obj.network{srcIndex}) %obj.network{srcIndex}
%                 j=i;
%                 %send the need msg only to the nei
%                 obj.nodes{i}.set_meas(meas,h);
%             end
        end
        function check_p1(obj,diffEnable,fstate,algorithm,method,Q)
            for i=1:length(obj.nodes)%dp
                obj.nodes{i}.check_p1(diffEnable,fstate,algorithm,method,Q);
            end
        end
        
        function Rout =  NonlinearEstimator(obj,R0,s,f,meas,pi,pj,zm,zdp,method,dt)
            % has no effect if the h does not have u
            Rm=meas.getCovariance();
            y=meas.vectorize();
            if meas.nodei==10
            optionsh.U = zonotope(pj);
            elseif meas.nodej==10
            %+ options.uTrans;
            optionsh.U = zonotope(pi);
            end
            optionsh.p.u = center(optionsh.U);%center(optionsh.U) ;
            %linearization point p.x and p.y
            optionsh.p.x = center(R0);
            optionsh.pi = pi;
            optionsh.pj = pj;
            optionsh.var = y-zm;
            optionsh.hl = (((pi-pj).*(y-zm))/zm);
            optionsh.Rl = [Rm;Rm;Rm]; 
            optionsh.meas = meas;
            optionsh.s = s;
            optionsh.f = f;
            optionsh.zm = zm;
            optionsh.zdp = zdp;
            optionsh.method= method;
            % dimension of x
            optionsh.dim_x=3;
            optionsh.dim_u=3;
            optionsh.dim_f=1;
            %optionsh.method='radius';%'svd';%'radius';%'normGen',volume;
            optionsh.reductionTechnique = 'girard';
            
            % Reachability Settings
            optionsh.zonotopeOrder = 100;
            optionsh.tensorOrder = 1;
            optionsh.errorOrder =100;
            %dt = 0.05; %not used
            %             str = ['hdynamics = @(x,u)', benchmark, '(x,u);'];
            %             eval(str);
            %dp if t_last<=4.2
                
               %dp h{1} = [0 1];
               %dp r{1} =1;
               %dp y{1}= 2.25;
               %dp fimplicit(@(x11,x22) abs(h{1}*[x11;x22] - y{1})-r{1},'-.r')
                
                %Rout =intersectConZonoStrip1(conZonotope(R0),h,r,y);
                %dp Rout = intersectZonoStrip(R0,h,r,y);                
                
           %dp elseif t_last<= 10.5  
                
                optionsh.dim_h=1;
                optionsh.fun =@(x,u) hNorm(x,u);
                hDisc = nonlinearDT('hNorm',optionsh.fun,dt,optionsh.dim_x,optionsh.dim_u,1);
                %a=[0.3225;0.2175];
                %a=[0.09];
                a=[1.429319]^2;
%                 fimplicit(@(x,u) abs(hNorm(optionsh.pi,optionsh.pj))-a,'-.b')
                %Rout = intersectNonLinearH(hDisc,R0,optionsh);
                Rout=intersectZonoZono(R0,meas.getmeasurementzonotope,meas.getdpzonotope,optionsh.hl,optionsh.Rl,meas.R_ij,optionsh.method);
               % 
%                 optionsh.fun =@(x,u) hNonLinear12(x,u);
%                 hDisc = nonlinearDT('hnon12',optionsh.fun,dt,optionsh.dim_x,2,1);
%                 %a=[0.3225;0.2175];
%                 %a=[0.09];
%                 a=[0.2175];
%                 Rout = intersectCZNonLinearH(hDisc,a,Rout1,optionsh);
               % [Rout]= intersectNonLinearH(hDisc,a,R0,optionsh);
           %dp else%if %t_last<=29
           %dp     h{1} = [0 1];
           %dp     r{1} = 1;
           %dp     y{1} = -2.1694;
           %dp      fimplicit(@(x11,x22) abs(h{1}*[x11;x22] - y{1})-r{1},'-.r')
                
                %Rout =intersectConZonoStrip1(conZonotope(R0),h,r,y);
           %dp     Rout = intersectZonoStrip(R0,h,r,y);  
%             else
%                 Rout = R0;
           %dp end

        end
        
        
        function setMeasParametersCentralized(obj,meas,pi,pj,zm,zdp)
            
            walltime = meas(1);
%dp             R = meas.getCovariance();
            [xyz, latency] = obj.dataparser.getMocapPos(1, walltime);
            obj.nodes{9}.true_p = xyz';
%dp-start             if meas.nodei==10
%             obj.nodes{9}.x_zonotope = zonotope(pi,diag([R;R;R]));
%             %obj.nodes{meas.nodej+1}.x_zonotope = zonotope(pj,diag([R;R;R]));
%             else
%             obj.nodes{9}.x_zonotope = zonotope(pj,diag([R;R;R]));
%             %obj.nodes{meas.nodei+1}.x_zonotope = zonotope(pi,diag([R;R;R]));
% 
%dp-end             end
            
            srcIndex= meas(2);
            if srcIndex==10
%                 for i =unique(obj.network{srcIndex-1}) %obj.network{srcIndex}
%                 j=i;
                %send the need msg only to the nei
                obj.nodes{srcIndex-1}.set_meas(meas,pi,pj,zm,zdp);
%                 end
            
            else 
%                 for i =unique(obj.network{srcIndex+1}) %obj.network{srcIndex}
%                 j=i;
                %send the need msg only to the nei
                obj.nodes{srcIndex+1}.set_meas(meas,pi,pj,zm,zdp);
%                 end
            end
%             for i =unique(obj.network{srcIndex}) %obj.network{srcIndex}
%                 j=i;
%                 %send the need msg only to the nei
%                 obj.nodes{i}.set_meas(meas,h);
%             end
        end

        
        
        %dp-end
    end
end