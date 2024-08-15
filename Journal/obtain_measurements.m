%start-dp
meas11 = nm.getNextMeasurement();
raw_measurements=[meas11.getTime() meas11.nodei meas11.nodej meas11.R_ij];

while(nm.msgidx < nm.dataparser.getNumMeasurements())
    measq = nm.getNextMeasurement();
    raw_measurements=[raw_measurements; measq.getTime() measq.nodei measq.nodej measq.R_ij];

end
 zm = zonotope([0],[.01 0.02]); %dp
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
            p1 = randPoint(zm); %dp
            %dp-start-measurement-noise-end
            
            %dp-laplace-noise-start
            r_Laplace1 = rand;
            ind_Laplace1 = find(r_Laplace1>cp_Laplace, 1, 'last');
            resultdp_Laplace1 = x_m(ind_Laplace1);
            %dp-laplace-noise-end
            temp = raw_measurements(i,4)+p1+resultdp_Laplace1;
            noisy_measurements = [noisy_measurements; raw_measurements(i,1) raw_measurements(i,2) raw_measurements(i,3) temp];
end
diffEnable = 1; % enable diffusion
nm.resetMeasurements();
%end-dp
