x_axis= 1:1:1443;
LDP_laplace_noise_error_table = readtable("logs/LDP_Estimation_error_laplace_noise.xlsx","ReadVariableNames",true); 
CDP_laplace_noise_error_table = readtable("logs/CDP_Estimation_error_laplace_noise.xlsx","ReadVariableNames",true);
no_noise_error_table = readtable("logs/NoDP_Estimation_error.xlsx","ReadVariableNames",true);
CDP_laplace_noise_error_array = table2array(CDP_laplace_noise_error_table(:,4));
LDP_laplace_noise_error_array = table2array(LDP_laplace_noise_error_table(:,4));
no_noise_error_array = table2array(no_noise_error_table(:,4));
no_noise_error_mean = sum(no_noise_error_array)/length(no_noise_error_array);
CDP_laplace_noise_error_mean = sum(CDP_laplace_noise_error_array)/length(CDP_laplace_noise_error_array);
LDP_laplace_noise_error_mean = sum(LDP_laplace_noise_error_array)/length(LDP_laplace_noise_error_array);

no_noise_data=plot(x_axis',no_noise_error_array,'color',[0.3010, 0.7450, 0.9330]);
hold on
CDP_laplace_noise_data=plot(x_axis',CDP_laplace_noise_error_array,'color',[0.9290, 0.6940, 0.1250]);
hold on
LDP_laplace_noise_data=plot(x_axis',LDP_laplace_noise_error_array,'color',[0.8500, 0.3250, 0.0980]);
xlabel('Estimation Iterations - K', 'FontSize',10);
ylabel({'Error in Estimated Location (m)'; '||X_{error},Y_{error},Z_{error}||_{2}'}, 'FontSize',10);
htitle = title('Error in Estimated Location','FontSize',16);
no_noise_txt=sprintf('Error without using the DP-noise "Mean=%0.4f (m)"',no_noise_error_mean);
CDP_laplace_noise_txt=sprintf('Error using the DP-noise (Trunc. Laplace) CDP "Mean=%0.4f (m)"',CDP_laplace_noise_error_mean);
LDP_laplace_noise_txt=sprintf('Error using the DP-noise (Trunc. Laplace) LDP "Mean=%0.4f (m)"',LDP_laplace_noise_error_mean);
legend([no_noise_data CDP_laplace_noise_data LDP_laplace_noise_data],{no_noise_txt,CDP_laplace_noise_txt,LDP_laplace_noise_txt}, 'Orientation', 'vertical', 'Location', 'NE');
txt = ['Average Error without adding the DP-noise= ' num2str(no_noise_error_mean) '(m)'];
%txt.Color=[0.3010, 0.7450, 0.9330];
% x = [0.6 0.5];
% y = [0.6 0.5];
% a=annotation('textbox',x,'String','y = x ','FitBoxToText','on');
% text(100,1.75,txt)
% txt = ['Average Error using the truncated CDP_laplace distribution DP-noise= ' num2str(CDP_laplace_noise_error_mean) '(m)'];
% text(100,1.65,txt)
% txt = ['Average Error using the truncated Laplace distribution DP-noise= ' num2str(LDP_laplace_noise_error_mean) '(m)'];
% text(100,1.55,txt)