x_axis= 1:1:1443;
laplace_noise_error_table = readtable("logs/LDP_Estimation_error_laplace_noise.xlsx","ReadVariableNames",true); 
optimal_noise_error_table = readtable("logs/LDP_Estimation_error_optimal_noise.xlsx","ReadVariableNames",true);
no_noise_error_table = readtable("logs/NoDP_Estimation_error.xlsx","ReadVariableNames",true);
opt_noise_error_array = table2array(optimal_noise_error_table(:,4));
laplace_noise_error_array = table2array(laplace_noise_error_table(:,4));
no_noise_error_array = table2array(no_noise_error_table(:,4));
no_noise_error_mean = sum(no_noise_error_array)/length(no_noise_error_array);
opt_noise_error_mean = sum(opt_noise_error_array)/length(opt_noise_error_array);
laplace_noise_error_mean = sum(laplace_noise_error_array)/length(laplace_noise_error_array);

no_noise_data=plot(x_axis',no_noise_error_array,'color',[0.3010, 0.7450, 0.9330]);
hold on
opt_noise_data=plot(x_axis',opt_noise_error_array,'color',[0.9290, 0.6940, 0.1250]);
hold on
laplace_noise_data=plot(x_axis',laplace_noise_error_array,'color',[0.8500, 0.3250, 0.0980]);
xlabel('Estimation Iterations - K', 'FontSize',10);
ylabel({'Error in Estimated Location (m)'; '||X_{error},Y_{error},Z_{error}||_{2}'}, 'FontSize',10);
%htitle = title('Error in Estimated Location','FontSize',16);
no_noise_txt=sprintf('Error without using the DP-noise "Mean=%0.4f (m)"',no_noise_error_mean);
opt_noise_txt=sprintf('Error using the DP-noise (Trunc. Optimal) LDP "Mean=%0.4f (m)"',opt_noise_error_mean);
laplace_noise_txt=sprintf('Error using the DP-noise (Trunc. Laplace) LDP "Mean=%0.4f (m)"',laplace_noise_error_mean);
legend([no_noise_data opt_noise_data laplace_noise_data],{no_noise_txt,opt_noise_txt,laplace_noise_txt}, 'Orientation', 'vertical', 'Location', 'NE');
txt = ['Average Error without adding the DP-noise= ' num2str(no_noise_error_mean) '(m)'];
%txt.Color=[0.3010, 0.7450, 0.9330];
% x = [0.6 0.5];
% y = [0.6 0.5];
% a=annotation('textbox',x,'String','y = x ','FitBoxToText','on');
% text(100,1.75,txt)
% txt = ['Average Error using the truncated optimal distribution DP-noise= ' num2str(opt_noise_error_mean) '(m)'];
% text(100,1.65,txt)
% txt = ['Average Error using the truncated Laplace distribution DP-noise= ' num2str(laplace_noise_error_mean) '(m)'];
% text(100,1.55,txt)