function [] = plotting_and_stepinfo(out_obj)
%% Fungsi ini digunakan untuk mencari parameter transien dari sinyal output
%  serta melakukan plotting terhadap sinyal set-point dan sinyal output
%%
clc;

% Plotting
t = out_obj.tout;
setpoint = out_obj.SetPoint;
output = out_obj.Output;
% Tes apakah dimensi dari sumbu waktu dan sumbu vertikal sama
try
    tiledlayout(1,2);
    nexttile;
    plot(t,setpoint,'Color',[0,0.7,0.9]);
    xlabel('Time [second]');
    ylabel('Magnitude [Unit]');
    title('Set-Point');
    nexttile;
    plot(t,output,'Color',[0.4,0.5,0.88]);
    xlabel('Time [second]');
    ylabel('Magnitude [Unit]');
    title('Output/Response');

    % Output the transient response parameters
    param = stepinfo(output,t,1,0);
    disp(param);
% Jika tidak, maka masuk ke blok ini
catch ME
    close all;
    tiledlayout(1,2);
    nexttile;
    plot(t,setpoint,'Color',[0.9,0.05,0.3], 'LineWidth', 1.5);
    xlabel('Time [second]');
    ylabel('Magnitude [Unit]');
    title('Set-Point');
    nexttile;
    plot(t(1:(length(t)-1)),output,'Color',[0.4,0.5,0.88], 'LineWidth', 1.5);
    xlabel('Time [second]');
    ylabel('Magnitude [Unit]');
    title('Output/Response');

    % Output the transient response parameters
    param = stepinfo(output,t(1:(length(t)-1)),1,0);
    disp(param);
end
end