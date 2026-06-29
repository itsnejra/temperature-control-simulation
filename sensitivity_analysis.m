%SENSITIVITY_ANALYSIS  Analiza osjetljivosti na koeficijent sprege poremecaja.
%
%   Koeficijent dist_gain odredjuje koliki dio potrosene snage zaista
%   zagrijava posmatranu prostoriju (ostatak ode na rasvjetu kroz prozore,
%   druge prostorije, ventilaciju...). Posto je to procjena, ovdje se
%   ispituje koliko je sistem osjetljiv na taj izbor: isti PID i isti realni
%   poremecaj puste se za vise vrijednosti dist_gain i porede se odzivi.
%
%   Rezultat: figura 8_osjetljivost.png + tabela osjetljivost.csv

clc; close all;
here = fileparts(mfilename('fullpath'));
cd(here);
S = load(fullfile(here,'results.mat'));      % parametri + PID + data
R=S.R; tau=S.tau; tau2=S.tau2; u_max=S.u_max; setpoint=S.setpoint;
T_amb=S.T_amb; Kp=S.Kp; Ki=S.Ki; Kd=S.Kd; data=S.data;
load_ts = data.load_ts;  t_end = data.t(end);

%% Gradnja modela (isti kao u glavnoj simulaciji)
mdl = 'temp_control';
build_model(mdl);

%% Vrijednosti koeficijenta sprege koje se ispituju
gains = [0.15 0.30 0.50 0.70];

base = {'setpoint',setpoint;'T_amb',T_amb;'R',R;'tau',tau;'tau2',tau2; ...
        'u_max',u_max;'Kp',Kp;'Ki',Ki;'Kd',Kd;'load_ts',load_ts;'t_end',t_end};

figdir = fullfile(here,'figures'); if ~isfolder(figdir), mkdir(figdir); end
f = figure('Name','Osjetljivost','Color','w','Position',[80 80 820 560]);

ax1 = subplot(2,1,1); hold(ax1,'on'); grid(ax1,'on');
cols = lines(numel(gains));
rows = {};
for i = 1:numel(gains)
    in = Simulink.SimulationInput(mdl);
    for k=1:size(base,1), in = in.setVariable(base{k,1},base{k,2}); end
    in = in.setVariable('dist_gain', gains(i));
    out = sim(in);
    T = out.T_meas;
    plot(ax1, T.Time/60, T.Data, 'LineWidth',1.4,'Color',cols(i,:));

    e = T.Data - setpoint; idx = T.Time > 0.3*T.Time(end);
    rows(end+1,:) = {gains(i), rms(e(idx)), max(abs(e(idx))), max(T.Data)}; %#ok<SAGROW>
end
yline(ax1,setpoint,'k--','setpoint');
xlabel(ax1,'vrijeme [min]'); ylabel(ax1,'Temp [C]');
title(ax1,'Odziv temperature za razlicite koeficijente sprege (dist\_gain)');
legend(ax1, compose('dist\\_gain = %.2f', gains(:)), 'Location','SE');

%% Tabela osjetljivosti
Tsens = cell2table(rows,'VariableNames', ...
    {'dist_gain','RMS_greska_C','Max_greska_C','Max_temp_C'});
disp(' '); disp('=== OSJETLJIVOST NA dist_gain ==='); disp(Tsens);
writetable(Tsens, fullfile(here,'osjetljivost.csv'));

%% Stubci: max greska po koeficijentu
ax2 = subplot(2,1,2);
bar(ax2, categorical(compose('%.2f',gains)), [Tsens.RMS_greska_C Tsens.Max_greska_C]);
grid(ax2,'on'); ylabel(ax2,'greska [C]');
xlabel(ax2,'koeficijent sprege dist\_gain');
title(ax2,'RMS i maksimalna greska regulacije u funkciji sprege');
legend(ax2,{'RMS greska','Max greska'},'Location','NW');

exportgraphics(f, fullfile(figdir,'8_osjetljivost.png'),'Resolution',150);
fprintf('Figura: 8_osjetljivost.png,  tabela: osjetljivost.csv\n');
