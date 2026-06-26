%RUN_PROJECT  Glavna skripta: priprema podataka, gradnja modela, simulacija.
%
%   Pokrece kompletan tok:
%     1) priprema realnih podataka (poremecaj)
%     2) definicija fizickog modela prostorije + tuniranje PID-a
%     3) gradnja Simulink modela
%     4) simulacija (PID na realni poremecaj)
%     5) snimanje rezultata za analizu (analyze_stability.m)

clc; clearvars; close all;
here = fileparts(mfilename('fullpath'));
cd(here);

%% 1) Realni podaci -> toplinski poremecaj
data = prepare_data();
load_ts = data.load_ts;
t_end   = data.t(end);

%% 2) Fizicki model prostorije (toplinski sistem 2. reda)
%   C*dT/dt = Q_grijac + Q_poremecaj - (T - T_amb)/Rth
%   Plant od toplote [W] do temperature [C]:  G(s) = R/(tau*s + 1)
R    = 0.005;          % toplinski otpor [C/W]  (DC pojacanje)
C    = 120000;         % toplinski kapacitet [J/C]
tau  = R * C;          % vremenska konstanta prostorije [s]  (= 600 s)
tau2 = 30;             % kasnjenje aktuatora/grijaca [s]

setpoint  = 22;        % zeljena temperatura [C]
T_amb     = 5;         % ambijentalna (zimska) temperatura [C]
dist_gain = 0.30;      % udio potrosene snage koji grije prostoriju
u_max     = 8000;      % maksimalna snaga grijaca [W]

% Prijenosne funkcije
Gact   = tf(1, [tau2 1]);          % aktuator
Gplant = tf(R, [tau 1]);           % prostorija
P_ol   = Gact * Gplant;            % od izlaza kontrolera [W] do temp [C]

%% 3) Tuniranje PID-a (Control System Toolbox)
Cpid = pidtune(P_ol, 'PID');
Kp = Cpid.Kp;  Ki = Cpid.Ki;  Kd = Cpid.Kd;
fprintf('PID tuniran:  Kp=%.1f  Ki=%.3f  Kd=%.1f\n', Kp, Ki, Kd);

%% 4) Gradnja i simulacija modela
mdl = 'temp_control';
build_model(mdl);
save_system(mdl, fullfile(here, [mdl '.slx']));

in = Simulink.SimulationInput(mdl);
vars = {'setpoint',setpoint; 'T_amb',T_amb; 'R',R; 'tau',tau; 'tau2',tau2; ...
        'dist_gain',dist_gain; 'u_max',u_max; 'Kp',Kp; 'Ki',Ki; 'Kd',Kd; ...
        'load_ts',load_ts; 't_end',t_end};
for k = 1:size(vars,1)
    in = in.setVariable(vars{k,1}, vars{k,2});
end
out = sim(in);

T_meas = out.T_meas;          % izmjerena temperatura [C]
U_cmd  = out.U_cmd;           % snaga grijaca [W]

fprintf('Simulacija gotova. T: min=%.2f  zavrsna=%.2f  setpoint=%.0f C\n', ...
        min(T_meas.Data), T_meas.Data(end), setpoint);

%% 5) Snimanje rezultata
save(fullfile(here,'results.mat'), 'T_meas','U_cmd','setpoint','T_amb', ...
     'R','tau','tau2','dist_gain','u_max','Kp','Ki','Kd','data');
fprintf('Rezultati snimljeni u results.mat\n');
