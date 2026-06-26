%ANALYZE_STABILITY  Analiza dinamike i stabilnosti sistema regulacije temp.
%
%   Generise:
%     - poredjenje P / PI / PID (step odziv + tabela performansi)
%     - Bode dijagram otvorene petlje + gain/phase margin (PID)
%     - pole-zero mapa zatvorene petlje (PID)
%     - Nyquist dijagram (PID)
%     - rezultat realne simulacije (pracenje + poremecaj + napor grijaca)
%   Sve figure se snimaju u /figures.

clc; close all;
here = fileparts(mfilename('fullpath'));
cd(here);
load(fullfile(here,'results.mat'));   % T_meas, U_cmd, params, data
figdir = fullfile(here,'figures');
if ~isfolder(figdir), mkdir(figdir); end

%% Plant
Gact   = tf(1,[tau2 1]);
Gplant = tf(R,[tau 1]);
P_ol   = Gact*Gplant;

%% Kontroleri: P, PI, PID
Cp   = pidtune(P_ol,'P');
Cpi  = pidtune(P_ol,'PI');
Cpid = pid(Kp,Ki,Kd);              % isti kao u simulaciji
ctrls = {Cp,'P'; Cpi,'PI'; Cpid,'PID'};

%% ---- Figura 1: poredjenje step odziva ------------------------------
f1 = figure('Name','Step odziv: P/PI/PID','Color','w','Position',[100 100 720 420]);
hold on; grid on;
colors = lines(3);
rows = {};
for i = 1:3
    Ci = ctrls{i,1};
    CL = feedback(Ci*P_ol,1);          % zatvorena petlja (referenca->izlaz)
    [y,t] = step(CL, 0:1:1500);
    plot(t/60, y, 'LineWidth', 1.8, 'Color', colors(i,:));
    S = stepinfo(CL);
    [Gm,Pm] = margin(Ci*P_ol);
    rows(end+1,:) = {ctrls{i,2}, S.RiseTime, S.SettlingTime, ...
                     S.Overshoot, dcgain(CL), 20*log10(Gm), Pm}; %#ok<SAGROW>
end
yline(1,'k--','referenca');
xlabel('vrijeme [min]'); ylabel('normalizovani odziv');
title('Poredjenje step odziva zatvorene petlje'); legend(ctrls(:,2),'Location','SE');
exportgraphics(f1, fullfile(figdir,'1_step_poredjenje.png'),'Resolution',150);

%% ---- Tabela performansi --------------------------------------------
Tperf = cell2table(rows, 'VariableNames', ...
    {'Kontroler','RiseTime_s','SettlingTime_s','Overshoot_pct', ...
     'DCgain','GainMargin_dB','PhaseMargin_deg'});
disp(' '); disp('=== TABELA PERFORMANSI ==='); disp(Tperf);
writetable(Tperf, fullfile(here,'performanse.csv'));

%% ---- Figura 2: Bode + margine (PID) --------------------------------
L = Cpid*P_ol;                          % otvorena petlja
f2 = figure('Name','Bode + margine (PID)','Color','w','Position',[120 120 720 520]);
margin(L); grid on;
title('Bode dijagram otvorene petlje (PID) sa marginama');
exportgraphics(f2, fullfile(figdir,'2_bode_margine.png'),'Resolution',150);

%% ---- Figura 3: Pole-Zero zatvorene petlje (PID) --------------------
CLpid = feedback(L,1);
f3 = figure('Name','Pole-Zero (PID)','Color','w','Position',[140 140 560 480]);
pzmap(CLpid); grid on; sgrid;
title('Pole-Zero mapa zatvorene petlje (PID)');
exportgraphics(f3, fullfile(figdir,'3_polezero.png'),'Resolution',150);
isstab = all(real(pole(CLpid)) < 0);

%% ---- Figura 4: Nyquist (PID) ---------------------------------------
f4 = figure('Name','Nyquist (PID)','Color','w','Position',[160 160 560 480]);
nyquist(L); grid on;
title('Nyquist dijagram otvorene petlje (PID)');
exportgraphics(f4, fullfile(figdir,'4_nyquist.png'),'Resolution',150);

%% ---- Figura 5: realna simulacija -----------------------------------
tmin = T_meas.Time/60;                   % min
f5 = figure('Name','Realna simulacija','Color','w','Position',[80 80 820 640]);

subplot(3,1,1);
plot(tmin, T_meas.Data,'b','LineWidth',1.5); hold on; grid on;
yline(setpoint,'r--','setpoint'); yline(T_amb,'k:','ambijent');
ylabel('Temp [C]'); title('Regulacija temperature na realni poremecaj');
legend('izmjereno','setpoint','Location','SE');

subplot(3,1,2);
plot(T_meas.Time/60, T_meas.Data - setpoint,'m','LineWidth',1.2); grid on;
ylabel('greska [C]'); yline(0,'k-'); title('Greska regulacije (e = T - setpoint)');

subplot(3,1,3);
plot(data.t/60, dist_gain*data.Q_load,'Color',[0.85 0.4 0],'LineWidth',1.0);
hold on; grid on;
plot(U_cmd.Time/60, U_cmd.Data,'b','LineWidth',1.0);
xlabel('vrijeme [min]'); ylabel('snaga [W]');
title('Toplinski poremecaj (realni podaci) vs. napor grijaca');
legend('poremecaj (potrosnja)','grijac (PID)','Location','NE');
exportgraphics(f5, fullfile(figdir,'5_simulacija.png'),'Resolution',150);

%% ---- Metrika realne regulacije -------------------------------------
e   = T_meas.Data - setpoint;
idx = T_meas.Time > 0.3*T_meas.Time(end);     % ustaljeni rezim
IAE = trapz(T_meas.Time, abs(e));
fprintf('\n=== STABILNOST / PERFORMANSE (realni poremecaj) ===\n');
fprintf('Zatvorena petlja stabilna (svi polovi Re<0): %d\n', isstab);
fprintf('Max temperatura: %.2f C, min: %.2f C\n', max(T_meas.Data), min(T_meas.Data));
fprintf('RMS greska (ustaljeno): %.3f C\n', rms(e(idx)));
fprintf('Max |greska| (ustaljeno): %.3f C\n', max(abs(e(idx))));
fprintf('IAE (ukupno): %.1f C*s\n', IAE);
fprintf('Figure snimljene u: %s\n', figdir);
