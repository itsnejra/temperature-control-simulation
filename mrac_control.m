%MRAC_CONTROL  Model Reference Adaptive Control regulacije temperature.
%
%   Bonus uz klasicni PID: regulator cija se pojacanja (theta) podesavaju
%   ONLINE po Ljapunovljevom zakonu adaptacije, da bi izlaz pratio
%   referentni model.  Pravi plant je 2. reda (aktuator + prostorija),
%   a MRAC ga tretira kao 1. red -> pokazuje robusnost na nemodelovanu
%   dinamiku, uz isti realni poremecaj kao kod PID-a.
%
%   Zakon upravljanja (u deviacijama od ambijenta):
%       u = theta1*r - theta2*y + theta0           (W)
%   Adaptacija (Ljapunov + sigma-modifikacija protiv drifta):
%       dtheta1/dt = -g1*e*r       - sigma*theta1
%       dtheta2/dt =  g2*e*y       - sigma*theta2
%       dtheta0/dt = -g0*e         - sigma*theta0
%   gdje je e = y - y_m  (odstupanje od referentnog modela).

clc; close all;
here = fileparts(mfilename('fullpath'));
cd(here);
S = load(fullfile(here,'results.mat'));     % plant params, PID rezultat, data
R=S.R; tau=S.tau; tau2=S.tau2; dist_gain=S.dist_gain; u_max=S.u_max;
setpoint=S.setpoint; T_amb=S.T_amb; data=S.data;

%% Referentni model: zeljena dinamika (1. red, DC=1, brz ali bez preskoka)
tau_m = 300;                 % vremenska konstanta ref. modela [s]

%% Parametri adaptacije (tunirani)
g1 = 2.0e-2;   g2 = 1.0e-2;   g0 = 2.0;   sigma = 1.0e-5;

%% Integracija (Euler, dt=1 s)
dt = 1; t = (0:dt:data.t(end)).';  n = numel(t);
Qd = dist_gain * interp1(data.t, data.Q_load, t, 'linear', 'extrap');  % poremecaj [W]
r  = setpoint - T_amb;       % komanda u deviaciji od ambijenta [C]

% stanja
h    = 0;                    % izlaz aktuatora (snaga grijaca nakon kasnjenja)
Tdev = 0;                    % temperatura prostorije (deviacija od ambijenta)
ym   = 0;                    % izlaz referentnog modela (deviacija)
th0=0; th1=0; th2=0;         % adaptivna pojacanja

% logovi
Y=zeros(n,1); YM=zeros(n,1); U=zeros(n,1);
TH0=zeros(n,1); TH1=zeros(n,1); TH2=zeros(n,1);

for k=1:n
    y = Tdev;                         % mjerimo deviaciju temperature
    e = y - ym;                       % greska prema referentnom modelu

    % --- zakon upravljanja + saturacija grijaca ---
    u  = th1*r - th2*y + th0;
    us = min(max(u,0),u_max);

    % logovanje
    Y(k)=y+T_amb; YM(k)=ym+T_amb; U(k)=us;
    TH0(k)=th0; TH1(k)=th1; TH2(k)=th2;

    % --- adaptacija (Ljapunov + sigma-mod) ---
    th1 = th1 + dt*(-g1*e*r - sigma*th1);
    th2 = th2 + dt*( g2*e*y - sigma*th2);
    % anti-windup: bias se ne integriše ako bi pogoršao zasićenje
    sat = (us<=0 && e<0) || (us>=u_max && e>0);
    if ~sat
        th0 = th0 + dt*(-g0*e - sigma*th0);
    end

    % --- referentni model ---
    ym = ym + dt*((-ym + r)/tau_m);

    % --- pravi plant (2. red): aktuator + prostorija ---
    h    = h    + dt*((-h + us)/tau2);
    Tdev = Tdev + dt*((-Tdev + R*(h + Qd(k)))/tau);
end

%% --- Figura: MRAC pracenje + adaptacija pojacanja ---
figdir = fullfile(here,'figures'); if ~isfolder(figdir), mkdir(figdir); end
f = figure('Name','MRAC','Color','w','Position',[80 80 860 640]);

subplot(3,1,1);
plot(t/60,Y,'b','LineWidth',1.5); hold on; grid on;
plot(t/60,YM,'g--','LineWidth',1.5); yline(setpoint,'r:','setpoint');
ylabel('Temp [C]'); legend('MRAC izlaz','referentni model','Location','SE');
title('MRAC: pracenje referentnog modela uz realni poremecaj');

subplot(3,1,2);
plot(t/60,TH1,'LineWidth',1.3); hold on; grid on;
plot(t/60,TH2,'LineWidth',1.3); plot(t/60,TH0,'LineWidth',1.3);
ylabel('\theta'); legend('\theta_1 (referenca)','\theta_2 (feedback)','\theta_0 (bias/poremecaj)','Location','E');
title('Online adaptacija pojacanja (konvergencija)');

subplot(3,1,3);
plot(t/60,U,'b','LineWidth',1.0); hold on; grid on;
plot(t/60,Qd,'Color',[0.85 0.4 0],'LineWidth',1.0);
xlabel('vrijeme [min]'); ylabel('snaga [W]');
legend('grijac (MRAC)','poremecaj','Location','NE');
title('Napor grijaca vs. poremecaj');
exportgraphics(f, fullfile(figdir,'6_mrac.png'),'Resolution',150);

%% --- Poredjenje MRAC vs fiksni PID ---
e_mrac = Y - setpoint;
ePID   = S.T_meas.Data - setpoint;  tPID = S.T_meas.Time/60;
idxM = t > 0.3*t(end);  idxP = S.T_meas.Time > 0.3*S.T_meas.Time(end);

f2 = figure('Name','MRAC vs PID','Color','w','Position',[120 120 760 420]);
plot(tPID, ePID,'LineWidth',1.2); hold on; grid on;
plot(t/60, e_mrac,'LineWidth',1.2);
yline(0,'k-'); xlabel('vrijeme [min]'); ylabel('greska [C]');
legend('fiksni PID','MRAC (adaptivni)','Location','NE');
title('Poredjenje greske regulacije: fiksni PID vs MRAC');
exportgraphics(f2, fullfile(figdir,'7_mrac_vs_pid.png'),'Resolution',150);

fprintf('\n=== MRAC REZULTATI ===\n');
fprintf('Konacna pojacanja: theta1=%.1f  theta2=%.1f  theta0=%.1f\n',TH1(end),TH2(end),TH0(end));
fprintf('RMS greska (ustaljeno): MRAC=%.3f C | PID=%.3f C\n', rms(e_mrac(idxM)), rms(ePID(idxP)));
fprintf('Max |greska| (ustaljeno): MRAC=%.3f C | PID=%.3f C\n', max(abs(e_mrac(idxM))), max(abs(ePID(idxP))));
fprintf('Figure: 6_mrac.png, 7_mrac_vs_pid.png\n');
