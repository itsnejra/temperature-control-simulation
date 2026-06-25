function data = prepare_data()
%PREPARE_DATA  Ucitava realne podatke o potrosnji i pravi toplinski poremecaj.
%
%   Dataset: Individual Household Electric Power Consumption (UCI / Kaggle).
%   Kolona Global_active_power [kW] se tumaci kao toplinsko opterecenje
%   prostorije: sva potrosena elektricna energija se pretvara u toplotu,
%   pa predstavlja realni poremecaj (disturbance) koji PID mora kompenzovati.
%
%   Rezultat se snima u 'sim_data.mat' i vraca kao struktura.

    here = fileparts(mfilename('fullpath'));
    txt  = fullfile(here, 'data', 'household_power_consumption.txt');
    if ~isfile(txt)
        error('Nedostaje dataset: %s', txt);
    end

    % --- Izbor prozora: jedan dan (1440 minuta) realnih podataka ----------
    N0 = 2;                 % prvi red podataka (preskoci header)
    N  = 1440;              % broj uzoraka (1 dan, minutna rezolucija)

    opts = detectImportOptions(txt, 'Delimiter', ';', 'FileType', 'text');
    opts.DataLines = [N0, N0 + N - 1];
    opts = setvartype(opts, 'Global_active_power', 'char');  % '?' -> char
    T = readtable(txt, opts);

    % --- Ciscenje: '?' (nedostajuce) -> NaN -> linearna interpolacija -----
    gap_kW = str2double(T.Global_active_power);   % '?' postaje NaN
    nMiss  = sum(isnan(gap_kW));
    gap_kW = fillmissing(gap_kW, 'linear');
    gap_kW = fillmissing(gap_kW, 'nearest');      % rubovi ako ostanu

    % --- Vremenska osa (sekunde) -----------------------------------------
    n = numel(gap_kW);
    t = (0:n-1).' * 60;     % minutni uzorci -> sekunde

    % --- Toplinski poremecaj [W]: P[kW] * 1000 ---------------------------
    Q_load = gap_kW * 1000;                        % W

    % --- Pakovanje za Simulink (From Workspace ocekuje [t, u]) -----------
    data.t        = t;
    data.Q_load   = Q_load;
    data.load_ts  = [t, Q_load];
    data.gap_kW   = gap_kW;
    data.nMissing = nMiss;
    data.N        = n;

    save(fullfile(here, 'sim_data.mat'), '-struct', 'data');

    fprintf('Pripremljeno %d uzoraka (%.1f h). Nedostajalo: %d. ', ...
            n, t(end)/3600, nMiss);
    fprintf('Q_load: min=%.0f W, avg=%.0f W, max=%.0f W\n', ...
            min(Q_load), mean(Q_load), max(Q_load));
end
