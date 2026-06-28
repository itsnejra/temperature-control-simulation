# Simulacija adaptivnog sistema upravljanja temperaturom (MATLAB / Simulink)

Projekt iz predmeta **Računarsko modeliranje i simulacije** (master SE).
Tema 21 — feedback regulacija temperature sa PID kontrolerom, vođena
**realnim podacima** o potrošnji električne energije kao toplinskim poremećajem,
uz bonus **MRAC** (adaptivni) kontroler.

---

## Tok i način odvijanja procesa

### Cilj

Reguliše se temperatura prostorije i održava na zadanoj vrijednosti (**22 °C**),
iako prostorija počinje hladna (**5 °C**, zimski uslovi) i iako je stalno
remeti vanjski uticaj. Princip je isti kao kod termostata i ponavlja se u krug,
svake minute:

1. Izmjeri se trenutna temperatura prostorije.
2. Uporedi se sa zadanom vrijednošću (22 °C) — koliko odstupa.
3. Odredi se potrebna snaga grijanja.
4. Prostorija se zagrije, stanje se promijeni i postupak se vraća na korak 1.

Takvo neprekidno mjerenje vlastitog rezultata i ispravljanje naziva se
**povratna sprega (feedback)** i osnova je automatske regulacije.

### Ulazni podaci i poremećaj

Koristi se tabela sa stvarnim mjerenjima potrošnje električne energije jednog
domaćinstva (minutna rezolucija, period od više godina). Tabela **ne sadrži
temperaturu** — sadrži samo podatak o potrošnji struje.

Polazi se od činjenice da se sva potrošena električna energija na kraju
pretvara u toplotu (uređaji, rasvjeta, mašine zagrijavaju prostor). Zato se
potrošnja struje koristi kao **poremećaj (disturbance)** — realan i
nepredvidiv uticaj koji dodatno zagrijava prostoriju, a koji regulator mora
kompenzovati da bi održao 22 °C. Time se simulacija vodi stvarnim podacima,
a ne pretpostavljenim.

### Odnos ulaza i temperature

- Potrošnja struje iz tabele nije temperatura, već **ulaz (poremećaj)**.
- Temperatura se **izračunava** po zakonima fizike (na osnovu toplote koja
  ulazi od grijača i poremećaja te toplote koja se gubi kroz zidove).
- Tok je dakle: tabela daje poremećaj → regulator zadaje grijanje →
  temperatura se izračunava kao odziv prostorije → regulator je očitava i
  ispravlja se. Postupak se ponavlja kroz cijeli simulirani period.

### Pretpostavke radi realnosti

Budući da se ne pretvara sva potrošena energija u korisnu toplotu te
prostorije, uvedene su dvije pažljive pretpostavke:

- Uzima se isključivo **aktivna snaga** (`Global_active_power`), jer se samo
  ona stvarno troši i pretvara u toplotu. **Jalova snaga**
  (`Global_reactive_power`) se izostavlja, jer se ne pretvara u toplotu, već
  oscilira između izvora i potrošača.
- Uzima se samo dio te snage (≈ **30 %**, parametar `dist_gain = 0.30`) kao
  toplota koja zaista zagrijava posmatranu prostoriju. Ostatak odlazi na
  rasvjetu kroz prozore, u druge prostorije i ventilaciju. Time se izbjegava
  nerealna pretpostavka da sva potrošnja zagrijava upravo tu prostoriju.

### Rezultat

Temperatura je podignuta sa 5 °C na 22 °C i održavana tokom cijelog
simuliranog dana, uprkos stalnom poremećaju. Prosječno odstupanje u
ustaljenom režimu iznosi ≈ **0.37 °C**. Pokazano je da je sistem **stabilan**
i da puni PID nadmašuje jednostavnije varijante (P, PI). Kao proširenje,
**MRAC** regulator podešava vlastita pojačanja tokom rada (adaptivno
upravljanje iz naslova zadatka).

---

## Ideja modela

Reguliše se temperatura prostorije na zadanu vrijednost (**22 °C**) pri
zimskim uslovima (ambijent **5 °C**). Toplinski model prostorije je sistem
2. reda (aktuator/grijač + termalna inercija prostorije):

```
C·dT/dt = Q_grijač + Q_poremećaj − (T − T_amb)/R
```

Prijenosna funkcija od toplote [W] do temperature [°C]:
`G(s) = R / (τ·s + 1)`, sa `τ = R·C`. Aktuator dodaje kašnjenje `1/(τ₂·s+1)`.

**Realni podaci kao poremećaj:** kolona `Global_active_power` [kW] iz UCI/Kaggle
dataseta *Individual Household Electric Power Consumption* tumači se kao
toplinsko opterećenje (potrošena struja → toplota u prostoriji). PID mora
držati 22 °C uprkos tom realnom, promjenjivom opterećenju.

## Blok dijagram (`temp_control.slx`)

![Blok dijagram sistema](figures/0_blok_dijagram.png)

Pojednostavljeni prikaz toka signala:

```
Setpoint(22°C) → [Σ greška] → [PID] → [Saturacija grijača] → [Aktuator]
                     ↑                                            ↓
                     │                       realni podaci → [×coupling] → [Σ +poremećaj]
                     │                                            ↓
                     └──── feedback ──── [+ambijent] ← [Plant: prostorija]
```

## Struktura projekta

| Fajl | Opis |
|------|------|
| `prepare_data.m` | Učitava i čisti dataset, pravi toplinski poremećaj (1 dan, minutno) |
| `build_model.m` | **Programski gradi** Simulink blok dijagram |
| `run_project.m` | **Glavna skripta**: priprema → tuniranje PID-a → gradnja → simulacija |
| `analyze_stability.m` | Analiza: P/PI/PID poređenje, Bode, pole-zero, Nyquist, metrika |
| `mrac_control.m` | Bonus: Model Reference Adaptive Control (adaptivni regulator) |
| `temp_control.slx` | Generisani Simulink model |
| `data/household_power_consumption.txt` | Realni dataset |
| `figures/` | Svi grafici (PNG) |
| `performanse.csv`, `results.mat` | Numerički rezultati |

## Kako pokrenuti

U MATLAB-u (R2023b+, potreban Simulink + Control System Toolbox):

```matlab
run_project          % 1) gradi model i pokreće simulaciju s PID-om
analyze_stability    % 2) analiza stabilnosti + svi grafici
mrac_control         % 3) bonus: adaptivni MRAC + poređenje s PID-om
```

## Ključni rezultati

**Poređenje regulatora (`performanse.csv`):**

| Regulator | Rise [s] | Settling [s] | Overshoot | Greška u ust. stanju | Phase margin |
|-----------|----------|--------------|-----------|----------------------|--------------|
| P   | 57  | 188  | 11.9 % | 6 % (postoji) | 60.0° |
| PI  | 468 | 1792 | 12.9 % | 0 % | 60.0° |
| **PID** | 551 | 1908 | **6.0 %** | **0 %** | **73.8°** |

**Stabilnost (PID, realni poremećaj):** sistem stabilan (svi polovi Re<0),
RMS greška u ustaljenom režimu **0.37 °C**, maksimalna greška **1.77 °C**.

**MRAC (adaptivni):** pojačanja konvergiraju online (θ₁≈290, θ₂≈178, θ₀≈239),
RMS greška **0.52 °C**, max **1.76 °C** — uprkos tome što tretira plant
2. reda kao 1. red (robusnost na nemodelovanu dinamiku).

## Grafici (`figures/`)

1. `1_step_poredjenje.png` — step odziv P/PI/PID
2. `2_bode_margine.png` — Bode otvorene petlje + gain/phase margin
3. `3_polezero.png` — pole-zero mapa zatvorene petlje
4. `4_nyquist.png` — Nyquist dijagram
5. `5_simulacija.png` — regulacija na realni poremećaj (PID)
6. `6_mrac.png` — MRAC praćenje + online konvergencija pojačanja
7. `7_mrac_vs_pid.png` — poređenje greške PID vs MRAC

## Zaključak

Feedback PID rješava zadatak: nulta greška u ustaljenom stanju (I-član),
mali overshoot i visoka fazna margina (robusnost) — dok ga realni podaci o
potrošnji konstantno poremećuju. MRAC dodatno pokazuje princip adaptivnog
upravljanja: regulator uči svoja pojačanja u toku rada, bez prethodnog
tuniranja, i ostaje stabilan na nemodelovanu dinamiku i realne smetnje.
