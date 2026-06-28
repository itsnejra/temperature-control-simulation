# Simulacija adaptivnog sistema upravljanja temperaturom (MATLAB / Simulink)

Projekt iz predmeta **Računarsko modeliranje i simulacije** (master SE).
Tema 21 — feedback regulacija temperature sa PID kontrolerom, vođena
**realnim podacima** o potrošnji električne energije kao toplinskim poremećajem,
uz bonus **MRAC** (adaptivni) kontroler.

---

## 📖 Objašnjenje za sve (bez stručnih riječi)

### Šta ovaj projekat radi?

Napravili smo **pametan termostat** — onu spravu na zidu koja drži grijanje
na željenoj temperaturi. Naš termostat treba da održi sobu na **22 °C**, iako
soba kreće hladna (**5 °C**, zima) i iako je nešto stalno remeti.

Princip rada je kao kod svakog termostata, u krug, svake minute:

1. **Izmjeri** koliko soba trenutno ima stepeni
2. **Uporedi** sa ciljem (22 °C) → koliko fali ili je previše?
3. **Odluči** koliko jako grijati
4. **Grij**, soba se promijeni, pa nazad na korak 1

Ovo "stalno mjeri svoj rezultat i ispravlja se" se zove **povratna sprega
(feedback)** — srce svake automatske regulacije.

### Odakle realni podaci i šta je "smetnja"?

Dobili smo tabelu sa **pravim mjerenjima potrošnje struje** jedne kuće
(svaka minuta, 4 godine). U tabeli **nema temperature** — ima samo koliko
struje kuća troši.

Ključna ideja: **sva potrošena struja se na kraju pretvori u toplotu**
(šporet, TV, sijalice, mašine... sve to grije prostor). Zato potrošnju struje
koristimo kao **smetnju (poremećaj)** — nešto realno i nepredvidivo što stalno
dodatno zagrijava sobu, a naš termostat se mora boriti protiv toga da bi
održao tačno 22 °C. Time projekat radi na **stvarnim podacima iz života**, ne
na izmišljenim.

### Šta je tačno temperatura, a šta nije? (važno!)

- Potrošnja struje iz tabele **NIJE** temperatura — to je **ulaz/smetnja**.
- **Temperaturu računar SAM izračuna** po zakonima fizike (koliko toplote uđe
  od grijača i smetnje, koliko pobjegne kroz zidove → kolika je nova temp).
- Dakle: tabela daje smetnju → termostat daje grijanje → **računar iz toga
  izračuna temperaturu** → termostat je gleda i ispravlja se. I tako u krug.

### Šta smo uzeli u obzir da bude realno?

Pošto **ne pretvori se SVA struja u korisnu toplotu baš te sobe**, bili smo
oprezni:

- **Koristimo samo "aktivnu" snagu** (`Global_active_power`) — onu koja se
  zaista troši i pretvara u toplotu. **Jalovu snagu** (`Global_reactive_power`,
  koja se samo "ljulja" naprijed-nazad i ne grije ništa) **smo izostavili**,
  jer ona ne daje toplotu.
- **Uzimamo samo ~30 %** te snage kao toplotu koja zaista zagrije baš ovu
  sobu (parametar `dist_gain = 0.30`). Ostatak ode na svjetlo kroz prozore,
  u druge prostorije, ventilaciju i sl. Time izbjegavamo nerealnu pretpostavku
  da baš sve grije našu sobu.

### Šta smo dobili?

Termostat je sobu podigao sa 5 °C na 22 °C i **držao je tu cijeli dan**,
uprkos stalnoj smetnji. Prosječno odstupanje od cilja: svega **~0.37 °C**.
Pokazali smo i da je sistem **stabilan** (ne počne divljati) i da je puni PID
bolji od jednostavnijih varijanti. Kao bonus, **MRAC** regulator **sam uči i
podešava se** dok radi — to je "adaptivni" dio iz naslova zadatka.

> **Jednom rečenicom:** uzeli smo prave podatke o potrošnji struje, pretvorili
> ih u toplotnu smetnju (pažljivo: samo aktivnu snagu i samo ~30 % nje), pa
> napravili pametan termostat koji uspješno i stabilno drži sobu na 22 °C
> cijeli dan uprkos toj smetnji.

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
