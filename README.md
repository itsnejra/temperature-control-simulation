# Simulacija adaptivnog sistema upravljanja temperaturom (MATLAB / Simulink)

**🇧🇦 Bosanski** | [🇬🇧 English](README.en.md)

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
| `sensitivity_analysis.m` | Analiza osjetljivosti na koeficijent sprege poremećaja |
| `temp_control.slx` | Generisani Simulink model |
| `data/household_power_consumption.txt` | Realni dataset |
| `figures/` | Svi grafici (PNG) |
| `performanse.csv`, `osjetljivost.csv`, `results.mat` | Numerički rezultati |

## Dataset (preuzimanje)

Dataset se čita **lokalno** i nije uključen u repozitorij (≈ 133 MB, prelazi
GitHub limit od 100 MB). Preuzima se zasebno i smješta u folder `data/`:

1. Preuzeti *Individual Household Electric Power Consumption* sa
   [Kaggle](https://www.kaggle.com/datasets/uciml/electric-power-consumption-data-set)
   ili [UCI](https://archive.ics.uci.edu/dataset/235/individual+household+electric+power+consumption).
2. Raspakovati i smjestiti fajl ovako:
   `data/household_power_consumption.txt`

Skripte zatim čitaju podatke s diska; ništa se ne preuzima tokom izvođenja.

## Kako pokrenuti

U MATLAB-u (R2023b+, potreban Simulink + Control System Toolbox):

```matlab
run_project            % 1) gradi model i pokreće simulaciju s PID-om
analyze_stability      % 2) analiza stabilnosti + svi grafici
mrac_control           % 3) bonus: adaptivni MRAC + poređenje s PID-om
sensitivity_analysis   % 4) analiza osjetljivosti na koeficijent sprege
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

**Osjetljivost na koeficijent sprege (`osjetljivost.csv`):**

| dist_gain | RMS greška | Max greška | Max temp |
|-----------|-----------|-----------|----------|
| 0.15 | 0.19 °C | 0.87 °C | 23.5 °C |
| **0.30** (korišteno) | **0.37 °C** | **1.77 °C** | **24.0 °C** |
| 0.50 | 0.62 °C | 2.98 °C | 25.0 °C |
| 0.70 | 1.01 °C | 4.18 °C | 26.2 °C |

Sistem ostaje **stabilan i bez trajne greške** za sve ispitane vrijednosti;
raste samo prolazno odstupanje pri jačim poremećajima. Izbor 0.30 daje
realan i blag režim rada.

## Prijenosna funkcija sistema (algebarski i eksponencijalni oblik)

### Algebarski oblik (racionalna funkcija od `s`)

Gradivni blokovi:

```
Aktuator:    G_act(s) = 1 / (30 s + 1)
Prostorija:  G_p(s)   = 0.005 / (600 s + 1)
Proces:      P(s) = G_act·G_p = 0.005 / (18000 s² + 630 s + 1)
PID:         C(s) = (8334 s² + 309.6 s + 0.856) / s
                  = 8334.1 (s + 0.03414)(s + 0.003008) / s
```

Otvorena petlja `L(s) = C(s)·P(s)`:

```
            0.002315 s² + 8.600·10⁻⁵ s + 2.378·10⁻⁷
L(s) = ─────────────────────────────────────────────
              s³ + 0.035 s² + 5.556·10⁻⁵ s
```

Zatvorena petlja (s kraja na kraj) `T(s) = L / (1 + L)`:

```
            0.002315 s² + 8.600·10⁻⁵ s + 2.378·10⁻⁷
T(s) = ───────────────────────────────────────────────────
        s³ + 0.03732 s² + 1.416·10⁻⁴ s + 2.378·10⁻⁷
```

Polovi zatvorene petlje (svi Re < 0 → stabilno):
`s₁ = −0.0333`, `s₂,₃ = −0.0020 ± 0.0018·j`.

### Eksponencijalni oblik `G(jω) = |G(jω)|·e^{jφ(ω)}`

Za blok prvog reda `K/(τs+1)`, uvrštavanjem `s = jω`:

```
G(jω) = K/(1+jωτ) = [ K / √(1+(ωτ)²) ] · e^( −j·arctan(ωτ) )
```

Pojedinačni blokovi:

```
G_act(jω):  |G| = 1 / √(1+(30ω)²),       φ = −arctan(30ω)
G_p(jω):    |G| = 0.005 / √(1+(600ω)²),  φ = −arctan(600ω)
```

Proces (moduli se množe, faze sabiraju):

```
|P(jω)| = 0.005 / [ √(1+(30ω)²) · √(1+(600ω)²) ]
 φ_P(ω) = −arctan(30ω) − arctan(600ω)
```

PID `C(jω) = Kp + j(ωKd − Ki/ω)`:

```
|C(jω)| = √( Kp² + (ωKd − Ki/ω)² )
 φ_C(ω) = arctan( (ωKd − Ki/ω) / Kp )
```

Otvorena i zatvorena petlja:

```
L(jω) = |C(jω)|·|P(jω)| · e^{ j(φ_C + φ_P) }
T(jω) = ( |L| / |1+L| ) · e^{ j( argL − arg(1+L) ) }
```

U eksponencijalnom obliku **moduli se množe, a faze sabiraju** — to je i razlog
zašto Bode dijagram radi: množenje modula postaje sabiranje u decibelima, a faze
se direktno sabiraju.

## Analiza stabilnosti: koja petlja i zašto

Svaki alat za stabilnost vezan je za tačno određenu petlju — to nije
proizvoljno, nego proizlazi iz same definicije metode. Posmatraju se dvije
prijenosne funkcije:

- **Otvorena petlja** `L(s) = C(s)·P(s)` — regulator i proces u nizu, bez
  zatvaranja povratne sprege.
- **Zatvorena petlja** `T(s) = L(s) / (1 + L(s))` — stvarni sistem koji radi,
  s povratnom spregom.

| Alat | Petlja | Zašto baš ta petlja |
|------|--------|---------------------|
| **Bode + margine** | **otvorena** `L(s)` | Amplitudna i fazna margina su **definisane na otvorenoj petlji** — čitaju se iz amplitude i faze od `L(s)`. One mjere koliko sistem ima rezerve do nestabilnosti; taj podatak postoji samo na otvorenoj petlji. |
| **Nyquist** | **otvorena** `L(s)` | Nyquistov kriterij je **po prirodi metoda nad otvorenom petljom**: crta se `L(jω)` i broje obilasci tačke (−1, 0) — `Z = N + P`. Cilj je iz otvorene petlje **zaključiti** o zatvorenoj, pa Nyquist na zatvorenoj petlji nema smisla. |
| **Pole-zero (polovi/nule)** | **zatvorena** `T(s)` | Stvarni sistem koji se izvršava je zatvorena petlja. Njena stabilnost zavisi od korijena `1 + L(s) = 0`, tj. od **polova zatvorene petlje**. Ako su svi Re < 0 → stabilno. Polovi otvorene petlje to ne dokazuju (stabilna otvorena može dati nestabilnu zatvorenu i obrnuto). |

Logika je dakle dvostruka: **otvorena petlja predviđa** stabilnost i daje
margine (Bode, Nyquist), a **zatvorena petlja direktno potvrđuje** stabilnost
(položaj polova). Sva tri alata daju isti zaključak — stabilan sistem — što je
unakrsna provjera iz tri ugla.

## Grafici sa objašnjenjima

### Blok dijagram sistema
![Blok dijagram](figures/0_blok_dijagram.png)

Struktura modela u Simulinku: referenca (Setpoint) i izmjerena temperatura
ulaze u sumator greške, PID određuje snagu grijača (ograničenu saturacijom),
aktuator i prostorija (Plant) daju temperaturu, a realni podaci (Load) preko
koeficijenta sprege ulaze kao poremećaj. Povratna sprega zatvara petlju.

### 1. Poređenje step odziva P / PI / PID
![Step odziv](figures/1_step_poredjenje.png)

Odziv zatvorene petlje na skok reference. **P** je brz ali ostaje trajna
greška (ne dostiže 1). **PI** uklanja grešku ali je sporiji i s većim
preskokom. **PID** ima najmanji preskok i nultu trajnu grešku — najbolji
kompromis.

### 2. Bode dijagram + margine stabilnosti (PID)
![Bode](figures/2_bode_margine.png)

Frekvencijski odziv otvorene petlje. Pokazuje **faznu i amplitudnu marginu** —
mjeru koliko je sistem daleko od nestabilnosti. Visoka fazna margina (≈ 74°)
znači robustan, dobro prigušen sistem.

### 3. Pole-zero mapa zatvorene petlje (PID)
![Pole-zero](figures/3_polezero.png)

Položaj polova sistema u kompleksnoj ravni. **Svi polovi su lijevo od
imaginarne ose (Re < 0)** → sistem je stabilan. Položaj polova određuje
brzinu i prigušenje odziva.

### 4. Nyquist dijagram (PID)
![Nyquist](figures/4_nyquist.png)

Još jedan dokaz stabilnosti: kriva ne obuhvata kritičnu tačku (−1, 0), što
po Nyquistovom kriteriju potvrđuje stabilnost zatvorene petlje.

### 5. Regulacija na realni poremećaj (PID)
![Simulacija](figures/5_simulacija.png)

Glavni rezultat. Gore: temperatura se diže s 5 °C i drži na 22 °C cijeli dan.
Sredina: greška ostaje blizu nule. Dolje: vidi se **inverzna sprega** — kad
poremećaj (potrošnja) skoči, PID smanji snagu grijača, i obrnuto.

### 6. MRAC — praćenje i adaptacija pojačanja
![MRAC](figures/6_mrac.png)

Adaptivni regulator. Gore: izlaz prati referentni model. Sredina: **pojačanja
se sama podešavaju (uče) tokom rada** i konvergiraju. Dolje: napor grijača
prema poremećaju.

### 7. Poređenje greške PID vs MRAC
![MRAC vs PID](figures/7_mrac_vs_pid.png)

Direktno poređenje greške regulacije fiksnog PID-a i adaptivnog MRAC-a na
istom realnom poremećaju.

### 8. Analiza osjetljivosti
![Osjetljivost](figures/8_osjetljivost.png)

Odziv za različite vrijednosti koeficijenta sprege. Gore: temperaturni odzivi.
Dolje: RMS i maksimalna greška rastu s koeficijentom, ali sistem ostaje
stabilan — potvrda da zaključci ne zavise kritično od te procjene.

## Zaključak

Feedback PID rješava zadatak: nulta greška u ustaljenom stanju (I-član),
mali overshoot i visoka fazna margina (robusnost) — dok ga realni podaci o
potrošnji konstantno poremećuju. MRAC dodatno pokazuje princip adaptivnog
upravljanja: regulator uči svoja pojačanja u toku rada, bez prethodnog
tuniranja, i ostaje stabilan na nemodelovanu dinamiku i realne smetnje.
Analiza osjetljivosti pokazuje da zaključci o stabilnosti i tačnosti vrijede
za širok raspon koeficijenta sprege, pa rezultati ne zavise kritično od te
procjene.
