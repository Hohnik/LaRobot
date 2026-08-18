# Setup-Anleitung: Bimanuale YAM-Station (ABC-Stack)

*Operative Schritt-für-Schritt-Anleitung · v2 mit Zwei-Zweig-Struktur · Stand 2026-08-06*
*Ergänzt [Setup-Plan.md](Setup-Plan.md) (Strategie & Begründungen)*

**Wie zu benutzen:** Reihenfolge einhalten. Jeder Schritt hat **Tun** und **⚠️ Achtung**. Jede Phase endet mit einem **✅ Gate** — erst weiter, wenn erfüllt. Fehler in Phase B/C (Licht, Datenformat, Encoding) sind später **nur durch Neusammeln** reparierbar.

**Struktur:** Phasen **A–C sind gemeinsam** (Hardware + Teleop + Daten). Danach teilt sich der Weg in **zwei Zweige**, die in Phase E wieder zusammenlaufen:

```
A ── B ── C ──┬── Zweig 1: klein & lokal  (Diagnose, schnell)  ──┬── E ── F
              └── Zweig 2: groß & extern  (Sommerziel, stark)  ──┘
```

**Grundprinzip:** Lokal (RTX 5090) = Teleop, Daten, Inferenz, Deployment. Extern (8× H100/H200) = großes Training/SFT. Die Naht ist das **MCAP-/ABC-Datenformat** — der Vertrag, an dem alles hängt.

---

## Phase A — Basis: OS, Treiber, ABC-Repo *(gemeinsam)*

> Ziel: ABCs Policy läuft in der Sim. **Braucht noch keine Hardware.**

### A1. Betriebssystem
- **Tun:** **Ubuntu LTS** — **24.04** (Kernel 6.8, besser für Blackwell) oder **22.04** (exakte ENPIRE-Referenz). Secure Boot ggf. aus.
- **⚠️ Achtung:** **Kein Arch/Rolling Release** — ein Update mitten in der Datensammlung kann NVIDIA-Modul, CAN oder RealSense brechen. **Kein Windows** (Stack ist Linux-first). Distro nach Support-Matrix wählen, nicht nach Geschmack.

### A2. GPU-Stack
- **Tun:** Aktueller NVIDIA-Treiber + CUDA (Referenz: 595.x / CUDA 13.2; praktisch: neuester stabiler mit **sm_120**). Verifizieren: `nvidia-smi`.
- **⚠️ Achtung:** RTX 5090 = Blackwell (**sm_120**). Alte PyTorch/CUDA-Builds kennen die Architektur nicht. **Früh testen.**

### A3. Python-Umgebung
- **Tun:** `uv` (ABC nutzt `uv`, Python **3.12**) · `sudo apt-get install -y ffmpeg` · PyTorch mit CUDA-Build. Verifizieren: `torch.cuda.is_available()` + GPU-Matmul.
- **⚠️ Achtung:** **Jetzt** festlegen, dass lokale und externe Env identisch sind (gleiche `uv.lock`) — sonst später „läuft lokal, bricht auf dem Cluster".

### A4. ABC-Repo
- **Tun:** `git clone https://github.com/amazon-far/abc`, Deps per `uv`, **DINOv3-Gewichte (~8 GB) separat laden**.
- **⚠️ Achtung:** DINO-Gewichte sind **nicht** im Repo — großer Download, früh anstoßen.

### A5. Checkpoint + Sim
- **Tun:**
  ```bash
  uv run prepare.py --checkpoint   # 75k-DiT-Checkpoint (put_bottles)
  uv run eval_policy.py            # MuJoCo-Warp Sim-Eval
  uv run viz_policy.py             # Viser-Visualisierung
  ```
- **⚠️ Achtung:** Schnellster „aha"-Moment **und** Beweis, dass GPU/CUDA/Deps stimmen. Läuft das, ist die halbe Softwareseite validiert. `eval_policy.py` nutzt **MuJoCo-Warp** (GPU) → Probleme liegen meist am CUDA-Stack, nicht am Code.

### A6. Externen Cluster testen *(parallel; nur für Zweig 2 nötig)*
- **Tun:** Identische Env extern aufsetzen, einmal auf ABCs Beispieldaten trainieren:
  ```bash
  uv run torchrun --standalone --nproc-per-node 8 train.py
  ```
  Referenz: **~2,6–3 it/s** auf H100/H200 bei Batch 90/GPU; Loss **~0,048** nach 75k Schritten.
- **⚠️ Achtung:** Braucht **8 GPUs mit je ≥80 GB**. Datenweg (lokal → **S3/GCS** → Cluster streamt) und Checkpoint-Rückweg **jetzt** klären, nicht später improvisieren. **Dieser Schritt ist der größte Terminunsicherheitsfaktor** — deshalb existiert Zweig 1.

> ### ✅ Gate A
> Sim-Eval läuft lokal mit 75k-Checkpoint · (für Zweig 2) Cluster-Trainingslauf startet · Datenweg steht.

---

## Phase B — Hardware & physischer Aufbau *(gemeinsam)*

> Jede Komponente **einzeln** testen, bevor etwas zusammengeschaltet wird.

### B1. Arbeitsplatz & visuelle Umgebung
- **Tun:** Zwei YAM-Arme parallel montieren, **Einhausung an 3 Seiten, weiße Wände** (ABC C.1). Arbeitsfläche **matt, einfarbig, kontrastierend zu den Objekten**. **Konstantes, diffuses Kunstlicht**, Fenster abdunkeln.
- **⚠️ Achtung:**
  - **Kein Tageslicht** → Policy lernt sonst „Tageszeit" als Scheinvariable.
  - **Keine harten Punktstrahler** → wandernde Schatten (auch Arm-Eigenschatten) = Scheinvarianz.
  - **Nichts Glänzendes** → Glanzlichter wandern und brennen die Belichtung aus.
  - Hintergrund **statisch** (keine durchlaufenden Personen).
  - **Merksatz:** Die konkrete Farbe ist beliebig — **Konstanz** ist alles, und die Bedingungen müssen beim **Deployment identisch** sein.

### B2. CAN + Arme
- **Tun:** I2RT-Doku → **CAN-Adapter** verifizieren/beschaffen. SocketCAN: `ip link show`, `candump can0`. Dann **i2rt Python-API** (`github.com/i2rt-robotics/i2rt`), **einen** Arm bewegen.
- **⚠️ Achtung:**
  - **Wahrscheinlichster Blocker des Projekts.** Adapter-Kompatibilität (SocketCAN, Baudrate) **vor dem Kauf** klären.
  - Erst ein Arm, dann beide (CAN-IDs nicht verwechseln).
  - **Greifer kraftbegrenzt** betreiben (torque-limited compliant grasp, ENPIRE B.3): schließt mit **begrenzter Kraft**, nicht auf feste Weite → robustes Greifen *und* Sicherheit (Fehlgriff = sicherer Stillstand statt Hardwareschaden).
  - **Software-Arbeitsraumgrenzen** einziehen, **bevor** autonome Rollouts laufen.

### B3. Kameras
- **Tun:** `librealsense2` + `pyrealsense2`. **2× D405 an den Handgelenken** + **1× normale RGB-Kamera top-down**. Mit `realsense-viewer` prüfen.
- **Verifizieren:** 3 Views gleichzeitig, **stabile 30 fps, keine Drops**.
- **⚠️ Achtung:**
  - **USB-Bandbreite:** auf **getrennte USB-3-Controller** verteilen, nicht alle an einen Hub.
  - **Belichtung + Weißabgleich an ALLEN Kameras fest** (Auto aus). Auto-Exposure ändert Helligkeit korreliert mit Szeneninhalt → giftig fürs Lernen.
  - Normale Top-Kamera ist ok (ABC-DiT ist RGB-only), braucht aber **stabile 30 fps** und **sauberes eigenes Timestamping**.
  - Montage so planen, dass **4. Kamera** (RoboTTT) / Top-D405 später ergänzbar ist.

### B4. SpaceMice
- **Tun:** `spacenavd`/`libspnav` oder `pyspacemouse`; **beide** gleichzeitig auslesen und **eindeutig unterscheiden**.
- **⚠️ Achtung:** Zwei identische HID-Geräte sind der klassische Stolperstein — feste udev-Regeln/Seriennummern, sonst tauschen die Arme nach einem Neustart die Steuerung.

> ### ✅ Gate B
> Beide Arme fahren sicher (Kraftlimit + Arbeitsraumgrenzen) · 3 Views @ 30 fps ohne Drops, Auto-Modi aus · beide SpaceMice eindeutig · Szene visuell konstant.

---

## Phase C — Teleop + Datenaufzeichnung *(gemeinsam — der kritischste Teil)*

> **Hier entscheidet sich die Datenqualität. Beide Zweige lesen dieselben Daten.**

### C1. IK-Kette
- **Tun:** `mink` installieren. Kette: **SpaceMouse-Twist → Ziel-EE-Pose → IK → Gelenkziele → Arm**. Robotermodell: **YAM-MJCF aus dem ABC-Repo** (`assets/put_bottles/assets/i2rt_yam/yam.xml` + `scene.xml`).
- **⚠️ Achtung:** Das YAM-Modell bekommt ihr **geschenkt** — nicht selbst modellieren. IK-Sprünge/Singularitäten abfangen (Gelenk- und Geschwindigkeitslimits), sonst landen **Ruckler in den Trainingsdaten**. Erst ein Arm, dann beide.

### C2. Robot-Loop
- **Tun:** Für den Start **ein einzelner Python-Prozess**: Kameras + SpaceMice lesen → IK → Arme → Recorder. Module: `spacemouse`×2, `ik`, `yam_arm`×2, `camera`×3, `recorder`, später `policy`.
- **Taktraten:** Policy/Aufzeichnung **30 Hz**; Low-Level **100 Hz** über CAN (PD + Gravitationskompensation).
- **⚠️ Achtung:** **ZMQ ist nicht nötig** — ABCs ZMQ-Framework ist nicht im Release, und für eine Station ist ein Prozess einfacher. ZMQ/ROS 2 erst bei echtem Entkopplungsbedarf. (ABC-Timing-Trick, falls doch Nodes: `time.sleep` grob, letzte ~300 µs Busy-Spin.)

### C3. Recorder: MCAP mit ABCs Topic-Schema
- **Tun:** MCAP schreiben mit **genau diesen Topics** (aus `export_mcap.py`):
  ```
  Kameras: /top-camera   /left-wrist-camera   /right-wrist-camera
  States:  /left-arm-state (6)   /left-ee-state (1)   /right-arm-state (6)   /right-ee-state (1)
  Actions: /left-arm-action (6)  /left-ee-action (1)  /right-arm-action (6)  /right-ee-action (1)
  ```
- **⚠️ Achtung (wichtigster Punkt der Anleitung):**
  - **MCAP ist Single Source of Truth.** Exakte Namen/Dimensionen → ABCs komplette Kette läuft **unverändert**, für **beide Zweige**.
  - **Aktionsraum = Joint-Space** (6 Gelenke + 1 Greifer/Arm). Eure **IK-Ausgabe** ist die *action*, nicht der SpaceMouse-Input.
  - **Zusätzlich alles mitloggen** (SpaceMouse-Roh, EE-Pose, Greiferkraft) in eigenen Topics — kostet fast nichts, erlaubt später andere Aktionsräume ohne Neusammeln.
  - **Tick = 33.333.333 ns** (30 Hz). Alle Streams synchron halten.

### C4. Format + Encoding verifizieren — **mit Mini-Sample, VOR dem echten Sammeln**
- **Tun:** 2–3 Test-Episoden aufnehmen, konvertieren, prüfen:
  ```bash
  uv run export_mcap.py ./train_run_1 ./out
  ```
  Erwartet pro Episode:
  ```
  episode_<uuid>/
    states_actions.bin              # (num_steps, 28) float64 = 14 state + 14 action
    combined_camera-images-rgb.mp4  # 3 Views vertikal gestapelt, 224×224, 30 fps
    episode_metadata.json
  ```
- **⚠️ Achtung:**
  - Encoding ist **strikt**, weil der Trainer den Frame-Index *analytisch* rekonstruiert: `libx264 -preset fast -crf 18` · **`-bf 0`** · **GOP=30, kein Scenecut** · **faststart** · `yuv420p` · **Timebase 1/15360, PTS = 512·k**, Keyframe bei `k%30==0`.
  - **Bester Rat: nicht selbst encodieren — `export_mcap.py` macht es korrekt.**
  - Falsches Encoding (GOP 250, B-Frames) → Dataloader **~70× langsamer**.
  - `.bin`-Form **numerisch** verifizieren (`num_steps × 28`, float64), nicht per Augenmaß.

### C5. Datensatz-Config anlegen (`norm_stats.json` + Mixture)
- **Tun:** Der Trainer erwartet in `cache_root`:
  - **`norm_stats.json`** (z-Score-Statistiken für States/Actions) — für **eure** Daten erzeugen.
  - Eine **Mixture** statt des `bottles`-Presets: `MixtureComponent(train_dir, val_dir, weight, task_name)` mit **eigenem `task_name`** (= euer Sprach-Prompt) und eigenen train/val-Ordnern.
- **⚠️ Achtung:**
  - **Mixture-Gewichte müssen exakt auf 1,0 summieren** (`validate_train_config` bricht sonst ab).
  - `task_name` ist der Prompt, auf den ihr später beim Deployment konditioniert — **identisch halten** zwischen Training und Inferenz.
  - **Val-Split von Anfang an abtrennen** (eigene Episoden, nicht Frames aus Trainingsepisoden), sonst ist die Validierung wertlos.

### C6. Aufgabe(n) festlegen
- **Tun:** **1–2 Aufgaben**: simpel, **gut resettbar**, klares Erfolgskriterium (z. B. „Objekt in Behälter legen"). Kategorie *Pick-and-Place*.
- **⚠️ Achtung:** Eine Aufgabe möglichst **nah an ABCs `put_bottles`** → dann ist der 75k-Checkpoint ein besonders guter SFT-Start (Zweig 2). **Nicht** mit dexterosen Aufgaben (Falten, Insertion) anfangen.

### C7. Daten sammeln
- **Tun:** **~50–200 Demos pro Aufgabe** (grob 1–3 h). **Objekt- und Startpositionen variieren.** Teleop vorher üben.
- **⚠️ Achtung:**
  - **Nur variieren, was variieren soll:** Objektlage/Aufgabenzustand — **nicht** Licht, Kamera, Hintergrund. Sonst Scheinkorrelationen (von ABC dokumentiert).
  - **Einheitliche SOP** über alle Demos (ABC musste wegen inkonsistenter Teleoperator-Stile extra nachsammeln).
  - Bimanuelle SpaceMouse-Teleop ist ungewohnt → **Übungsdemos verwerfen**, nicht ins Training.
  - Episoden **stichprobenartig ansehen** (Video + Trajektorie), nicht blind 200 aufnehmen.

> ### ✅ Gate C
> Teleop flüssig · Konvertierung erzeugt exakt das ABC-Format (numerisch verifiziert) · `norm_stats.json` + Mixture vorhanden · Val-Split getrennt · 50–200 saubere Demos unter konstanten Bedingungen.

---

# ⑃ Verzweigung

Ab hier zwei Zweige. **Beide nutzen denselben ABC-Code, dasselbe Datenformat, denselben Deploy-Pfad** — sie unterscheiden sich nur in **Modellgröße und Trainingsort**.

| | **Zweig 1 — klein & lokal** | **Zweig 2 — groß & extern** |
|---|---|---|
| **Zweck** | **Diagnose:** Sind die Daten lernbar? Läuft der Deploy-Loop? | **Liefergegenstand:** starke Policy (Sommerziel) |
| Modell | ABC-DiT klein (~150 M): `hidden_size 384`, `depth 12`, `num_heads 6` | ABC-DiT voll (2 B): Defaults `1536 / 32 / 24` |
| Start | `load_pretrained=False` (from scratch) | `load_pretrained=True` (75k-Checkpoint) |
| Compute | **1× RTX 5090**, `--nproc-per-node 1`, Batch 16–32 | **8× H100/H200**, Batch 90/GPU |
| Zeit bis Signal | **Stunden** | Tage–Wochen (Cluster-abhängig) |
| Externe Abhängigkeit | **keine** | Cluster, Upload, Queue |
| Erwartete Leistung | mäßig (klein + from scratch) | hoch |

**Empfehlung:** **Zweig 1 zuerst starten und zeitlich boxen (max. 2 Wochen)**, dann Zweig 2. Zweig-1-Arbeit fällt in die Totzeit (Warten auf CAN-Adapter, Einhausung, Cluster-Zugang) — geringer Mehraufwand, hoher Informationsgewinn. Zweig 2 kann parallel vorbereitet werden.

**⚠️ Warum Zweig 1 überhaupt:** Zweig 2 hat **sechs serielle Abhängigkeiten** vor dem ersten autonomen Rollout (Schema → Encoding → norm_stats → Cluster → Training → Deploy). Scheitert es, wisst ihr nicht welche. Zweig 1 halbiert die Kette und nimmt **den Cluster vom kritischen Pfad**.

**⚠️ Warum lokal ein *kleines* Modell sein muss:** Der 5090 kann ABC-DiT-2B **hervorragend ausführen** (36 ms/Chunk), aber **nicht komfortabel trainieren**: 2 B ≈ 4 GB Gewichte + ~16 GB AdamW-Zustände + 4 GB Gradienten ≈ **24 GB vor Aktivierungen** auf 32 GB. Ein ~150-M-Modell braucht ~2 GB.

**⚠️ Scope-Drift-Warnung:** Zweig 1 ist **Diagnose, nicht Ziel**. Sobald die Gates D1 erfüllt sind → zu Zweig 2. Nicht „bei klein bleiben, läuft doch".

---

## Zweig 1 — Klein & lokal (Diagnose) → Gate D1

### D1.1 Overfit-Test *(der wertvollste Einzeltest überhaupt)*
- **Tun:** Kleines DiT auf **5–10 Episoden** trainieren, bis der Trainings-Loss sehr niedrig ist. Prüfen, ob das Modell diese Episoden **reproduziert** (Sim-Rollout / Aktionsvergleich).
- **⚠️ Achtung:** **Kann ein Modell 5 Episoden nicht overfitten, ist die Pipeline kaputt** — typisch: Zeitversatz zwischen Bild und Aktion, falsche Kamera-Reihenfolge im Stack, kaputte Normalisierung, vertauschte links/rechts-Arme. Dieser Test kostet Stunden und spart Wochen. **Vor allem anderen machen.**

### D1.2 Kleines Modell konfigurieren
- **Tun:** In `DiTConfig` **nur den Action-Head verkleinern**:
  ```python
  hidden_size = 384     # statt 1536
  depth       = 12      # statt 32
  num_heads   = 6       # statt 24
  ```
  In `TrainConfig`: `batch_size = 16…32`, `load_pretrained = False`, `train_steps` klein halten (z. B. 20–50k), `compile = True`.
- **⚠️ Achtung:**
  - **Die `vit_*`-Felder NICHT ändern** (`vit_embed_dim 768`, `vit_depth 12`, `vit_num_heads 12`) — sie müssen zum **DINOv3-ViT-B-Checkpoint** passen. Nur das DiT-Kopf-Trio anfassen.
  - Constraints aus `validate_model_config`: `hidden_size % num_heads == 0` und `hidden_size % 2 == 0`. (384/6 ✓)
  - `camera_keys` unverändert lassen (`("top","left","right")`), solange ihr 3 Views habt.

### D1.3 Lokal trainieren
- **Tun:** `uv run torchrun --standalone --nproc-per-node 1 train.py` mit der kleinen Config.
- **⚠️ Achtung:** Bei OOM zuerst `batch_size` senken, dann `compile=False` zum Debuggen (Compile verschleiert Fehlermeldungen). Kleine Batches → verrauschter Loss; nicht überinterpretieren.

### D1.4 Auswerten
- **Tun:** Trainingsverlauf + **Validation Action Error** ansehen; Rollout in **Sim** (`eval_policy.py`) und danach real (→ Phase E).
- **⚠️ Achtung:** Erfolgskriterium ist **„lernt die Aufgabe im Prinzip"**, nicht hohe Erfolgsrate. Ein kleines Modell from scratch auf 100 Demos ist erwartbar schwächer als das finegetunte 2 B. **Nicht** hier auf Leistung optimieren.

> ### ✅ Gate D1
> Overfit-Test bestanden (⇒ Datenpipeline korrekt) · kleines Modell trainiert lokal ohne OOM · Rollout zeigt aufgabenrelevantes Verhalten · Deploy-Loop steht (Phase E) → **jetzt zu Zweig 2**.

---

## Zweig 2 — Groß & extern (Sommerziel) → Gate D2

### D2.1 Daten hochladen
- **Tun:** Konvertierte Episoden nach **S3/GCS**; ABCs Dataloader streamt direkt daraus.
- **⚠️ Achtung:** Ordner-/Shard-Struktur sauber pro Aufgabe — ihr werdet Quellen später **mischen und gewichten** (Pretraining-Mix, Interventionsdaten). `norm_stats.json` und Mixture-Config mit hochziehen.

### D2.2 SFT starten
- **Tun:** Volle Config (Defaults `1536/32/24`), **`load_pretrained = True`** vom 75k-Checkpoint:
  ```bash
  uv run torchrun --standalone --nproc-per-node 8 train.py
  ```
  Finetuning-Anker aus ABC: AdamW, **LR 1e-5**, Weight Decay 1e-2, Gradient-Clipping (max-norm 10). *(Repo-Defaults sind Pretraining-Werte: LR 1e-4, 1000 Warmup-Schritte, `mask_state_ratio` 0.1 = Proprio-Dropout.)*
- **⚠️ Achtung:**
  - **Vom Checkpoint starten, nicht from scratch** — ABC zeigt klar: besseres Pretraining ⇒ stärkere Single-Task-Policy.
  - **LR nicht auf dem Pretraining-Default lassen** — 1e-4 auf wenigen Demos zerstört das Vortrainierte („katastrophales Vergessen"). Für SFT **1e-5**.
  - Bilder 224×224 (Letterbox), 30-Schritt-Chunks, z-Score — konsistent Training↔Inferenz.

### D2.3 Checkpoint auswählen — **die richtige Metrik**
- **Tun:** Auswahl über **Validation Action Error** (L2 zum Ground-Truth-Chunk bei **fester** Diffusions-Schrittzahl) + **Training Loss**. Zusätzlich in Sim ansehen (ABC prüft alle 50k Schritte visuell).
- **⚠️ Achtung — klassische Falle:**
  - **Validation Loss ist NICHT mit Real-Performance korreliert** (ABC: r = −0,04, p = 0,89) — sie kann steigen, während die Policy **besser** wird. **Nicht danach auswählen.**
  - Action Error korreliert stark (r ≈ −0,89), aber **nur bei fest gehaltener Diffusions-Schrittzahl** — sonst „verbessert" ihr den Wert trivial ohne echten Gewinn.

> ### ✅ Gate D2
> SFT konvergiert · Checkpoint per Action Error + Sim-Sichtprüfung gewählt · Checkpoint lokal verfügbar.

---

## Phase E — Deployment @ 30 Hz *(gemeinsam — beide Zweige laufen hier zusammen)*

### E1. Policy-Adapter-Interface definieren — **einmal, für beide Zweige**
- **Tun:** Roboter-Loop **einmal** schreiben, Policy dahinter austauschbar machen:
  ```
  Policy.predict(obs: {images[3], state[14]}) -> action_chunk[30, 14]
  ```
  Zwei Implementierungen (klein/lokal vs. groß/SFT) hinter derselben Schnittstelle.
- **⚠️ Achtung:** Das ist der Grund, warum die Zweigung fast nichts kostet. **Nicht zwei Deploy-Loops bauen.**

### E2. Inferenz aufsetzen
- **Tun:** `abc_minimal/fast_inference.py` (`FastInferenceGraph`) **1:1 als Kern** verwenden (bf16 + CUDA-Graph um `model.sample_actions`), plus `abc_minimal/preprocess.py` für normalize/resize-pad.
- **Optimierungen (ABC App. D):** bf16 · **Visual-Features cachen** · `torch.compile(fullgraph=True)` · Autotuning · **CUDA-Graphs**. Auf **eurer GPU (RTX 5090)** gemessen: 63 ms → **36,3 ms** pro Chunk (10 Diffusionsschritte).
- **⚠️ Achtung:**
  - **Deployment-Infra ist nicht im Repo** — den Live-Loop (Kameras → Preprocess → Inferenz → Arm) schreibt ihr selbst; der *rechenintensive Kern* ist fertig.
  - Preprocessing muss **bitgenau** wie im Training sein: Größe, Padding, Normalisierung, **Kamera-Reihenfolge im Stack**. Falsche View-Reihenfolge = stille, völlig falsche Policy.
  - Der **`task_name`/Prompt** beim Deployment muss dem Training entsprechen.
  - `fullgraph=True` schlägt bei Graph-Breaks fehl — gewollt: Synchronisationspunkte finden und entfernen.

### E3. Trockenlauf *(vor dem ersten echten Rollout)*
- **Tun:** Loop mit geladenem Modell fahren, aber **Aktionen nicht an die Arme senden** — nur loggen. Prüfen: **echte 30 Hz**, Latenz, keine Frame-Drops, plausible Aktionswerte.
- **⚠️ Achtung:** Damit validiert ihr die gesamte Kette **ohne Bewegungsrisiko**. Erst danach Arme freigeben — mit **reduzierter Geschwindigkeit, aktiven Arbeitsraumgrenzen, Kraftlimit und Hand am Not-Aus**.

### E4. Asynchrone Ausführung / Action-Prefix (RTC)
- **Tun:** Policy asynchron mit **Real-Time Chunking** ausführen (neuer Chunk konditioniert auf Prefix bereits ausgeführter Aktionen). Config-Felder: `rtc`, `rtc_prefix_length`, `rtc_inference_lead_steps`, `execute_chunk_dim` (Default 15 von 30 Chunk-Schritten).
- **⚠️ Achtung:** Prefix-Länge ist ein **Zielkonflikt**: lang = glatter, aber die Policy **ignoriert teils die Kamera** und fährt blind weiter (ABC zeigt genau das: Griff misslingt, Policy fährt trotzdem zum Behälter). Kurz = reaktiver, ruckeliger. In ABCs Test war **Prefix = 1 klar besser als 4** (4,6 vs. 3,9 Punkte) — Repo-Default ist 4, also **beides testen**.

### E5. Erst Sim, dann echt
- **Tun:** Checkpoint zuerst in **Sim** evaluieren (billig, viele Trials), dann real.
- **⚠️ Achtung:** ABC zeigt **starke Sim→Real-Korrelation** (r = 0,85 strict / 0,91 progress) → Sim ist ein verlässlicher Vorfilter. Nutzt das, statt teure Real-Trials zu verbrennen.

### E6. Sauber evaluieren
- **Tun:** **Feste Trial-Zahl** (ABC: 50/Aufgabe), **vorab definierte Fortschritts-Rubrik** (Teilpunkte, nicht 0/1), festes Zeitlimit (ABC: 120–180 s). Dokumentieren.
- **⚠️ Achtung:** Rubrik **vor** dem Evaluieren festlegen. Ohne feste Rubrik + Trial-Zahl sind Checkpoint-Vergleiche wertlos.

> ### ✅ Gate E = **Sommerziel erreicht**
> Stabiler lokaler Stack · SpaceMouse-Teleop liefert ABC-Format-Daten für 1–2 Aufgaben · ABC-DiT extern per SFT trainiert · Roboter fährt autonom @ 30 Hz · Sim läuft und korreliert.

---

## Phase F — Danach (Ausblick, nicht Sommer)

- **F1 · ABC-VLA per SFT:** Sobald der VLA-Code released ist (Roadmap „Ende Juli" überfällig → Repo beobachten). Interim mit gleicher Datenpipeline: **GR00T N1.x**, π0/openpi, OpenVLA.
  - **⚠️** VLA ist **nicht automatisch besser**: ABC nutzt DiT als Arbeitspferd; VLA war nur bei der schwersten Aufgabe klar vorn und erst bei sehr großen Batches.
- **F2 · DAgger:** Interventionsdaten (Pedal/Button → SpaceMouse übernimmt). Datenmix **80:10:10** (alt : Interventionen : restliche Episoden der neuen Runde).
  - **⚠️** **Nicht nur auf Interventionen** trainieren — verschlechtert die Policy. ABCs „Passive Leader"-Trick funktioniert mit SpaceMouse **nicht** 1:1.
- **F3 · ENPIRE:** Auto-Reset + automatische Reward-Verifikation (SAM3/BundleSDF/cuRobo, Ziel-Latenz < 150 ms) + agentische Policy-Verbesserung; FastAPI (`/start`, `/restart`, `/home`); RL nach SERL/RLPD. **Hier wird die Tiefe der Wrist-D405 gebraucht**; ggf. Top-D405 nachrüsten.
- **F4 · ASPIRE:** Coding-Agent + Skill-Library in Sim (CaP-X/MuJoCo Playground), Skills sim→real. Parallel zu F3 möglich.
- **F5 · RoboTTT:** Long-Context (8K Schritte) auf GR00T N1.7. Braucht **4 Kameras** + DAgger-Daten. Nur Post-Training veröffentlichter Checkpoints realistisch.

---

## Die 12 häufigsten Fehler

| # | Fehler | Folge |
|---|---|---|
| 1 | CAN-Adapter ohne SocketCAN-Support gekauft | Projekt blockiert am Anfang |
| 2 | Alle RealSense an einem USB-Controller | Frame-Drops, unbrauchbare Daten |
| 3 | Auto-Belichtung/Weißabgleich aktiv | Helligkeit korreliert mit Szene → giftig |
| 4 | Fensterlicht / wechselndes Licht | Policy lernt Tageszeit als Scheinvariable |
| 5 | Video mit Standard-GOP/B-Frames encodiert | Dataloader ~70× langsamer |
| 6 | Datenformat weicht ab (`.bin`-Form, Topic-Namen) | Training bricht; Neusammeln droht |
| 7 | **Overfit-Test übersprungen** | Pipeline-Fehler erst nach Wochen entdeckt |
| 8 | `vit_*`-Dims verkleinert | DINOv3-Gewichte passen nicht mehr |
| 9 | SFT mit Pretraining-LR (1e-4) statt 1e-5 | katastrophales Vergessen |
| 10 | Checkpoint über **Validation Loss** gewählt | falscher Checkpoint (Metrik unkorreliert!) |
| 11 | Preprocessing/View-Reihenfolge Deploy ≠ Training | stille Fehlfunktion, schwer zu finden |
| 12 | Zweig 1 wird zum Dauerzustand | Sommerziel (SFT groß) verfehlt |

---

## Referenz: Was 1:1 aus `amazon-far/abc` kommt

**Nutzen (nicht selbst bauen):** `abc_minimal/dit.py` (Modell) · `train.py` + `train_loop.py` + `config.py` (Training/SFT, **beide Zweige**) · `preprocess.py` · **`fast_inference.py`** (bf16+CUDA-Graph-Inferenz) · `eval_policy.py`/`viz_policy.py` (MuJoCo-Warp-Sim) · `export_mcap.py`/`export_hf_task.py`/`prepare.py` (Daten) · `assets/put_bottles/.../yam.xml` (**YAM-Modell — auch für die IK!**) · 75k-Checkpoint.

**Selbst bauen:** Teleop (SpaceMouse→IK) · Hardware-I/O (RealSense, YAM/CAN, Greifer) · Recorder→MCAP · Live-Deploy-Loop (mit Policy-Adapter, E1).

**Fehlt noch im Release:** ABC-VLA-Training · volle Checkpoints · ABCs ZMQ-Deployment-Framework.
