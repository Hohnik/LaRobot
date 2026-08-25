# Aufbau-Plan: Bimanuales YAM-Setup (ABC + ASPIRE + ENPIRE, optional RoboTTT)

*Stand: 2026-08-05 · Grundlage: die vier hinterlegten PDFs (ABC, ASPIRE, ENPIRE, RoboTTT)*

---

> **Kontext dieser Überarbeitung (v2):** (a) Es steht **externer schwerer Compute** zur Verfügung (für das Training großer Modelle). (b) Prof-Vorgabe für den Sommer: so viel wie möglich aus **`github.com/amazon-far/abc`** nehmen, einen **stabilen lokalen Stack** bauen, **SpaceMouse-Teleop** für 1–2 Aufgaben, **SFT** dieser Tasks (vmtl. VLA-basiert) und eine **funktionierende Sim** als *minimale* Meilensteine. Der Plan ist entsprechend umgestellt.

## 0. TL;DR / Kernempfehlungen

1. **Compute-Aufteilung ist jetzt der Schlüssel — und sie löst das alte Problem auf.** Nicht „geht/geht nicht auf dem 5090", sondern **zwei Rollen**: die **lokale Station (RTX 5090)** = Teleop, Datensammeln, Inferenz, Deployment @ 30 Hz; der **externe Compute** = Training/**SFT**/Pretraining. Das ist auch technisch zwingend: ABCs Trainingscommand ist wörtlich `torchrun --nproc-per-node 8` und braucht **8 GPUs mit je ≥80 GB VRAM (H100/H200)**. Damit werden ABC-DiT- und (später) ABC-VLA-**SFT realistisch** — auf dem externen Cluster, nicht lokal. Siehe neue §1.5.
2. **Euer „Diffusion-Policy-first"-Instinkt und die Prof-Vorgabe „SFT VLA" widersprechen sich nicht — sie konvergieren.** ABC-**DiT ist selbst eine (Diffusion-)Policy** und ist die **einzige heute im Repo lauffähige** Trainingspipeline. Also: **Schritt 1 = ABC-DiT per SFT** aus dem veröffentlichten 75k-Checkpoint (das *ist* der Diffusion-Policy-Einstieg), **Schritt 2 = ABC-VLA per SFT**, sobald dessen Code released ist. Details §7.
3. **Repo-Realität (heute, Stand 2026-08):** `amazon-far/abc` hat **released**: ABC-DiT-Trainingspipeline, Datenkonvertierung, MuJoCo-Sim mit YAM-Modell, 75k-DiT-Checkpoint (Bottles). **Noch NICHT released** (Roadmap „end of July" offen): **VLA-Trainingscode, Deployment-Infra, volle Checkpoints**. **Teleop-Code existiert gar nicht** → SpaceMouse-Teleop + Recorder baut ihr selbst (§4, §6). Vor jedem Schritt aktuellen Repo-Stand prüfen — die fehlenden Teile können jederzeit erscheinen.
4. **Betriebssystem: Ubuntu 22.04 LTS** — nicht Windows. ENPIRE nennt es explizit (App. B.2); ABC-Repo nutzt `uv` + Python 3.12 + `ffmpeg`; der ganze Stack (RealSense, CAN, cuRobo, MuJoCo, torchcodec, I2RT-API) ist Linux-first.
5. **Größte Abweichung von den Papern: ihr habt SpaceMice, nicht GELLO-Leader-Arme.** ABC/ENPIRE/RoboTTT teleoperieren alle mit GELLO (Joint-Space) oder VR. SpaceMouse liefert kartesische 6-DoF-Velocities → braucht **inverse Kinematik (IK)** → Konsequenzen für Aktionsraum, Datenformat, DAgger (§4). Bewusst zu entscheiden.
6. **Build-Reihenfolge:** ABC-Basis (Teleop → Daten → SFT DiT/VLA → Deploy → Sim) → ENPIRE-Harness → ASPIRE parallel in Sim → RoboTTT zuletzt. Details §7–§11.

---

## 1. Was jedes Paper beiträgt und wie sie zusammenpassen

Die vier Paper sind **keine Alternativen**, sondern Schichten. Wichtig zu verstehen, sonst baut man in falscher Reihenfolge.

| Paper | Was es ist | Rolle im Stack | Braucht als Voraussetzung |
|---|---|---|---|
| **ABC** | Open-Source Behavior-Cloning-Stack: Hardware-Plattform (YAM-Station), Datenformat + Dataloader (`abcdl`), Robot-Code (ZMQ), Modelle (ABC-DiT, ABC-VLA), Sim (MuJoCo), Eval | **Fundament** — Hardware, Datenpipeline, Trainings-/Deploy-Framework | Nichts. Das ist die Basis. |
| **ASPIRE** | Agentischer "Code-as-Policy"-Ansatz: ein Coding-Agent (Claude Code/Codex) schreibt+debuggt Roboterprogramme, sammelt wiederverwendbare *Skills* in einer Library, evolutionäre Suche | **Orthogonale Programmier-/Skill-Schicht** — v.a. in Sim; transferiert Skills sim→real | Perception-/Planning-APIs (SAM3, cuRobo), Coding-Agent. Läuft primär in Sim (CaP-X/MuJoCo Playground). |
| **ENPIRE** | Agentisches Self-Improvement-**Harness** auf echter Hardware: EN=Auto-Reset+Verifikation, PI=Policy-Verbesserung, R=Rollout, E=Evolution (Git-basiert, Multi-Agent-Fleet) | **Automatisierungs-Schicht um die BC/RL-Basis** — macht aus Real-World-Learning eine Optimierschleife | Eine trainierbare Policy-Basis (BC/RL) **+** Auto-Reset **+** automatische Reward-Funktion. Setzt ABC-artige Basis voraus. |
| **RoboTTT** | Long-Context-VLA (8K Zeitschritte ≈ 5 min @ 30 Hz) mit Test-Time-Training-Layern auf **GR00T N1.7**; One-Shot-Imitation aus Menschen-Video, On-the-fly-Verbesserung, DAgger-Distillation | **Fortgeschrittene Fähigkeits-Schicht auf einem VLA-Foundation-Model** | Ein VLA-Foundation-Model + schweres Pretraining (Paper: 16× GB200). Klar das Letzte/Optionale. |

**Die logische Abhängigkeitskette:** ABC (Basis) → ENPIRE (automatisiert die Basis) und ASPIRE (parallel, Sim) → RoboTTT (obendrauf, braucht VLA + viel Compute).

**Realistische Einordnung mit lokalem 5090 + externem Compute (siehe §1.5):**
- ✅ **Lokal (5090):** Teleop, Datensammeln, Sim, Inferenz, Deployment @ 30 Hz — **plus Training eines kleinen ABC-DiT (~150 M) als Diagnose-Zweig** (§7, Zweig 1).
- ⚠️ **Lokal NICHT:** ABC-DiT-2B trainieren — ≈24 GB (Gewichte + AdamW-Zustände + Gradienten) vor Aktivierungen auf einer 32-GB-Karte. *Ausführen* dagegen problemlos (36 ms/Chunk).
- ✅ **Extern (8×H100/H200):** ABC-DiT-SFT (`torchrun --nproc-per-node 8`), ABC-VLA-SFT (wenn released), größere Pretraining-Runs.
- ✅ ENPIRE-Harness (Auto-Reset, Reward, Inferenz) läuft lokal; die eigentlichen Trainingsjobs können extern laufen.
- ✅ ASPIRE in Sim (Coding-Agent + MuJoCo) → lokal ok.
- ⚠️ RoboTTT-Pretraining (16× GB200) → auch für „extern" sehr groß; realistisch nur Post-Training/Nutzung veröffentlichter Checkpoints.

---

## 1.5 Compute-Aufteilung (lokal vs. extern) & was „SFT" bedeutet

### Was ist SFT?
**SFT = Supervised Fine-Tuning** = „überwachtes Nachtrainieren". Man nimmt ein **bereits vortrainiertes Modell** und trainiert es auf **euren gelabelten Demonstrationsdaten** weiter, bis es eure Aufgabe kann. Im Robotik-Kontext sind eure Teleop-Demos genau diese Labels: zu jeder Beobachtung (Kamerabilder + Gelenkzustand) gehört die vom Menschen ausgeführte Aktion — das Modell lernt, diese Aktion vorherzusagen. **Behavior Cloning ist genau diese Form von überwachtem Lernen.**

Zur Einordnung, drei Begriffe die oft verwechselt werden:
- **Pretraining** = Modell *von Grund auf* auf riesigen, breiten Daten trainieren (ABC: 3.500 h, 8–12 H200-Nodes). Macht ihr **nicht** selbst — ihr nehmt fertige Gewichte.
- **SFT / Finetuning** = fertiges Modell auf *eure* 1–2 Tasks spezialisieren (wenige Demos, wenige GPU-Stunden). **Das ist euer Sommerziel.**
- **RL** = Policy durch Belohnung/Ausprobieren *über* das Demo-Niveau hinaus verbessern (das macht ENPIRE später, §9).

„SFT für diese Tasks (vmtl. VLA-basierte Policy)" heißt also konkret: **nehmt ein vortrainiertes VLA (bzw. zunächst ABC-DiT) und finetunt es auf euren gesammelten SpaceMouse-Demos.**

### Die zwei Compute-Rollen
| | **Lokale Station (RTX 5090, 32 GB)** | **Externer Cluster (8×H100/H200, ≥80 GB)** |
|---|---|---|
| Aufgabe | Teleop, Datensammeln, **Inferenz + Deployment @ 30 Hz**, Sim, Debug | **Training / SFT / Pretraining** |
| Warum dort | Muss physisch am Roboter sein, Echtzeit; Inferenz in ABC ist auf genau dem 5090 gemessen | ABC-Training ist wörtlich `torchrun --nproc-per-node 8`, batch 90/GPU, ~2.6–3 it/s auf H100/H200 → passt nicht auf eine GPU |
| Datenfluss | erzeugt Episoden im ABC-Format | konsumiert sie (lokal gemountet **oder** aus Cloud-Storage gestreamt) |

### Praktischer Workflow (der „Naht" zwischen lokal und extern)
1. **Lokal sammeln** → Episoden im exakten ABC-Format schreiben (§6.0).
2. **Hochladen** in Objekt-Storage (S3/GCS). *Vorteil:* ABCs Dataloader `abcdl` kann **direkt aus S3/GCS streamen** — kein manuelles Kopieren auf den Cluster nötig (ABC App. B.1).
3. **Extern trainieren/SFT** mit `torchrun --nproc-per-node 8 train.py`.
4. **Checkpoint zurückholen** auf die lokale Station.
5. **Lokal deployen** @ 30 Hz (bf16 + torch.compile + CUDA-Graphs, §7).

**Konsequenz für den Plan:** Die frühere „passt-nicht-auf-den-5090"-Sorge ist damit vom Tisch. Der limitierende Faktor ist jetzt **Datenqualität + saubere Pipeline**, nicht lokaler VRAM. Umso wichtiger: von Anfang an im **exakten ABC-Format** sammeln (sonst muss extern nachkonvertiert werden).

---

## 2. Strategie: Crawl → Walk → Run (an den Prof-Meilensteinen ausgerichtet)

Die **minimalen Sommer-Meilensteine** (Prof) sind bewusst genau die „Crawl"-Phase dieses Plans:

> **① stabiler lokaler ABC-Stack · ② SpaceMouse-Teleop für 1–2 Tasks · ③ SFT dieser Tasks (DiT jetzt, VLA sobald verfügbar) · ④ funktionierende Sim.**

Warum diese Reihenfolge zwingend ist, in einem Satz: **Ihr braucht zuerst die verifizierte Ende-zu-Ende-Kette** (Teleop → Daten → SFT → 30-Hz-Deploy + Sim), bevor die agentischen Schichten (ENPIRE/ASPIRE) etwas zum Automatisieren haben.

```
Phase 0  Hardware + OS + Treiber + ABC-Repo lauffähig     (§3, §5, §6.0)   ~1–2 Wochen
Phase 1  SOMMERZIEL: 1–2 Tasks, SpaceMouse-Teleop,        (§7)             ~4–8 Wochen
         SFT (ABC-DiT), Deploy @ 30 Hz, ABC-Sim läuft                      ← Prof-Meilensteine ①–④
         └─ Zweig 1 (klein/lokal, Diagnose ≤2 Wo) ──┬── Deploy @ 30 Hz
            Zweig 2 (groß/extern, SFT = Ziel)    ──┘
Phase 2  ABC-VLA-SFT (sobald Code da) + Architektur-Feintuning (§8)         danach
Phase 3  ENPIRE-Harness (Auto-Reset/Reward/Self-Improve)  (§9)             später
Phase 4  ASPIRE (Sim, parallel möglich)                   (§10)            parallel
Phase 5  RoboTTT (optional, obendrauf)                    (§11)            zuletzt
```

---

## 3. Hardware — vollständige Stückliste

### 3.1 Der "ABC-PC" (exakte Referenz aus ENPIRE App. B.2, Tab. 1)

| Komponente | Spezifikation | Anmerkung |
|---|---|---|
| **GPU** | 1× NVIDIA RTX 5090, 32 GB | Blackwell. Inferenz in ABC/RoboTTT auf genau dieser Karte gemessen. Reicht für Training single-task + Deploy. |
| **CPU** | Intel Core Ultra 9 285K, 24 Kerne | CPU-Last kommt v.a. vom Video-Decoding im Dataloader (§6.3) und den ZMQ-Nodes. |
| **RAM** | 128 GB | Für Dataloader-Worker + Frame-Buffer wichtig. |
| **OS** | Ubuntu 22.04 LTS | siehe §5 |
| **GPU-Stack** | NVIDIA-Treiber + CUDA (Referenz: Treiber 595.x, CUDA 13.2) | **Praktisch: nimm den aktuellsten stabilen Treiber/CUDA, der Blackwell/RTX-5090 (sm_120) unterstützt, und ein PyTorch-Build dafür.** Die Paper-Versionen sind Referenz, nicht Pflicht. |

### 3.2 Was zusätzlich zum PC gebraucht wird (in den Papern verteilt / implizit)

| Kategorie | Konkret | Warum / Quelle |
|---|---|---|
| **Arme** | 2× I2RT YAM (6-DoF + 1-DoF Parallel-Greifer = 7 aktuierte Gelenke/Arm, 14 gesamt), brushless Aktuatoren über **CAN-Bus** | ABC C.1, ENPIRE B.2. Station-Kosten lt. ABC ~$8.000. |
| **CAN-Interface** | USB-CAN- oder PCIe-CAN-Adapter (I2RT-Doku beachten) | Die Arme kommunizieren über CAN; PC braucht ein CAN-Interface. Low-Level-Controller läuft mit **100 Hz** über CAN (ENPIRE B.3). |
| **Kameras** | 3 Views: **2× D405 an den Handgelenken** (habt ihr ✅) + **1× obere/third-person Kamera** — hier reicht eine **normale RGB-Kamera** (habt ihr ✅), keine D405 nötig. 30 Hz. | ABC C.1, ENPIRE B.4. D405 = 7–50 cm + Global Shutter → ideal fürs bewegte Handgelenk. ABC-DiT ist **RGB-only** → obere Kamera braucht keine Tiefe (§3.3). Optional 1× D435i seitlich (nur bestimmte Tasks). |
| **USB** | Mehrere unabhängige USB-3-Controller/Hubs | 3× RealSense sind bandbreitenhungrig; nicht alle an einen Hub. Ggf. PCIe-USB-Karte nachrüsten. |
| **Teleop-Controller** | **2× 3Dconnexion SpaceMouse** (eine pro Arm) | *Euer* Setup. **Abweichung von den Papern** (die nutzen GELLO-Leader-Arme). Siehe §4. |
| **Foot-Pedal** (optional) | Handelsübliches USB-Fußpedal | Für Moduswechsel bei DAgger-Interventionen (ABC App. F). |
| **Enclosure** | Käfig, an 3 Seiten weiße Wände | ABC C.1 — reduziert Hintergrund-Varianz, isoliert Fein-Manipulation. |
| **Storage** | 2–4 TB NVMe-SSD (schnell) | Videodaten sind groß. Für Single-Task reichen ~100 GB, aber Puffer + mehrere Experimente + Checkpoints füllen schnell. NVMe wegen Dataloader-Durchsatz. |
| **Netzteil** | ≥ 1000 W (RTX 5090 ~575 W Board-Power) | Damit GPU + 24-Core-CPU + Peripherie stabil laufen. |
| **Visualisierung** | (Software) Viser — Browser-basiert | ENPIRE B.2: Echtzeit-3D-Ansicht von Roboter-State, Kamera, Zielposen; für Monitoring/Kalibrierung/Debug. |

### 3.3 Kritische Hardware-Entscheidungen

- **Ein Käfig oder frei?** ABC empfiehlt Käfig für Training (weniger Varianz). Policies transferieren teils trotzdem nach draußen. → **Empfehlung: mit Käfig starten.**
- **CAN-Interface-Wahl:** hängt von der I2RT-YAM-Doku ab — vor Kauf verifizieren (welcher Adapter, welche Baudrate, SocketCAN-Support unter Linux). Das ist der wahrscheinlichste "Stolperstein" beim Erstaufbau.
- **RealSense-USB-Topologie:** vorab planen. 3 Kameras @ 30 Hz mit Tiefe können einen einzelnen USB-Controller sättigen → Frame-Drops. Getrennte Controller einplanen.
- **Obere Kamera = normale RGB-Kamera ist ok** (keine D405 nötig). ABC-DiT braucht 3 Views (top + 2 wrist; DINOv3-Encoder geteilt), aber die Policy ist **RGB-only** und die Top-Kamera steht fest → Global-Shutter/Tiefe dort unkritisch. **Bedingungen:** stabile **30 fps**, sauber ins MCAP-`/top-camera`-Topic synchronisiert, **Belichtung/Weißabgleich fixiert**, und **identische Kamera+Montage+Settings bei Collection und Deployment** (kein Domain-Shift — das ist die eigentliche Regel). Da ihr SFT auf euren Daten macht, lernt das Modell eure Top-Kamera; der Pretrained-Checkpoint-Startvorteil für den Top-View ist geringer, aber unkritisch.
- **Depth: von ABC-DiT ungenutzt, später wertvoll.** Die BC-Policy ist **RGB-only** (`export_mcap.py` loggt nur RGB-Topics) → MVP nimmt nur RGB auf. Die Depth-Fähigkeit der Wrist-D405 ist aber für **ENPIRE/ASPIRE** (SAM3/BundleSDF/cuRobo: Auto-Reset, Reward, Grasping = RGBD) nötig → depth-fähige D405 an den Armen sind genau richtig, **nicht** gegen RGB-Kameras tauschen. Mit normaler Top-Kamera fehlt euch **Top-Down-Tiefe** — für BC egal, für die spätere Perception-Phase ggf. eine D405 oben **nachrüsten** (kein Blocker, Wrist-Tiefe bleibt). Optional Depth schon jetzt parallel mitloggen (Nachsammeln ist teuer; ABCs RGB-Pfad ignoriert sie).

### 3.4 Visuelle Umgebung (Tisch, Licht, Hintergrund) — wichtig für Datenqualität

**Grundprinzip:** Eine BC-Policy lernt Bild → Aktion. Die *spezifische* Tischfarbe/Helligkeit ist theoretisch beliebig (das Netz lernt jede feste Erscheinung), **aber** die eiserne Regel ist **Trainingsverteilung = Deployment-Verteilung**. Konstant & identisch = leichter zu lernen, robuster, weniger Daten. Genau darum baut ABC einen **weißen Käfig** (C.1: reduziert Hintergrund-Varianz, isoliert Fein-Manipulation). Der Feind ist nicht „falsche Farbe", sondern **Domain-Shift zwischen Sammeln und Einsatz** + **Scheinkorrelationen**.

**Konkret umsetzen:**
- **Kamera-Belichtung + Weißabgleich fixieren** (kein Auto-Modus) — sonst ändert sich die Erscheinung korreliert mit dem Aufgabenzustand.
- **Konstantes, diffuses Kunstlicht**; **kein Fenster-/Tageslicht** (Tageszeit wird sonst Scheinvariable), kein Flackern, weiche/stabile Schatten (keine harten Punktstrahler, die mit dem Arm wandernde Schatten werfen).
- **Tisch: matt (nicht glänzend)**, einfarbig, mit **Kontrast zu den Objekten** (nicht in Objektfarbe). Glänzende Flächen → wandernde Glanzlichter + Überbelichtung.
- **Sauberer, statischer Hintergrund** (keine durchlaufenden Personen/wechselnden Objekte). Der Käfig/weiße Wände erledigen das.
- **Alle Demos unter gleichen Bedingungen** sammeln — nur Objektpositionen/Aufgabenzustand sollen variieren, nicht Licht/Hintergrund. (ABC warnt explizit vor Scheinkorrelationen zwischen unterschiedlich gesammelten Daten.)
- **Identisch bei Collection UND Deployment.** Das ist die eigentliche Regel.

**Trade-off (später):** Robustheit gegen wechselndes Licht/Tisch/Hintergrund braucht *bewusst eingestreute* Variation (Domain Randomization) + deutlich mehr Daten → separates, späteres Ziel, nicht der Sommer-MVP.

---

## 4. Die zentrale Design-Entscheidung: SpaceMouse & Aktionsraum

Das ist der wichtigste Punkt, an dem euer Setup von den Papern abweicht. Hier die Fakten und meine Empfehlung.

### 4.1 Was die Paper machen vs. was ihr habt
- **ABC/ENPIRE/RoboTTT:** Teleop über **GELLO-Leader-Arme** (billige passive Kopien der Arme; Leader-Gelenkwinkel → Follower-Gelenkwinkel, **direkter Joint-Space**). ABC-Policy: Input = 14D-Gelenk-Propriozeption; Output = **30-Schritt-Chunk absoluter Gelenk-Zielpositionen** @ 30 Hz, z-Score-normalisiert.
- **Ihr:** 2× **SpaceMouse** = kartesischer 6-DoF-Twist (Translation + Rotation des Endeffektors) + Buttons. Das ist **kein** Joint-Space.

### 4.2 Konsequenz: ihr braucht IK im Teleop-Loop
SpaceMouse-Twist → integrieren zu Ziel-EE-Pose → **IK** → Gelenkziele → Arm kommandieren. Für die IK nennen die Paper direkt nutzbare Werkzeuge:
- **mink** (MuJoCo-basierte IK, von ABC für DAgger verwendet), oder **PyRoKi**, oder **cuRobo** (GPU, collision-aware; ENPIRE nutzt es).
- **Empfehlung: `mink`** für den Teleop-Loop (leichtgewichtig, ABC nutzt es bereits), cuRobo später für kollisionsbewusste Auto-Resets in ENPIRE.

### 4.3 Aktionsraum-Entscheidung (bewusst treffen!)
Zwei valide Optionen:

**Option A — Joint-Space (ABC-kompatibel):** Policy sagt Gelenkziele voraus (wie ABC). SpaceMouse ist nur *Eingabegerät*; beim Sammeln loggt ihr die per-IK erzeugten Gelenkziele. Vorteil: 1:1-kompatibel mit ABC-Modellen/Dataloader/Deploy. Nachteil: IK-Qualität beeinflusst Datenqualität.

**Option B — Kartesischer Aktionsraum (EE-Delta):** Policy sagt EE-Posen/-Deltas voraus (wie die klassische Diffusion Policy von Chi et al.), IK erst beim Deployment. Ergonomisch näher am SpaceMouse. Nachteil: weicht vom ABC-Datenformat ab.

> **Meine Empfehlung:** **Beim Sammeln ALLES loggen** — SpaceMouse-Input, resultierende EE-Pose *und* die per-IK erzeugten Gelenkwinkel/-ziele. Dann könnt ihr den Aktionsraum pro Experiment wählen, ohne neu zu sammeln. Für den **MVP: Option A (Joint-Space)**, weil damit später der Sprung zur vollen ABC-Architektur nahtlos ist.

### 4.4 Greifer & Bimanualität
- Jede SpaceMouse steuert **einen** Arm (6-DoF). Bimanuell = beide gleichzeitig, ein Operator oder zwei.
- **Greifer:** SpaceMouse-Buttons → open/close. YAM-Greifer läuft **kraftbegrenzt** (torque-limited compliant grasp, ENPIRE B.3) — das ist zentral für robustes, sicheres, unbeaufsichtigtes Greifen. Greifer-Kommando als zusätzliche Aktions-Dimension mitloggen.

### 4.5 DAgger-Warnung
ABCs DAgger-Trick ("Passive Leader Intervention", App. F) nutzt die **FK der GELLO-Leader-Arme** — das habt ihr mit SpaceMouse **nicht**. Eure DAgger-Intervention muss anders laufen: Pedal/Button → SpaceMouse übernimmt → ihr loggt die Übernahme-Aktionen. Funktional äquivalent, aber die konkrete ABC-Implementierung ist nicht 1:1 übertragbar. Für den MVP irrelevant (DAgger kommt später).

---

## 5. Betriebssystem & Basis-Software (Installationsreihenfolge)

**OS: Ubuntu 22.04 LTS** (exakt ENPIRE-Referenz). Optional Low-Latency-Kernel für den 100-Hz-CAN-Loop.

Reihenfolge (jeder Schritt vor dem nächsten verifizieren):

1. **Ubuntu 22.04 LTS** frisch installieren. Secure Boot ggf. aus (vereinfacht NVIDIA-Treiber + CAN-Module).
2. **NVIDIA-Treiber** (aktuell + Blackwell-fähig) + **CUDA-Toolkit** (Referenz 13.2; praktisch der neueste stabile mit sm_120). Verifizieren: `nvidia-smi`.
3. **Python-Umgebungsmanager**: `uv` oder `conda`/`mamba` oder `pixi`. Pro Projekt eigene Env.
4. **PyTorch** mit passendem CUDA-Build (Blackwell-Support prüfen). Verifizieren: `torch.cuda.is_available()`, kleiner GPU-Matmul.
5. **librealsense2** + `pyrealsense2`. Verifizieren: `realsense-viewer`, alle 3 Kameras @ 30 Hz + Tiefe ohne Drops.
6. **SocketCAN** einrichten + CAN-Interface testen (`ip link`, `candump`). Dann **I2RT `i2rt` Python-API** (github.com/i2rt-robotics/i2rt) — Arme einzeln bewegen/auslesen.
7. **SpaceMouse**: `spacenavd` (libspnav) oder `pyspacemouse`. Beide Mäuse gleichzeitig auslesen testen.
8. **ffmpeg** (H.264) + **torchcodec** (für den ABC-Dataloader, §6.3).
9. **MuJoCo** + **mink** (IK). Optional **Blender** (High-Fidelity-Renders in ABC-Sim).
10. **ZeroMQ** (`pyzmq`) — Backbone des ABC-Robot-Frameworks (§6).
11. **ffmpeg/torchcodec-Encoding-Test**: kurzes Video mit GOP=30, `+faststart`, ohne B-Frames encodieren + zufälligen Frame laden (§6.3).

Erst wenn 1–7 einzeln laufen, die Nodes zusammenschalten (§6).

---

## 6. Software-Architektur (ABC-Robot-Framework)

### 6.0 Was `amazon-far/abc` liefert — auf Datei-Ebene (verifiziert am Repo-Baum, Stand 2026-08)

Der **gesamte** Code-Umfang des Repos: `abc_minimal/` + Top-Level-Launcher + Bottles-Sim-Assets. **Es gibt keinerlei Roboter-I/O** (kein ZMQ, CAN, RealSense, Teleop, FastAPI) — das Paket heißt `abc_minimal`.

**1:1 übernehmbar (die schweren Teile — Modell, Training, Daten, Sim, Inferenz-Kern):**

| Baustein | Datei(en) | Nutzung |
|---|---|---|
| ABC-DiT-Modell | `abc_minimal/dit.py` + `third_party/{dinov3,clip}` | 1:1 |
| Training / SFT | `train.py` + `abc_minimal/train_loop.py` + `config.py` | 1:1, extern (`torchrun --nproc-per-node 8`) |
| Preprocessing | `abc_minimal/preprocess.py` (normalize, resize-pad) | 1:1 (train + inference) |
| **Inferenz-Kern (Deployment!)** | `abc_minimal/fast_inference.py` — `FastInferenceGraph` (bf16 + CUDA-Graph um `model.sample_actions`) | **1:1 als Herz des `policy_node`** |
| Sim-Eval | `eval_policy.py` + `abc_minimal/eval_policy.py` (MuJoCo-**Warp**, GPU) | 1:1 |
| Visualisierung | `viz_policy.py` | 1:1 |
| Datenkonvertierung | `export_mcap.py`, `export_hf_task.py`, `prepare.py` | 1:1 |
| Sim-Szene + **YAM-Modell** | `assets/put_bottles/…/yam.xml`, `scene.xml` | 1:1 — auch als **Roboter-MJCF für eure mink-IK** |
| 75k-DiT-Checkpoint (Bottles) | `uv run prepare.py --checkpoint` | **SFT-Startpunkt** |

**Selbst zu bauen (nur die Echtzeit-Klempnerei — nicht die ML-Seite):** Teleop (SpaceMouse→IK), Hardware-I/O (RealSense, YAM/CAN via i2rt-API, Greifer), **Recorder → MCAP** (§6.1), Online-Deploy-Loop (Live-Obs → `FastInferenceGraph` → Arm @ 30 Hz). **Noch nicht released, ggf. abwarten:** ABC-VLA-Trainingscode, volle Checkpoints, ABCs eigenes ZMQ-Deployment-Framework.

**Setup-Basics:** `uv` · Python 3.12 · `sudo apt-get install -y ffmpeg` · DINO-Backbone (~8 GB) separat laden. Referenz-Loss ~0.048 nach 75k Steps.

### 6.1 Die MCAP-Schnittstelle = euer wichtigster Integrations-Hebel

`export_mcap.py` legt das **exakte Topic-Schema** der ABC-Daten fest. **Schreibt euer Recorder MCAP mit genau diesen Topics, funktioniert die ganze Daten→Training→Eval-Hälfte unverändert:**
```
Kameras:  /top-camera (640×480 mono)   /left-wrist-camera   /right-wrist-camera
States:   /left-arm-state (6)  /left-ee-state (1)  /right-arm-state (6)  /right-ee-state (1)
Actions:  /left-arm-action (6) /left-ee-action (1) /right-arm-action (6) /right-ee-action (1)
```
Zwei Konsequenzen:
1. **Aktionsraum ist bestätigt Joint-Space** (6 Armgelenke + 1 Greifer/Arm, als *action* geloggt) → eure SpaceMouse→IK-Kette loggt am Ende genau `/*-arm-action` + `/*-ee-action`. Damit ist §4.3 entschieden: **Joint-Space, ABC-nativ.**
2. **Video-Encoding-Rezept steht wörtlich in `export_mcap.py`**: `libx264 -crf 18 -bf 0`, GOP=30 ohne scenecut, faststart, Timebase 1/15360, PTS 512·k → 1:1 kopieren (§6.3).

Zielformat nach Konvertierung (das der Trainer liest): `episode_<uuid>/` mit `states_actions.bin` (num_steps × 28 float64), `combined_camera-images-rgb.mp4` (gestapelt, 224×224, 30 fps), `episode_metadata.json`.

### 6.2 Robot-Loop (Teleop + Deployment) — selbst zu bauen, aber schlank

ABC (App. C.2) beschreibt ein Node-System auf **ZMQ** (PUB/SUB), ROS-ähnlich aber schlanker — **dieser Code ist nicht im Release**. **Wichtig: ZMQ ist nicht Pflicht.** Für den MVP mit einer Station genügt oft ein **einziger Python-Prozess** (Kameras + SpaceMouse lesen → IK → Arme → Recorder in einer Schleife). ZMQ (oder ROS 2) lohnt erst, wenn ihr die Teile entkoppeln und mit unabhängigen Taktraten fahren wollt. **Empfehlung: single-process starten, ZMQ später.** Logische Nodes/Module (ob als Prozesse oder Funktionen):

- `spacemouse_node` ×2 → publiziert 6-DoF-Twist + Buttons
- `ik_node` (mink) → Twist → Ziel-EE-Pose → Gelenkziele
- `yam_arm_node` ×2 → CAN, 100 Hz, PD + Gravitationskompensation; Greifer kraftbegrenzt
- `realsense_node` ×3 → Frames @ 30 Hz
- `recorder_node` → loggt nach **MCAP** mit ABCs Topic-Schema (§6.1) → `export_mcap.py` konvertiert 1:1
- `policy_node` (später) → Inferenz @ 30 Hz, publiziert Aktions-Chunks
- `viser_node` → Visualisierung/Monitoring

Timing-Detail aus ABC: jeder Node hält seine Tick-Rate per `time.sleep`, für die letzten ~300 µs Busy-Spin (Präzision).

### 6.3 Video-Encoding (kritisch für Dataloader-Speed)
Das Zielformat pro Episode steht in §6.0. Für die gestapelte MP4 die ABC-Encoding-Optionen exakt einhalten (App. B.1), sonst wird der Dataloader zum Flaschenhals:
- H.264, **konstantes GOP = 30** (1 Keyframe/s bei 30 fps) → Frame-Index analytisch rekonstruierbar.
- **`+faststart`** (moov-Atom nach vorne).
- **keine B-Frames** (Frames hängen nur vom letzten Keyframe ab).
- **CFR** (constant frame rate).

### 6.4 Dataloader (`abcdl`)
Nutzt **torchcodec** mit analytisch rekonstruiertem Frame-Index → nahezu freier Random-Access (Paper: ~70× weniger gelesene Bytes/Decode). Unterstützt lokales FS und S3/GCS-Streaming; freies Mischen/Gewichten mehrerer Datenquellen (wichtig für Pretraining-Mixes + Interventions-Daten). Für den MVP reicht lokal.

---

## 7. Phase 1 — Sommerziel: 1–2 Tasks, SpaceMouse-Teleop, SFT, Sim

Ziel = die vier Prof-Meilensteine. **Erfolg hier = alles Wesentliche funktioniert.** Roter Faden: alles auf `amazon-far/abc` aufsetzen, ABC-DiT als erste SFT-Policy (Diffusion), extern trainieren, lokal deployen.

**Schritte:**
1. **Aufgabe(n) wählen:** 1–2 simple, gut resettbare Tasks mit klarem Erfolgskriterium (z.B. „Objekt in Behälter legen"). Aus ABCs Taxonomie: *Pick-and-Place*. Tipp: eine davon nah an ABCs *Bottles*-Task halten — dann ist der veröffentlichte 75k-Checkpoint ein besonders guter SFT-Startpunkt.
2. **ABC-Repo lauffähig machen** (§6.0): `uv` + Python 3.12 + `ffmpeg`; DINO-Backbone (~8 GB) laden; **Sim starten** (MuJoCo + YAM-Modell = Meilenstein ④); 75k-Checkpoint ziehen (`uv run prepare.py --checkpoint`) und mit `viz_policy.py`/`eval_policy.py` in Sim ansehen. *Das ist der schnellste „aha"-Moment und validiert eure Umgebung, noch bevor Hardware nötig ist.*
3. **Teleop-Loop bauen** (§4, §6.2): SpaceMice → IK (mink) → YAM-Arme; parallel `recorder_node`, der **nach MCAP** loggt (alles mitloggen: SpaceMouse-Input, EE-Pose, IK-Gelenkziele, Greiferzustand).
4. **Daten sammeln:** Für einfache Single-Task-SFT **~50–200 Demos** (grob 1–3 h; Objekt-/Startpositionen variieren). Dann via `export_mcap.py` ins ABC-Format konvertieren → verifizieren, dass `states_actions.bin` (28 float64) + gestapelte MP4 (§6.3) + JSON exakt stimmen.
5. **Policy = ABC-DiT — in zwei Zweigen** (kein Fremd-Framework nötig; `DiTConfig` ist voll parametrisierbar, `load_pretrained=False` ist Default):
   - **Zweig 1 „klein & lokal" (Diagnose, zeitgeboxt ≤2 Wochen):** DiT-Kopf verkleinern (`hidden_size 384`, `depth 12`, `num_heads 6`), `load_pretrained=False`, Batch 16–32, `--nproc-per-node 1` auf dem 5090. **Beginnt mit einem Overfit-Test auf 5–10 Episoden** — schlägt der fehl, ist die Datenpipeline kaputt (Zeitversatz, View-Reihenfolge, Normalisierung). Zweck: Datenqualität + Deploy-Loop validieren, **ohne** den Cluster im kritischen Pfad. ⚠️ `vit_*`-Felder **nicht** ändern (müssen zum DINOv3-ViT-B-Checkpoint passen).
   - **Zweig 2 „groß & extern" (Sommerziel):** volle Config (`1536/32/24`), **`load_pretrained=True`** vom 75k-Checkpoint, **extern** `uv run torchrun --standalone --nproc-per-node 8 train.py` (8×≥80 GB). Daten lokal → S3/GCS → Cluster streamt (§1.5). **SFT-LR 1e-5**, nicht der Pretraining-Default 1e-4 (sonst katastrophales Vergessen).
   - Beide Zweige teilen Datenformat, Deploy-Pfad (`fast_inference.py`) und Sim-Eval → Roboter-Loop nur **einmal** bauen, Policy hinter einem Adapter (`predict(obs) → action_chunk[30,14]`) austauschen.
   - **Warum lokal klein sein muss:** Der 5090 kann ABC-DiT-2B gut *ausführen* (36 ms/Chunk), aber nicht komfortabel *trainieren* (≈24 GB für Gewichte+AdamW+Gradienten vor Aktivierungen auf 32 GB).
   - ABC-DiT = DINOv3 ViT-B (geteilt über 3 Kameras) + DiT-Head mit Pooled Cross-Attention. Bilder 224×224 Letterbox, 30-Schritt-Aktions-Chunks, z-Score-Normalisierung. Trainer braucht `norm_stats.json` + eigene Mixture (Gewichte **exakt** Summe 1,0).
6. **Checkpoint-Auswahl über die richtigen Offline-Metriken** (ABC §3.4): **Validation Action Error** (L2 zum Ground-Truth-Chunk bei *fester* Diffusions-Schrittzahl) und **Training Loss** korrelieren stark mit Real-Performance; **Validation Loss ist unkorreliert** — nicht danach auswählen!
7. **Deployen @ 30 Hz lokal (5090):** Da ABCs Deployment-Infra noch fehlt (§6.0/§6.2), Inferenz selbst um euren `policy_node` bauen. Optimierungen aus ABC App. D: **bf16**, **Visual-Features cachen**, **torch.compile** (`fullgraph=True`), **CUDA-Graphs** → ABC-DiT 63 ms → 36 ms/Chunk auf dem 5090. Async-Ausführung mit Action-Prefix (**Real-Time Chunking**); Prefix-Länge = Tuning-Knopf (kürzer = reaktiver, länger = glatter, App. H.2).
8. **Evaluieren:** feste Trial-Zahl (ABC: 50/Task), Fortschritts-Rubrik (ABC App. G, Tab. 3). Erst in Sim (billig), dann real.

**Definition of Done Phase 1 (= Sommerziel):** stabiler lokaler ABC-Stack; SpaceMouse-Teleop nimmt saubere ABC-Format-Daten für 1–2 Tasks auf; ABC-DiT ist per SFT extern trainiert; Roboter führt die Task(s) autonom @ 30 Hz aus; ABC-Sim läuft und korreliert mit Real.

> **Hinweis zur „einfachen Policy zuerst"-Frage:** Sie ist mit Zweig 1 beantwortet — **ABC-DiT *ist* eine Diffusion Policy** (Diffusion Transformer + Rectified-Flow). Die relevante Achse war nie die Architektur, sondern **Stack-Komplexität** (Modellgröße + Trainingsort). Ein Fremd-Framework (LeRobot o.ä.) ist damit **nicht nötig** und würde nur ein zweites Datenformat und einen zweiten Deploy-Pfad einführen. ⚠️ Zweig 1 ist **Diagnose, nicht Ziel** — nach Gate D1 zu Zweig 2 wechseln. Operative Schritte: siehe [Setup-Anleitung.md](Setup-Anleitung.md).

---

## 8. Phase 2 — ABC-VLA per SFT + Architektur-Feintuning

Der Prof nennt „vmtl. VLA-basierte Policy". Sobald Phase 1 (DiT) steht, ist das der nächste Schritt — jetzt **realistisch, weil extern trainiert wird** (kein 5090-Limit mehr).

- **Weg A (bevorzugt, sobald verfügbar): ABC-VLA.** Gemma-3-4B-Backbone (SigLIP) + kleiner DiT-Action-Head mit **Pooled AdaLN**. ABC-Befund: AdaLN schlägt Cross-Attention/FAST als Connector; „multiple diffusion draws" (k=8) senkt die Gradienten-Varianz nahezu gratis. **Blocker:** ABC-VLA-Trainingscode ist noch nicht released (§6.0) → Repo beobachten.
- **Weg B (Interim, falls ABC-VLA noch fehlt und ihr *jetzt* ein VLA wollt): ein anderes offenes VLA per SFT** — z.B. **GR00T N1.x** (NVIDIA-offen, Basis von RoboTTT), **π0/openpi** oder **OpenVLA**. Gleiche Datenpipeline, anderer Trainingscode. So erfüllt ihr „SFT VLA" auch ohne ABC-VLA.
- **Wichtige, paper-gestützte Nuance für die Prof-Diskussion:** ABC nutzt **ABC-DiT als Arbeitspferd** (kompute-effizienter, für die meisten Tasks ausreichend); **ABC-VLA war nur bei der schwersten Aufgabe** (Kreditkarten aus Wallet) klar besser, und erst bei sehr großen Batch-Sizes. → VLA ist nicht automatisch „besser"; es lohnt sich vor allem bei sehr dexterosen/semantisch anspruchsvollen Tasks. Für 1–2 einfache Sommer-Tasks reicht DiT oft.
- **Conditioning-Features** (ABC App. H), falls relevant: Operator-ID-, Subtask-, Action-Prefix-Conditioning.
- **Sim vertiefen:** ABC zeigt **starke Sim→Real-Korrelation** (r=0.85 strict / 0.91 progress) → Design-Entscheidungen billig in Sim vorab testen. Optional Blender-Pipeline für High-Fidelity-Renders.

---

## 9. Phase 3 — ENPIRE-Harness

ENPIRE macht aus Real-World-Learning eine automatisierte Optimierschleife. Zwei Stufen:

**Stufe 1 (EN) — Umgebung aus menschlichem Feedback (einmaliger Aufwand):** Ein Coding-Agent baut per Tool-Calls:
- **Harte Safety-Constraints** (Konfigurationsraum begrenzen; Verletzung → sofort Fail + Reset).
- **Automatische Verifikation / Reward:** Binär-Reward aus wenigen Minuten Erfolgs-/Fehl-Demos synthetisiert (Vision + Propriozeption + Kraft; Ziel-Latenz <150 ms). Werkzeuge: **SAM3** (Open-Vocab-Detection), Tiefe, Kraft-Schätzung.
- **Automatischer Reset:** modulare Skills (SAM3 + BundleSDF-Pose-Tracking + **cuRobo** kollisionsfreie Planung + kraft-verifiziertes Greifen), resettet direkt an den Beginn der schwierigsten Phase.

**Stufe 2 (PIRE) — autonome Policy-Verbesserung:** Agent bekommt Schreibrechte auf eine schlanke Trainings-Codebase (BC, iteratives BC, offline/online RL mit BC-Regularisierung), liest Logs/Rollouts, stellt Hypothesen auf, tuned (Batch-Size, Update-Raten, BC-Term-Gewicht), hill-climbt die Erfolgsrate.

**Infrastruktur:** pro Station ein **FastAPI-Server** (Endpoints `/start`, `/restart`, `/home`, task-spezifisch `/avoid`,`/resume`). RL-Integration nach **SERL/RLPD** (dreistufig: Deployment / Learner / Actor; Disk-basierter Rollout-Buffer, `DiskBufferIngestor`, Datenmix RL vs. Demo). Multi-Agent/Fleet-Koordination **über Git** (Branches, cherry-pick) — für euch mit *einer* Station erstmal Single-Agent, aber die Architektur ist dieselbe.

**Voraussetzung:** funktionierende Basis-Policy (Phase 1/2) + Auto-Reset + Reward. Deshalb kommt ENPIRE nach ABC.

---

## 10. Phase 4 — ASPIRE (parallel, Sim-first)

ASPIRE ist **orthogonal** und kann parallel laufen, weil es primär in Sim arbeitet:
- Coding-Agent (Claude Code/Codex) schreibt "Code-as-Policy"-Programme, debuggt sie an **per-Primitive Multimodal-Traces** (Perception-Overlays, Grasp-Kandidaten, Motion-Plans, Kollisions-Feedback), sammelt validierte Fixes als **Skills** in einer Library, **evolutionäre Suche** über Programm-Kandidaten.
- Basis: **CaP-X** auf **MuJoCo Playground**. Skills, die in Sim entdeckt werden, transferieren als In-Context-Guidance auf den echten YAM (ASPIRE §3.6 zeigt genau das — reduziert Debug-Tokens deutlich).
- **Nutzen für euch:** Skill-Bibliothek + agentisches Debugging, das später ENPIRE-Auto-Resets und Programm-Synthese speist. ASPIRE und ENPIRE teilen sich Bausteine (CaP-X, SAM3, cuRobo, Coding-Agent).

---

## 11. Phase 5 — RoboTTT (optional, ganz zuletzt)

- Long-Context-VLA (8K Zeitschritte ≈ 5 min @ 30 Hz) via **Test-Time-Training-Layer** auf **GR00T N1.7** (Eagle-VLM + DiT). Fähigkeiten: One-Shot-Imitation aus Menschen-Video, On-the-fly-Verbesserung, Robustheit, lange Aufgaben.
- **Setup-relevant:** RoboTTT nutzt **4 RGB-Kameras** (top, bottom, **beide** Wrists) — eine mehr als ABCs Standard (plant die Montagepunkte perspektivisch mit ein). DAgger-Distillation braucht DAgger-Daten (Roboter-Aktionen + menschliche Korrekturen).
- **Compute-Realität:** Pretraining = 16× GB200 → selbst für euren externen Cluster sehr groß. Realistisch: **veröffentlichten RoboTTT/GR00T-Checkpoint post-trainieren**, nicht from scratch. Klar das letzte, optionale Ziel.

---

## 12. Offene Entscheidungen — mit meiner Empfehlung

| # | Entscheidung | Optionen | Empfehlung |
|---|---|---|---|
| 1 | Teleop-Gerät | SpaceMouse (habt ihr) / GELLO bauen | SpaceMouse für Start; GELLO später erwägen, falls sehr dexterose Tasks/DAgger nach ABC-Art wichtig werden |
| 2 | Aktionsraum | Joint-Space (ABC) / kartesisch (EE) | **Alles loggen**; MVP in **Joint-Space** (ABC-kompatibel) |
| 3 | IK-Tool | mink / PyRoKi / cuRobo | **mink** für Teleop, **cuRobo** später für kollisionsbewusste Auto-Resets |
| 4 | Erste Policy | ABC-DiT **klein/lokal** (Zweig 1) / ABC-DiT **groß/extern SFT** (Zweig 2) / Fremd-Framework | **Beides in dieser Reihenfolge**: Zweig 1 als zeitgeboxte Diagnose (≤2 Wo, Overfit-Test zuerst), dann Zweig 2 als Sommerziel. **Kein Fremd-Framework** — `DiTConfig` ist parametrisierbar (§7) |
| 5 | VLA-Weg (Phase 2) | ABC-VLA (warten) / GR00T / π0 / OpenVLA | **ABC-VLA sobald released**; sonst **GR00T N1.x** als Interim |
| 6 | Wo trainieren | lokal (5090) / extern (8×H100+) | **extern** für SFT/Training; **lokal** nur Inferenz/Deploy/Debug |
| 7 | Kameras | 3 (ABC) / 4 (RoboTTT) | 2× D405 Wrist + **1× normale RGB-Kamera Top** (ausreichend, da ABC-DiT RGB-only); Montage so planen, dass Top-D405/4. Kamera später ergänzbar |
| 8 | Coding-Agent (ENPIRE/ASPIRE) | Claude Code (Opus) / Codex (GPT-5.5) / Kimi | egal für Start; ENPIRE-Ablation fand Codex am schnellsten, aber alle funktionieren |
| 9 | Käfig | ja / nein | **ja** (weniger Varianz beim Training) |

---

## 13. Risiken & Stolpersteine (nach Wahrscheinlichkeit)

1. **CAN-Interface/YAM-Inbetriebnahme** — häufigster Erstaufbau-Blocker. I2RT-Doku vor dem Kauf des CAN-Adapters prüfen (SocketCAN-Support, Baudrate).
2. **RealSense-USB-Bandbreite** — 3 Kameras @ 30 Hz + Tiefe an einem Controller → Frame-Drops. USB-Topologie vorab planen.
3. **SpaceMouse→IK-Ergonomie** — bimanuelle kartesische Teleop ist ungewohnter als GELLO; anfangs schlechtere Datenqualität. Üben; simple Tasks zuerst.
4. **Video-Encoding falsch** (kein konstantes GOP / mit B-Frames) → Dataloader wird zum Flaschenhals. Genau die ABC-Optionen nutzen (§6.3).
5. **Checkpoint-Auswahl über Val-Loss** — ABC zeigt: Val-Loss ist **unkorreliert** mit Real-Performance. Nutzt **Validation Action Error** + Training Loss.
6. **Falsches Datenformat** — Recorder erzeugt nicht exakt `states_actions.bin` (28 float64) + gestapelte MP4 + JSON → `export_mcap.py`/`train.py` brechen. Früh gegen ein Mini-Sample verifizieren (§6.0).
7. **Lokal/Extern-Naht** — Daten-Upload, Env-Parität (CUDA/PyTorch/`uv`-Lock lokal vs. Cluster), Checkpoint-Rückweg. Von Anfang an über S3/GCS + identische `uv`-Envs lösen (§1.5).
8. **VLA-Code noch nicht da** — nicht auf ABC-VLA blockieren; mit ABC-DiT liefern (§7) und ABC-VLA/GR00T parallel vorbereiten (§8).
9. **Scope-Creep** — Versuch, den vollen Stack sofort zu bauen. → Phasen einhalten.
10. **Blackwell-Treiber/CUDA/PyTorch-Kompatibilität** — RTX 5090 (sm_120) braucht aktuelle Builds; früh verifizieren (§5, Schritt 4).

---

## 14. Konkrete nächste Schritte

- [ ] Ubuntu 22.04 LTS installieren; NVIDIA-Treiber + CUDA; `nvidia-smi` + PyTorch-GPU-Test grün.
- [ ] **ABC-Repo klonen, `uv`-Env, DINO-Backbone laden, Sim starten, 75k-Checkpoint in Sim ansehen** (Umgebung validiert, ohne Hardware).
- [ ] **Externen Cluster-Zugang testen**: `torchrun --nproc-per-node 8 train.py` auf ABC-Beispieldaten einmal durchlaufen lassen; S3/GCS-Datenpfad prüfen.
- [ ] I2RT-YAM-Doku lesen → **CAN-Adapter verifizieren/bestellen**; einen Arm über SocketCAN bewegen.
- [ ] 3× RealSense D405 an getrennten USB-Controllern; alle 3 @ 30 Hz + Tiefe ohne Drops.
- [ ] Beide SpaceMice parallel auslesen (`pyspacemouse`/`spacenavd`).
- [ ] `mink`-IK: SpaceMouse-Twist → EE-Pose → Gelenkziele → einen Arm live steuern.
- [ ] ZMQ-Node-Gerüst (§6.2) + Recorder → **MCAP**; mit `export_mcap.py` in ABC-Format konvertieren und Format gegen ein Mini-Sample verifizieren.
- [ ] 1–2 Tasks festlegen, ~50–200 Demos sammeln; `norm_stats.json` + eigene Mixture (Summe 1,0) anlegen, Val-Split trennen.
- [ ] **Zweig 1:** kleines ABC-DiT lokal — **Overfit-Test auf 5–10 Episoden zuerst**, dann Kurztraining; validiert Daten + Deploy-Loop.
- [ ] **Zweig 2:** **ABC-DiT extern per SFT** aus 75k-Checkpoint (LR 1e-5); Checkpoint per **Action Error** wählen; lokal @ 30 Hz deployen (bf16 + torch.compile + CUDA-Graphs), Trockenlauf vor dem ersten echten Rollout.
- [ ] Repo auf **VLA-/Deployment-Release** beobachten; sonst GR00T als VLA-Interim vorbereiten.

---

*Fragen offen? Die wichtigsten sind in §12 als Entscheidungen mit Empfehlung aufgeführt — v.a. Aktionsraum (§4.3), Teleop-Strategie (§4) und der DiT-vs-VLA-Weg (§8). Alles Übrige lässt sich phasenweise nachziehen.*
