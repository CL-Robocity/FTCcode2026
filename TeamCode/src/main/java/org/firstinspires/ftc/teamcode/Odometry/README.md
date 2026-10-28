# Percorsi autonomi da JSON

Sistema per far seguire al robot un percorso (linee e curve di Bézier) descritto in un file JSON, senza ricompilare il codice a ogni modifica.

> **Stato:** scritto e compilato in Android Studio, **non ancora testato sul robot**. I parametri di calibrazione sono ancora segnaposto (vedi [Calibrazione](#calibrazione)).

## Come funziona

```
HUD (Rust, PC)  --->  file .json  --->  Control Hub  --->  PathLoader  --->  PathFollower  --->  motori
 disegna il             /sdcard/FIRST/    legge il file     campiona la        segue il percorso
 percorso               paths/                              curva in punti     usando l'odometria
```

L'HUD (o un file scritto a mano) **descrive** il percorso. Tutto il calcolo del movimento avviene in Java sul robot.

## File

| File | Ruolo |
|---|---|
| `Odometry/PositionFinder.java` | Odometria con 2 dead wheels + IMU. Restituisce x, y e heading. |
| `Pathing/RobotPath.java` | Il percorso già campionato (lista di punti con x, y, heading e distanza lungo il percorso). |
| `Pathing/PathLoader.java` | Legge il JSON e lo trasforma in `RobotPath`. |
| `Pathing/PathFollower.java` | Controllore: fa muovere il robot lungo il percorso. |
| `Pathing/AutoFromJson.java` | OpMode di esempio che collega tutto. |
| `Pathing/test.json` | Percorso di esempio. |

**Importante:** la riga `package` di ogni file deve corrispondere alla cartella in cui si trova. Se i file stanno in `teamcode/Pathing`, la riga è `package org.firstinspires.ftc.teamcode.Pathing;`.

## Convenzioni (valgono per codice, JSON e HUD)

- **Unità:** il codice lavora in **centimetri**. Il JSON può essere in pollici (`"units": "in"`, default) o centimetri (`"units": "cm"`): il loader converte.
- **Origine:** (0, 0) è l'angolo del campo da cui si parte. +X verso destra, +Y in avanti.
- **Heading:** in gradi nel JSON, in radianti nel codice. **0° = robot rivolto verso +Y**, positivo **antiorario**. Non è la convenzione matematica classica (0° verso +X): è quella dell'IMU FTC e del `PositionFinder`.
- **Heading "tangent":** direzione di marcia (dx, dy) → heading = `atan2(-dx, dy)`.

## Formato del JSON

```json
{
  "units": "in",
  "start": { "x": 24, "y": 24, "heading": 0 },
  "segments": [
    { "type": "line",  "to": [24, 60], "heading": "tangent" },
    { "type": "curve", "controls": [[24, 84], [48, 84]], "to": [48, 60], "heading": 90 },
    { "type": "line",  "to": [48, 24], "heading": 180 }
  ]
}
```

- `start`: posizione e heading iniziali del robot. Il `PositionFinder` parte da qui.
- Ogni segmento parte dove finisce il precedente.
- `type: "line"`: retta fino a `to`.
- `type: "curve"`: curva di Bézier. `controls` sono i punti di curvatura: **1 controllo = quadratica, 2 = cubica**, e così via. La curva non passa dai controlli, ne è solo attirata.
- `heading` (opzionale), tre modi:
    - `"tangent"`: il robot guarda nella direzione di marcia.
    - un numero (gradi): il robot ruota gradualmente fino a quell'heading lungo il segmento.
    - omesso: mantiene l'heading che aveva.
- Errori (tipo sconosciuto, heading non valido, file mancante) vengono mostrati in telemetria all'init, prima che il robot si muova.

## Utilizzo in un OpMode

```java
RobotHardware robot = new RobotHardware();
robot.init(hardwareMap, telemetry);

RobotPath path = PathLoader.load("test.json");   // da /sdcard/FIRST/paths/

PositionFinder finder = new PositionFinder(
        robot, path.startX, path.startY, path.startHeadingDeg);

waitForStart();

PathFollower follower = new PathFollower(robot, finder);
follower.run(path, this::opModeIsActive);        // bloccante
```

Per un uso non bloccante (ad esempio per fare altro mentre il robot si muove) usa `follower.start(path)` e chiama `follower.update()` a ogni ciclo del loop finché restituisce `true`. `update()` chiama già `finder.update()`: **non chiamarlo anche tu**.

Per un esempio completo vedi `AutoFromJson.java`.

## Caricare il JSON sul robot

I JSON vanno sul **Control Hub** (quello sul robot), non sul Driver Hub, in `/sdcard/FIRST/paths/`. Non esiste uno slot SD: `/sdcard` è solo il nome Android della memoria interna.

Con `adb` (USB-C al Control Hub, oppure Wi-Fi con `adb connect 192.168.43.1:5555`):

```
adb shell mkdir -p /sdcard/FIRST/paths
adb push test.json /sdcard/FIRST/paths/test.json
```

In alternativa: Android Studio → *View → Tool Windows → Device Explorer* → `sdcard/FIRST`.

Per cambiare percorso basta rifare il `push`, senza ricompilare. Il nome del file è la costante `PATH_FILE` in `AutoFromJson`.

## Calibrazione

Da fare **prima** di qualsiasi test con movimento.

Nel `PositionFinder`:

| Costante | Cosa è | Come trovarla |
|---|---|---|
| `TICKS_PER_CM` | tick dell'encoder per 1 cm | spingi il robot di 100 cm in avanti, leggi i tick e dividi per 100 |
| `PARALLEL_OFFSET` | distanza (cm) della ruota parallela dal centro, positiva a destra | misura sul robot |
| `PERPENDICULAR_OFFSET` | distanza (cm) della ruota perpendicolare dal centro, positiva in avanti | misura sul robot |
| `PARALLEL_DIR`, `PERPENDICULAR_DIR` | +1 / −1 se l'encoder conta al contrario | vedi test 1 |

Nel `PathFollower`:

| Costante | Cosa è |
|---|---|
| `FORWARD_SIGN`, `STRAFE_SIGN`, `TURN_SIGN` | ±1 per far corrispondere i comandi al movimento reale (vedi test 2) |

## Ordine dei test sul robot

1. **Solo odometria**, senza muovere i motori. Spingi il robot a mano e guarda x, y e direzione. Avanti → `y` aumenta; destra → `x` aumenta; ruotando a sinistra la direzione aumenta. Se un segno è sbagliato, correggi `PARALLEL_DIR` / `PERPENDICULAR_DIR` o l'orientamento dell'IMU. Poi calibra `TICKS_PER_CM`.
2. **Segni del drive**, a bassa potenza e con il robot sollevato: avanti deve andare avanti, strafe positivo a destra, turn positivo in senso antiorario. Altrimenti correggi i `*_SIGN`.
3. **Percorso corto e dritto** (circa 50 cm, `maxPower = 0.3`).
4. **Una curva**, poi percorsi completi.

## Parametri del follower (tuning)

Sono campi pubblici di `PathFollower`, modificabili dopo `new PathFollower(...)`.

| Parametro | Default | Significato |
|---|---|---|
| `maxPower` | 0.6 | potenza massima di traslazione |
| `minPower` | 0.12 | potenza minima, per vincere l'attrito |
| `lookaheadCm` | 20 | distanza del punto inseguito: più alto = curve più morbide ma più tagliate |
| `kPos` | 0.02 | potenza per cm di strada rimasta (decelerazione finale) |
| `kHeading` | 1.2 | guadagno della correzione dell'heading |
| `maxTurn` | 0.5 | potenza massima di rotazione |
| `posTolCm` | 2 | tolleranza di posizione a fine percorso |
| `headTolDeg` | 3 | tolleranza di heading a fine percorso |
| `timeoutSec` | 30 | tempo massimo, poi il robot si ferma |

## Problemi comuni

- **Errore "package non corrisponde":** la riga `package` non coincide con la cartella del file (vedi [File](#file)).
- **"Errore nel percorso" in telemetria:** file non trovato (controlla `adb shell ls /sdcard/FIRST/paths`) o JSON non valido.
- **Il robot va nella direzione sbagliata:** controlla i `*_SIGN` e le direzioni dei motori in `RobotHardware`.
- **La posizione deriva o è sbagliata:** calibrazione di `TICKS_PER_CM` e degli offset, segno degli encoder, orientamento dell'IMU.
- **A fine percorso il robot resta fermo con un piccolo errore:** la correzione è sotto la potenza minima. Aumenta `kHeading` oppure `posTolCm` / `headTolDeg`. Dopo `timeoutSec` l'OpMode si ferma comunque.
- **Con `"tangent"` il robot ruota mentre avanza:** è normale. Per partire già allineato, imposta l'heading di `start` uguale alla direzione del primo tratto.

## Hardware assunto

- Drivetrain mecanum con motori `lf`, `lb`, `rf`, `rb`, mix come in `StandardMvmnt` (potenza positiva = avanti).
- Due dead wheels (parallela e perpendicolare) e IMU, configurati in `RobotHardware`.
- Gli encoder delle dead wheels sono collegati alle porte `"roller"` e `"frontintake"`, condivise con roller e intake. Funziona solo se l'encoder della dead wheel è nel connettore encoder di quella porta e l'encoder del motore intake non è collegato.

## Sviluppi previsti

- HUD in Rust (egui) per disegnare i percorsi sul campo ed esportare il JSON.
- Barriere / zone vietate e controllo delle collisioni.
- Simulazione 2D (e poi 3D) del movimento.