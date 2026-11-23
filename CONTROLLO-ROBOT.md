# Adriano – Copia di prova della webapp (porta 3001) e "Controlla il robot"

Questa cartella è una **copia** della webapp del robot. La webapp principale (porta 3000/80, in
`~/AlterEGO_Adriano/catkin_ws/src/alterego_webapp` sul vision) **non è stata modificata**.

- Dove sta la copia sul vision: `~/webapp-troubleshoot-prova/webapp/`
- Sorgenti di lavoro sul base: `~/webapp-troubleshoot-prova/webapp-copy/src` (`src.orig` = originale non toccato, scaricato il 23/11/2026)
- Anteprima solo grafica (senza robot): `~/webapp-troubleshoot-prova/anteprima/` sul base

---

## 1. Avviare e spegnere la copia (sul vision)

La copia **non parte da sola** all'accensione del robot.

```bash
# avviare
ssh alterego-vision@192.168.0.111 'cd ~/webapp-troubleshoot-prova/webapp/src && nohup node webapp.js >> ../webapp.log 2>&1 &'

# spegnere (solo la copia, la 3000 resta accesa)
ssh alterego-vision@192.168.0.111 "pkill -f '^node webapp[.]js$'"

# log
ssh alterego-vision@192.168.0.111 'tail -f ~/webapp-troubleshoot-prova/webapp/webapp.log'
```

Aprirla dal Mac (tunnel attraverso il base):

```bash
ssh -L 3001:192.168.0.111:3001 alterego-base@10.70.114.114
# poi nel browser: http://localhost:3001
```

`node_modules` è un collegamento a quello della webapp principale (nessuna installazione).

> ⚠️ **Non usare 3000 e 3001 insieme per accendere/spegnere il robot.** Ognuna ha il suo stato
> (`src/js/state.json`). Se il robot è acceso dalla 3001, la 3000 lo crede spento, e premere Start lì
> farebbe partire i nodi due volte. Inoltre **aprire o ricaricare la 3000 spegne tutto ROS**
> (comportamento originale della 3000, vedi §6).

---

## 2. "Controlla il robot": come funziona

Pulsante nel riquadro **"Stato del robot"**. Apre una finestra con una riga per ogni funzione del robot:

| Colore | Scritta | Significato |
|---|---|---|
| 🟢 | Funziona | tutti i nodi della funzione sono attivi |
| 🔴 | Non funziona | manca almeno un nodo |
| 🟠 | Avviato due volte | la funzione gira due volte (launch o nodo doppio) |
| ⚪ | Spento | il robot non è "al lavoro" (pausa, ricarica, avvio…): è normale |

Codice:
- server: `src/troubleshoot-routes.js` (montato da `src/webapp.js`)
- pagina: `src/js/troubleshoot.js` (collegata da `src/js/main.js`)

### Cosa viene letto (solo lettura) – `GET /api/check`

Il vision si collega via SSH al base (stessa chiave della webapp principale, `~/.ssh/id_rsa`) e lancia
comandi **fissi**:

| Comando | Dove | A cosa serve |
|---|---|---|
| `rosnode list` | base | nodi attivi (il master è unico, vede anche i nodi del vision) |
| `rosparam get /AlterEgoVersion` | base | versione con cui il robot è stato acceso (usata per i riavvii) |
| `pgrep -af 'roslaunch.*(test0\|test0_hardware\|handover\|gestures\|trial-gestures)[.]launch'` | base | prove tecniche di Adriano in corso |
| `pgrep -af '/opt/ros/noetic/bin/[r]oslaunch '` | base + vision | quali launch sono attivi e quante volte |
| `pgrep -af '[_]_name:='` | base + vision | processi dei nodi ROS (per trovare i doppioni) |
| lettura di `src/js/state.json` | vision | stato della copia (spento / al lavoro / pausa…) |

Non esiste un endpoint che accetta comandi dalla pagina: tutti i comandi sono scritti nel server.

---

## 3. Le funzioni controllate (nodi ROS)

Tutti i nodi sono sotto `/robot_adriano/`. Nomi presi dai launch in
`~/catkin_ws/src/AlterEGO_Adriano` (base) e `~/AlterEGO_Adriano/catkin_ws/src` (vision).

| Funzione (nome nella pagina) | Launch | Dove | Nodi controllati | Riavvio |
|---|---|---|---|---|
| Comandi del pilota | `alterego_robot pilot.launch` | base | `socket`, `inbound_data` | ✅ (bloccato durante le prove tecniche) |
| Movimento di braccia e testa | `alterego_robot body_movement.launch` | base | `left/arm_inv_kin_main`, `right/arm_inv_kin_main`, `left/arm_inv_dyn`, `right/arm_inv_dyn`, `head/head_inv_kin`, `pitch_correction` | ✅ (bloccato durante le prove tecniche) |
| Motori delle braccia | `alterego_robot body_activation.launch` | base | `left/qb_manager`, `right/qb_manager` | ❌ Chiama il tecnico |
| Ruote ed equilibrio | `alterego_robot wheels.launch` | base | `wheels/qb_interface_node`, `wheels/lqr`, `wheels/error_handler` | ❌ Chiama il tecnico |
| Sensore di equilibrio | `alterego_robot imu.launch` | base | `imu/Sensor`, `imu/qb_interface_imu_node`, `alterego_state_publisher` | ❌ Chiama il tecnico |
| Spostamento da solo nella stanza | `alterego_navigation autonomous_nav.launch` | base | `lidar`, `amcl`, `map_server`, `map_server_obstacle`, `move_base`, `navigation`, `visualize_robot` | ❌ Chiama il tecnico |
| Espressioni del viso | `alterego_robot face_expressions.launch` | base | `face_expressions` | ✅ |
| Guarda e riconosce le persone | `alterego_face_tracking face_tracking.launch` | vision | `face_tracker`, `face_recognition` | ✅ |
| Voce (il robot parla) | `alterego_text2speech text2speech_prefetch.launch` | vision | `text2speech` | ✅ |
| Ascolto (il robot capisce) | `alterego_conversation speech2text_multilang.launch` (anche `speech2text.launch`) | vision | `speech2text` | ✅ |

Note:
- `initialpose` **non** è controllato: invia la posizione iniziale ("Ingresso") una volta e poi si chiude da solo.
- Sul vision `face_tracking.launch` avvia **sia** `face_tracker` **sia** `face_recognition`: per questo sono un'unica funzione.
- Nei conteggi dei doppioni, `qb_manager`, `arm_inv_dyn` e `arm_inv_kin_main` esistono normalmente in 2 copie (sinistra/destra).
- Perché alcune funzioni non si riavviano da qui: `lqr` (ruote) e IMU tengono il robot in equilibrio, `qb_manager` attiva i motori delle braccia, e riavviare la navigazione fa perdere la posizione nella stanza. Inoltre `docking.launch` usa anch'esso `wheels/qb_interface_node`.

---

## 4. Riavvio di una funzione – `POST /api/restart/:id`

Il pulsante **Riavvia** compare solo dove è permesso e chiede conferma
("Vuoi riavviare questa funzione? Il robot potrebbe fermarsi per qualche secondo").

Controlli fatti dal **server** subito prima di agire (anche se la pagina è vecchia):
1. ROS è acceso;
2. il robot è **al lavoro** (`pipelineState = 3`); in pausa / ricarica / avvio non si riavvia niente;
3. per *Comandi del pilota* e *Movimento di braccia e testa*: **nessuna prova tecnica in corso**
   (i kit di prova usano gli stessi nomi di nodo con `required="true"`: un nodo con lo stesso nome li chiuderebbe);
4. la funzione è davvero ferma, **oppure** è avviata due volte dallo **stesso** launch
   (un doppione che arriva da un altro launch → "Chiama il tecnico");
5. un solo riavvio alla volta.

Poi, in ordine:
1. `pkill -INT -f '[r]oslaunch <pacchetto> <launch>'` → chiude il vecchio roslaunch (e tutte le sue copie);
2. `rosnode kill <nodi della funzione ancora vivi>`;
3. rilancio, staccato dalla sessione, come fa la webapp principale:
   - base: `setsid nohup bash -c '<ambiente ROS base>; roslaunch …' > /tmp/troubleshoot/troubleshoot-<id>.log`
   - vision: stesso comando con l'ambiente del vision, log in `~/webapp-troubleshoot-prova/webapp/logs/`
4. ricontrolla fino a 6 volte (ogni 3 s) e mostra "Fatto! Ora funziona di nuovo." o "Il riavvio non è riuscito. Chiama il tecnico."

**Versione del robot**: i launch con `AlterEgoVersion` vengono rilanciati con **la stessa versione con
cui il robot è stato acceso** (`rosparam get /AlterEgoVersion`), altrimenti quella della webapp
(`export AlterEgoVersion=` nel `.bashrc` del vision, oggi **2**). Rilanciare un pezzo con una versione
diversa cambierebbe un parametro condiviso da tutti i nodi.

---

## 5. Altre modifiche della copia rispetto all'originale

| File | Modifica |
|---|---|
| `src/webapp.js` | porta **3001**; monta `troubleshoot-routes.js` |
| `src/js/main.js`, `src/js/api.js` | WebSocket su `ws://<stesso indirizzo della pagina>` (l'originale va sempre su `localhost:3000`) |
| `src/js/main.js` | **all'apertura della pagina non spegne più ROS**: legge lo stato salvato (al massimo 5 s di attesa) invece di mandare `rosnode kill -a && killall -9 rosmaster` |
| `src/js/websocket.js` | "Chiudi app" chiude solo la copia (l'originale chiude Firefox e tutto sulla porta 3000) |
| `src/js/workflow.js` | se l'accensione fallisce, la rotellina di caricamento si chiude (prima restava per sempre) |
| testi (`index.html`, `components/*`, `js/*`) | tutto in **italiano**, frasi semplici |
| grafica (`css/main.css`, `index.html`, `components/*`) | sfondo chiaro; in alto **ADRIANO** in rosso + disegno del robot (`images/adriano.svg`); niente logo AlterEGO; impaginazione per lo **schermo del robot 480×800** (entra tutto senza scorrere); volume spostato nel riquadro "Stato del robot"; il menu tecnico si apre toccando 3 volte l'intestazione |

---

## 6. Problemi trovati (nella webapp originale, non corretti nella 3000)

- **Aprire/ricaricare la pagina spegne ROS** (`overrideInitRobotState` → `rosnode kill -a && killall -9 rosmaster`). Corretto solo nella copia.
- **"Mostra1" all'accensione va perso**: viene pubblicato su `target_location` circa 3 s prima che il nodo `navigation` sia pronto (`rostopic pub -1` resta disponibile solo ~3 s). Visto nei log del master il 23/11/2026 alle 12:32.
- **"Torna in base" può essere ignorato in silenzio**: se un altro popup è aperto, `showSyncedPopup` (`api.js`) risponde subito "no" e la procedura non parte, senza errori.
- `~/kill_speech.sh` sul vision **non è eseguibile**: quel passo fallisce in silenzio (la webapp poi usa `rosnode kill`).
- La versione è **2** nel `.bashrc` del vision: se il robot è V4 va cambiata lì (vale per 3000 e 3001).
