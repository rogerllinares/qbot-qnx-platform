// ---------------------------------------------------------------------------
// LLIBRERIES UTILITZADES
// ---------------------------------------------------------------------------
#include <stdio.h>      // Funcions d'entrada i sortida estandard (printf, fprintf)
#include <unistd.h>     // API de sistemes POSIX (gestio de fitxers, usleep, etc.)
#include <pthread.h>    // Llibreria per treballar amb fils d'execucio (threads)
#include <time.h>       // Funcions per mesurar i gestionar el temps del sistema
#include <errno.h>      // errno/EINTR per distingir la interrupcio per senyal d'un error real
#include "apibasica.h"  // Capcalera de l'API especifica per al control del QBot

// ---------------------------------------------------------------------------
// DEFINICIONS DE CONSTANTS
// ---------------------------------------------------------------------------
#define DURADA 10       // Durada de l'experiment, en segons

// Els dos valors que canvien entre sessions de laboratori.
#define QBOT_IP "192.168.2.16"                        // llegir-la sempre del LCD
#define FITXER_SORTIDA "/tmp/Prueba2_v4.txt"          // sortida de dades
#define PRIO_MON 15     // Defineix la prioritat del fil Monitor (15 sobre 99)
#define T_MON 50        // Periode del Monitor en ms, igual a la cadencia del robot:
                        // mostrejar mes rapid nomes repetiria la mateixa dada

// ---------------------------------------------------------------------------
// VARIABLES GLOBALS
// ---------------------------------------------------------------------------
struct timespec t0;     // Estructura per emmagatzemar el temps d'inici de l'execucio
FILE * pDataFile;       // Punter al fitxer on s'escriuran les dades recollides
// Bandera compartida Supervisor <-> Monitor. 'volatile' impedeix que el compilador
// se la guardi en un registre i que un fil no vegi el canvi fet per l'altre; no
// dona atomicitat ni exclusio mutua, i per aixo a mes va protegida per clau_robot.
volatile BOOL mode_emergencia = FALSE;
// Mutex que protegeix la parella (mode_emergencia + ordre enviada al robot):
// comprovar la bandera I actuar en consequencia es una sola operacio indivisible.
pthread_mutex_t clau_robot = PTHREAD_MUTEX_INITIALIZER;

// ---------------------------------------------------------------------------
// 1. FIL SUPERVISOR - esdeveniments de la cua POSIX (/cua_qbot)
// ---------------------------------------------------------------------------
// Quan l'API detecta bateria baixa envia un EVENT_T a la cua; el Supervisor es
// desperta, activa el mode d'emergencia i atura el robot.
// Parametre: l'adreca de la QBotConfig que va omplir initSystem(), d'on treu
// el descriptor de la cua.
// ---------------------------------------------------------------------------
void * Supervisor (void * arg) {
    // 'cfg' es un punter: guarda l'ADRECA de la QBotConfig del main (que la
    // passa amb &cfg), no una copia. D'aqui en surt cfg->cua, el descriptor
    // que hi va escriure initSystem(); cfg->cua es (*cfg).cua abreujat.
    // Es segur perque el main fa join dels dos fils abans de tornar: la
    // struct encara viu quan el Supervisor la llegeix.
    QBotConfig *cfg = (QBotConfig *)arg;
    // Inicialitzat a CAP_EVENT: BATERIA_BAIXA val 0, o sigui que la brossa de
    // pila podria disparar una parada d'emergencia sense cap event real.
    EVENT_T event_rebut = CAP_EVENT;
    ssize_t nb;            // bytes llegits de la cua; -1 si mq_receive falla
    int i; // Comptador pel bucle de parpadeig del LED

    // Colors predefinits: vermell pur i apagat
    // Format: array de 3 doubles [R, G, B] amb valors entre 0.0 i 1.0
    double colorVermell[NUM_LED_COLORS] = {1.0, 0.0, 0.0};
    double colorApagat[NUM_LED_COLORS]  = {0.0, 0.0, 0.0};

    // El bucle es infinit a proposit: qui el termina es el main amb
    // pthread_cancel(), i per aixo cal deixar la cancellacio habilitada.
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    do {
        // mq_receive bloqueja fins que hi ha un missatge: el fil esta suspes
        // sense consumir CPU, i es aqui on mor quan rep el cancel.
        // Es comprova el retorn perque un -1 persistent convertiria aquest
        // do-while en un bucle actiu a prioritat 25, per sobre del Monitor.
        nb = mq_receive(cfg->cua, (char *)&event_rebut, sizeof(EVENT_T), NULL);
        if (nb != (ssize_t)sizeof(EVENT_T)) {
            if (nb < 0 && errno == EINTR) {
                continue;   // interromput per una senyal: reintentem, es normal
            }
            perror("[Supervisor] mq_receive");
            break;          // qualsevol altre error: sortim
        }

        if (event_rebut == BATERIA_BAIXA) {

            // Parada d'emergencia: amb el mutex agafat, canviar mode_emergencia
            // i enviar l'ordre de parada son una sola operacio.
            pthread_mutex_lock(&clau_robot);
            mode_emergencia = TRUE;
            setWheelSpeeds(0.0, 0.0);
            pthread_mutex_unlock(&clau_robot);

            printf("\n==================================================\n"
                   " [ALERTA] BATERIA CRITICA VIA CUA! Aturant el robot...\n"
                   "==================================================\n");

            // LED vermell parpellejant 10 vegades. No es te clau_robot durant
            // l'usleep, per no bloquejar el Monitor.
            for (i = 0; i < 10; i++) {
                pthread_mutex_lock(&clau_robot);
                setLED_Color(TRUE, colorVermell);
                pthread_mutex_unlock(&clau_robot);
                usleep(300000); // 300 ms ences

                pthread_mutex_lock(&clau_robot);
                setLED_Color(TRUE, colorApagat);
                pthread_mutex_unlock(&clau_robot);
                usleep(300000); // 300 ms apagat
            }
        }

    } while(1);

    return NULL;
}

// ---------------------------------------------------------------------------
// 2. FIL MONITOR - bucle periodic de lectura i registre (T_MON = 50 ms)
// ---------------------------------------------------------------------------
// Getters: retornen structs per valor (WheelPair .esquerra/.dreta, Vector3D
// .x/.y/.z) i cadascun agafa el mutexAPI internament. getRobotState() llegeix
// TOT l'estat amb un unic bloqueig i garanteix coherencia temporal entre camps.
// Setters: setWheelSpeeds(esquerra, dreta) en rad/s i setLED_Color(enable, RGB)
// amb components [0.0, 1.0]. Retornen NO_ERROR (0) / ERROR (-1).
// ---------------------------------------------------------------------------
void * Monitor(void * none){
    // Estructures de temps per al control del bucle periodic
    struct timespec interval, ts, t1;

    // -----------------------------------------------------------------------
    // VARIABLES DE LECTURA - structs que reben les dades del robot cada cicle
    // -----------------------------------------------------------------------
    // WheelPair: camps .esquerra (index 0) i .dreta (index 1) del protocol Quanser
    WheelPair wheelPosition;   // rad   - posicio angular acumulada de cada roda
    WheelPair wheelSpeed;      // rad/s - velocitat angular actual de cada roda
    WheelPair cmdVoltage;      // V     - voltatge comandat als motors
    WheelPair current;         // A     - corrent consumit per cada motor

    // Vector3D: camps .x, .y, .z
    Vector3D  accelero;        // m/s^2 - lectura de l'accelerometre triaxial
    Vector3D  gyroscop;        // rad/s - lectura del giroscopi triaxial

    double    batteryLevel;    // %     - nivell de bateria [0.0 .. 100.0]

    // -----------------------------------------------------------------------
    // VARIABLES DE CONTROL - consignes que s'envien al robot
    // -----------------------------------------------------------------------
    double LED_Color[NUM_LED_COLORS] = {0.0, 1.0, 1.0}; // Color Cyan: R=0, G=1, B=1
    double velEsquerra = 1.0; // rad/s - consigna roda esquerra
    double velDreta    = 1.0; // rad/s - consigna roda dreta

    // Retard ABSOLUT: es calcula l'instant de la propera activacio com a
    // t0 + k*T i es dorm FINS a ell, no "durant T"; aixi el temps d'execucio
    // del cicle no se suma al periode i no hi ha deriva acumulada.
    nsec2timespec(&interval, T_MON * 1000000LL);
    timespecPlusTimeInterval(&ts, &t0, &interval);

    do {
        // -------------------------------------------------------------------
        // BLOC 1 - LECTURA DE SENSORS
        // -------------------------------------------------------------------

        // Rodes: posicio, velocitat, voltatge comandat i corrent
        wheelPosition = getWheelPositions();
        wheelSpeed    = getWheelSpeeds();
        cmdVoltage    = getWheelCMDvoltages();
        current       = getWheelCurrents();

        // IMU: accelerometre i giroscopi
        accelero = getRobotAccelero();
        gyroscop = getRobotGyroscop();

        // Bateria en percentatge [0.0 .. 100.0]
        batteryLevel = getBatteryLevel();

        // -------------------------------------------------------------------
        // BLOC 2 - ENVIAMENT D'ORDRES AL ROBOT
        // -------------------------------------------------------------------
        // clau_robot garanteix que el Supervisor no canvia mode_emergencia
        // entre la comprovacio i l'enviament de les ordres.
        pthread_mutex_lock(&clau_robot);
        if (mode_emergencia == FALSE) {
            setLED_Color(TRUE, LED_Color);   // encen el LED d'usuari amb el color definit

            // Consigna: 1r parametre roda esquerra, 2n roda dreta (rad/s)
            setWheelSpeeds(velEsquerra, velDreta);
        }
        pthread_mutex_unlock(&clau_robot);

        // -------------------------------------------------------------------
        // BLOC 3 - CONTROL DE TEMPS I REGISTRE DE DADES
        // -------------------------------------------------------------------
        clock_gettime(CLOCK_REALTIME, &t1);
        // Temps transcorregut des de l'inici, en segons
        double t_rel = (timespec2nsec(&t1) - timespec2nsec(&t0)) / 1.0e9;

        // Fi de l'experiment: el fil s'acaba a si mateix, i aixo es el que
        // desperta el pthread_join(tidMON) del main.
        if (t_rel >= DURADA) {
            pthread_exit(NULL);
        }

        // Una linia de dades per cicle de 50 ms.
        fprintf(pDataFile,
                "%lf "
                "%lf %lf "   // wheelPosition: esquerra, dreta
                "%lf %lf "   // wheelSpeed:    esquerra, dreta
                "%lf %lf "   // cmdVoltage:    esquerra, dreta
                "%lf %lf %lf "  // accelero:   x, y, z
                "%lf %lf %lf "  // gyroscop:   x, y, z
                "%lf %lf "   // current:       esquerra, dreta
                "%lf\n",     // batteryLevel:  %
                t_rel,
                wheelPosition.esquerra, wheelPosition.dreta,
                wheelSpeed.esquerra,    wheelSpeed.dreta,
                cmdVoltage.esquerra,    cmdVoltage.dreta,
                accelero.x, accelero.y, accelero.z,
                gyroscop.x, gyroscop.y, gyroscop.z,
                current.esquerra, current.dreta,
                batteryLevel);

        // clock_nanosleep amb TIMER_ABSTIME dorm FINS a l'instant 'ts', no
        // 'durant'. Just despres s'avanca ts un periode: bucle sense deriva.
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);
        timespecPlusTimeInterval(&ts, &ts, &interval);

    } while(1);
    return NULL;
}

// ---------------------------------------------------------------------------
// main - arranca tot i espera que acabi
// ---------------------------------------------------------------------------
// Arrancada: t0 (abans dels fils, perque el Monitor hi calcula les seves
// activacions) -> initSystem() -> fitxer de dades -> Supervisor -> Monitor.
// Parada: join(Monitor), que acaba sol despres de DURADA segons; cancel del
// Supervisor + join, perque ell no acaba sol; closeSystem(). El join va
// despres del cancel per no tancar recursos amb un fil encara viu.
// ---------------------------------------------------------------------------
int main(void) {
    pthread_t tidMON, tidSUP;           // Identificadors dels fils Supervisor i Monitor
    pthread_attr_t attr, attrSUP;       // Atributs de planificacio de cada fil
    struct sched_param param, paramSUP; // Prioritat de planificacio de cada fil
    int err;                            // Codi de retorn de pthread_create

    // Configuracio del sistema d'esdeveniments. Es LOCAL al main: viu mes que
    // els dos fils (el main els fa join abans de tornar), aixi que passar-ne
    // l'adreca al Supervisor es segur i no cal cap variable global.
    //   ESDE_BATERIA -> nomes vigilem la bateria
    //   20.0         -> llindar d'alarma en % [0.0 .. 100.0]
    //   10           -> capacitat de la cua, en missatges
    //   cua          -> l'omple initSystem(); aqui la deixem a 0
    QBotConfig cfg = { ESDE_BATERIA, 20.0, 10, 0 };

    clock_gettime(CLOCK_REALTIME, &t0); // Guarda el temps actual com a referencia d'inici

    // 3r argument (&cfg): hi entren els camps de configuracio i en surt el camp
    // cua, el descriptor que necessita el Supervisor. Amb NULL en lloc de &cfg
    // no es rebria cap descriptor ni events.
    // Un llindar fora de [0, 100] no es un error: initSystem() avisa per stderr
    // i aplica el valor per defecte.
    if (initSystem(QBOT_IP, WHEEL_VEL_MODE, &cfg) == ERROR) {
        printf("Error: No s'ha pogut inicialitzar el sistema.\n");
        return 1; // Finalitza el programa amb codi d'error si falla la connexio
    }

    // Obre el fitxer de text en mode escriptura ("w") al directori /tmp/
    pDataFile = fopen(FITXER_SORTIDA, "w");
    if (pDataFile == NULL) {
        perror("Error al obrir l'arxiu"); // Imprimeix l'error del sistema si no es pot crear el fitxer
        // Unic cami de fallada posterior a initSystem(), i per tant l'unic que
        // ha d'alliberar el socket, la cua i el fil que initSystem() va crear.
        closeSystem();
        return 1;
    }

    // -----------------------------------------------------------------------
    // ENGEGUEM EL SUPERVISOR
    // -----------------------------------------------------------------------
    // 1. Configurem el Supervisor com a temps real (SCHED_RR)
    pthread_attr_init(&attrSUP);
    pthread_attr_setinheritsched(&attrSUP, PTHREAD_EXPLICIT_SCHED); //determina d'on treu prioritat
    pthread_attr_setschedpolicy(&attrSUP, SCHED_RR); //round robin per gestionar prioritats
    // 2. Prioritat del Supervisor = Monitor + 10. Relativa i no absoluta perque
    //    la relacio d'urgencia es mantingui si es retoca PRIO_MON.

    paramSUP.sched_priority = PRIO_MON + 10;
    // Copia la prioritat dins dels atributs. Encara no hi ha cap fil: 'attrSUP'
    // nomes es la recepta, i pthread_create la copia en el moment de crear-lo.
    pthread_attr_setschedparam(&attrSUP, &paramSUP);
    // Els quatre arguments: &tidSUP es de SORTIDA (hi escriu l'identificador del
    // fil, que despres cal per al cancel i el join); &attrSUP es la recepta;
    // 'Supervisor' sense parentesis es l'ADRECA de la funcio, no una crida; i
    // &cfg es el que arribara al fil com a 'void *arg', d'on treu cfg.cua.
    // Si aquest fil no arrenca, ningu vigila la bateria: s'avorta en lloc de
    // moure el robot sense vigilancia.
    err = pthread_create(&tidSUP, &attrSUP, Supervisor, &cfg);
    // pthread_create no fa servir errno: retorna el codi d'error directament,
    //. El tipic es EPERM, perque
    // SCHED_RR amb prioritat 25 demana privilegis.
    if (err != 0) {
        fprintf(stderr, "[ERROR] No s'ha pogut crear el fil Supervisor. Codi: %d\n", err);
        fprintf(stderr, " -> Amb prioritat de temps real cal ser root.\n");
        // Es tanca el que ja estava obert. Cap cancel ni join: el Supervisor no
        // ha arribat a neixer i el Monitor encara no existeix.
        fclose(pDataFile);
        closeSystem();
        return 1;
    }

    // -----------------------------------------------------------------------
    // ENGEGUEM EL MONITOR
    // -----------------------------------------------------------------------
    pthread_attr_init(&attr); // Inicialitza l'estructura d'atributs del fil
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED); // no hereta la planificacio del fil pare
    pthread_attr_setschedpolicy(&attr, SCHED_RR); // politica de temps real Round Robin
    param.sched_priority = PRIO_MON; // Assigna la prioritat definida (15)
    pthread_attr_setschedparam(&attr, &param); // Aplica la prioritat als atributs
    // Si el Monitor no arrenca no hi ha experiment: es tomba el Supervisor,
    // que ja esta bloquejat a la cua, i es tanca tot.
    err = pthread_create(&tidMON, &attr, Monitor, NULL);
    if (err != 0) {
        fprintf(stderr, "[ERROR] No s'ha pogut crear el fil Monitor. Codi: %d\n", err);
        fprintf(stderr, " -> Amb prioritat de temps real cal ser root.\n");
        pthread_cancel(tidSUP);
        pthread_join(tidSUP, NULL);
        fclose(pDataFile);
        closeSystem();
        return 1;
    }

    // El programa principal espera aqui fins que el fil 'Monitor' acabi, son DURADA segons
    pthread_join (tidMON, NULL);

    // El Supervisor no acaba sol: esta bloquejat per sempre a mq_receive.
    // pthread_cancel el desbloqueja (mq_receive es punt de cancelacio) i el
    // join garanteix que ja es mort abans de tancar la cua.
    pthread_cancel(tidSUP);
    pthread_join(tidSUP, NULL);

    // Missatges i neteja final
    printf("Missio completada. Tancant el sistema.\n");
    fclose(pDataFile);   // Tanca el fitxer de dades
    if (closeSystem() == ERROR) {  // Tanca la connexio amb el robot i el fil de l'API
        fprintf(stderr, "[AVIS] closeSystem ha retornat ERROR: algun recurs no s'ha alliberat correctament.\n");
    }
    return 0;           // Finalitza el programa correctament
}
