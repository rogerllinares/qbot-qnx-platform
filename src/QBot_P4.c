// QBot_P4.c - maniobra d'esquiva d'un obstacle detectat pel LiDAR, amb un
// Supervisor per esdeveniments i un Monitor periodic.

#include <stdio.h>      // printf, fprintf, perror
#include <unistd.h>     // sleep, usleep
#include <pthread.h>    // fils POSIX i mutex
#include <time.h>       // clock_gettime, clock_nanosleep
#include <errno.h>      // errno, EINTR
#include "apibasica.h"  // interficie publica de l'API del QBot

#define QBOT_IP "192.168.2.16"                     // IP del robot, llegir-la del LCD
#define FITXER_SORTIDA "/tmp/P4_maniobra.txt"      // fitxer de dades que escriu el Monitor

#define PRIO_MON   15   // prioritat del fil Monitor (sobre 99)
#define T_MON      50   // periode del Monitor en ms
#define DURADA_MAX 40   // durada maxima de la prova en segons: tope que la para
#define TEMPS_IDLE  2   // segons quiet abans d'arrencar

// Geometria de la maniobra: gir de 90 graus a l'esquerra sobre el lloc, arc de
// 180 graus a la dreta al voltant de l'obstacle i gir de 90 graus a l'esquerra.
// Els girs sumen zero, o sigui que el robot surt amb el rumb que portava.
#define V_AVANC       0.22    // m/s de la recta i de l'arc
#define R_ARC         0.70    // m. radi de l'arc des del centre de l'obstacle
#define OMEGA_ARC     (V_AVANC / R_ARC)  // rad/s que donen radi R_ARC amb avanc V_AVANC
#define OMEGA_GIR     1.1     // rad/s del gir sobre el lloc
#define DIST_OBSTACLE 0.40    // m. llindar frontal del LiDAR (QBotConfig.distObstacle)
#define ANGLE_GIR     1.5708  // rad = 90 graus
#define ANGLE_ARC     3.1416  // rad = 180 graus

#define T_LATENCIA    0.25    // s. retard de l'actuador, descomptat dels objectius angulars

#define LIMIT_BATERIA  20.0   // % de bateria per sota del qual salta l'alarma
#define CAPACITAT_CUA  10     // missatges que hi caben a la cua

// Codis propis d'esdeveniment, continuant la numeracio de l'enum de l'API.
#define EV_ENGEGA   ((EVENT_T)(CAP_EVENT + 1))  // el posa el main per donar la sortida
#define EV_FI_GIR   ((EVENT_T)(CAP_EVENT + 2))  // el posa el Monitor en assolir l'objectiu

// Estats del Supervisor: un per moviment.
typedef enum {
    IDLE,             // quiet
    FORWARD,          // avancant recte
    ORIENT_ENTRADA,   // girant 90 graus sobre el lloc, abans de l'arc
    AVOID,            // vorejant l'obstacle (arc de 180 graus)
    ORIENT_SORTIDA,   // girant 90 graus sobre el lloc, per recuperar el rumb
    SHUTDOWN          // aturat per alarma de bateria
} MODESUP_T;

// Que vigila el Monitor a cada volta.
typedef enum {
    MON_OFF,      // res
    MON_ANGLE     // angle girat des de refOdo; en assolir-lo genera EV_FI_GIR
} MODEMON_T;

struct timespec t0;     // instant d'inici, origen de la columna de temps
FILE * pDataFile;       // fitxer on el Monitor escriu les dades

volatile MODESUP_T modeSUP = IDLE;      // estat del Supervisor; el Monitor nomes el llegeix

// Variables compartides Supervisor -> Monitor, sempre les tres sota el mateix mutex.
volatile MODEMON_T modeMON = MON_OFF;   // que ha de vigilar
Odometry refOdo;                        // pose en comencar el tram
volatile double objectiu = 0.0;         // angle objectiu en radiants
pthread_mutex_t clau = PTHREAD_MUTEX_INITIALIZER;   // protegeix les tres anteriors

mqd_t cua_p4;           // descriptor de la cua; el Monitor tambe hi ha d'enviar

// Segons transcorreguts des de t0.
double tempsAra(void) {
    struct timespec ara;
    clock_gettime(CLOCK_REALTIME, &ara);   // hora actual, amb resolucio de ns
    // Resta camp a camp; el divisor amb decimal forca la divisio en double.
    return (ara.tv_sec - t0.tv_sec) + (ara.tv_nsec - t0.tv_nsec) / 1000000000.0;
}

// Diu al Monitor que ha de vigilar i des de quina pose. modeMON s'escriu
// l'ultim, quan refOdo i objectiu ja son valids.
void armaMonitor(MODEMON_T mode, double objectiu_nou) {
    Odometry ara = getOdometry();   // fora del mutex: l'API ja te el seu

    pthread_mutex_lock(&clau);
        refOdo   = ara;
        objectiu = objectiu_nou;
        modeMON  = mode;
    pthread_mutex_unlock(&clau);
}

// Posa un esdeveniment a la cua. La fan servir el main i el Monitor.
void enviaEvent(EVENT_T ev) {
    // mq_send(cua, missatge, mida, prioritat): la cua vol bytes, d'aqui el cast
    // a char *. Torna -1 si falla.
    if (mq_send(cua_p4, (char *)&ev, sizeof(EVENT_T), 0) == -1) {
        perror("[P4] mq_send");
    }
}

// Fil Supervisor: maquina d'estats aperiodica, bloquejada a mq_receive() fins
// que arriba un esdeveniment. Rep per arg l'adreca de la QBotConfig del main.
void * Supervisor(void * arg) {
    QBotConfig *cfg = (QBotConfig *)arg;   // cast per poder llegir cfg->cua
    EVENT_T event = CAP_EVENT;   // valor neutre: evita una parada amb brossa de pila
    ssize_t nb;                  // bytes llegits de la cua; -1 si mq_receive falla

    // El bucle no acaba sol: el tomba el main amb pthread_cancel().
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    do {
        nb = mq_receive(cfg->cua, (char *)&event, sizeof(EVENT_T), NULL);
        if (nb != (ssize_t)sizeof(EVENT_T)) {
            if (nb < 0 && errno == EINTR) {
                continue;   // interromput per una senyal: es reintenta
            }
            perror("[Supervisor] mq_receive");
            break;          // qualsevol altre error: se surt
        }

        // La bateria mana sobre qualsevol estat: es mira abans del switch.
        if (event == BATERIA_BAIXA) {
            modeSUP = SHUTDOWN;
            armaMonitor(MON_OFF, 0.0);    // primer es desarma el Monitor
            setRobotVelocity(0.0, 0.0);   // i despres s'atura el robot
            printf("[%6.2f s] ALARMA DE BATERIA -> SHUTDOWN. Robot aturat.\n",
                   tempsAra());
            continue;
        }

        switch (modeSUP) {

            case IDLE:
                if (event == EV_ENGEGA) {   // sortida donada pel main
                    modeSUP = FORWARD;
                    setRobotVelocity(V_AVANC, 0.0);   // tram recte
                    printf("[%6.2f s] IDLE -> FORWARD. Recte a %.2f m/s.\n",
                           tempsAra(), V_AVANC);
                }
            break;

            case FORWARD:
                if (event == LIDAR_OBSTACLE) {   // unic punt que depen del LiDAR
                    modeSUP = ORIENT_ENTRADA;
                    // Objectiu amb el retard de l'actuador descomptat.
                    armaMonitor(MON_ANGLE, ANGLE_GIR - OMEGA_GIR * T_LATENCIA);
                    setRobotVelocity(0.0, OMEGA_GIR);   // gir sobre el lloc
                    printf("[%6.2f s] OBSTACLE -> ORIENT_ENTRADA. Gir de 90 graus.\n",
                           tempsAra());
                }
            break;

            case ORIENT_ENTRADA:
                if (event == EV_FI_GIR) {
                    modeSUP = AVOID;
                    // Objectiu negatiu: l'arc gira a la dreta i el Monitor compara amb signe.
                    armaMonitor(MON_ANGLE, -(ANGLE_ARC - OMEGA_ARC * T_LATENCIA));
                    setRobotVelocity(V_AVANC, -OMEGA_ARC);   // arc al voltant de l'obstacle
                    printf("[%6.2f s] ORIENT_ENTRADA -> AVOID. Arc de 180 graus, radi %.2f m.\n",
                           tempsAra(), R_ARC);
                }
            break;

            case AVOID:
                if (event == EV_FI_GIR) {
                    modeSUP = ORIENT_SORTIDA;
                    // Objectiu amb el retard de l'actuador descomptat.
                    armaMonitor(MON_ANGLE, ANGLE_GIR - OMEGA_GIR * T_LATENCIA);
                    setRobotVelocity(0.0, OMEGA_GIR);   // gir sobre el lloc
                    printf("[%6.2f s] AVOID -> ORIENT_SORTIDA. Gir de 90 graus.\n",
                           tempsAra());
                }
            break;

            case ORIENT_SORTIDA:
                if (event == EV_FI_GIR) {
                    modeSUP = FORWARD;
                    setRobotVelocity(V_AVANC, 0.0);   // recta amb el rumb inicial
                    printf("[%6.2f s] ORIENT_SORTIDA -> FORWARD. Rumb recuperat.\n",
                           tempsAra());
                }
            break;

            case SHUTDOWN:
                // Estat terminal: la resta d'esdeveniments s'ignoren.
            break;
        }

    } while (1);

    return NULL;
}

// Fil Monitor: cada T_MON ms llegeix l'odometria, escriu una fila al fitxer i
// mira si s'ha assolit l'objectiu armat pel Supervisor. En avisar es desarma
// ell mateix, perque els tres trams comparteixen EV_FI_GIR.
void * Monitor(void * none) {
    struct timespec interval, ts;   // periode i instant absolut del proper despertar
    Odometry o;                 // pose actual
    Odometry ref;               // copia local de refOdo
    MODEMON_T mode;             // copia local de modeMON
    double obj;                 // copia local de objectiu
    double girat;               // angle girat en aquest tram
    RobotState estat;           // rodes i bateria, per a les columnes del fitxer
    int i;
    int voltes = DURADA_MAX * 1000 / T_MON;  // periodes que caben a DURADA_MAX

    nsec2timespec(&interval, T_MON * 1000000LL);   // T_MON en ns (LL, 64 bits) passat a timespec
    timespecPlusTimeInterval(&ts, &t0, &interval); // primer despertar: t0 + interval

    for (i = 0; i < voltes; i++) {

        // Lectures de l'API, fora del mutex propi. getRobotState() agafa tot
        // l'estat amb un unic bloqueig: les seves columnes son coherents.
        o     = getOdometry();
        estat = getRobotState();

        fprintf(pDataFile, "%.3f\t%.4f\t%.4f\t%.4f\t%d\t%.3f\t%.3f\t%.2f\n",
                tempsAra(),
                o.x, o.y, o.phi,
                (int)modeSUP,
                estat.speed.esquerra, estat.speed.dreta,
                estat.battery);

        // Copia de les tres variables compartides d'una tirada: la decisio de
        // sota es pren amb un joc de valors coherent.
        pthread_mutex_lock(&clau);
            mode = modeMON;
            ref  = refOdo;
            obj  = objectiu;
        pthread_mutex_unlock(&clau);

        switch (mode) {

            case MON_OFF:
                // Res a vigilar.
            break;

            case MON_ANGLE:
                girat = o.phi - ref.phi;   // phi no embolcalla: la resta ja es el gir acumulat
                // Comparacio amb signe: objectiu positiu, gir a l'esquerra;
                // objectiu negatiu, arc a la dreta.
                if ((obj >= 0.0 && girat >= obj) ||
                    (obj <  0.0 && girat <= obj)) {
                    pthread_mutex_lock(&clau);
                        modeMON = MON_OFF;   // es desarma abans d'avisar
                    pthread_mutex_unlock(&clau);
                    enviaEvent(EV_FI_GIR);
                }
            break;
        }

        // TIMER_ABSTIME dorm fins a l'instant ts, no durant un temps. Avancar
        // ts un periode evita que una volta endarrerida arrossegui la deriva.
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);
        timespecPlusTimeInterval(&ts, &ts, &interval);   // ts = ts + interval
    }

    return NULL;
}

int main(void) {
    pthread_t tidMON, tidSUP;            // identificadors dels dos fils
    pthread_attr_t attr, attrSUP;        // atributs amb que es crea cada fil
    struct sched_param param, paramSUP;  // nomes en fem servir la prioritat
    int err;                             // codi de retorn de pthread_create

    // Configuracio de l'API: mascara d'esdeveniments, llindar de bateria,
    // capacitat de la cua i llindar d'obstacle. L'ultim camp (cua) l'omple
    // initSystem. Es local al main, que viu mes que els dos fils.
    QBotConfig cfg = { ESDE_BATERIA | ESDE_LIDAR_OBSTACLE, LIMIT_BATERIA,
                       CAPACITAT_CUA, DIST_OBSTACLE, 0 };

    // Firma a pantalla: identifica quin binari s'esta executant al robot.
    printf("=====================================================\n");
    printf(" QBot_P4 - maniobra tipus Practica 4\n");
    printf(" build %s %s\n", __DATE__, __TIME__);
    printf(" VERSIO 18-09-D: obstacle pel LiDAR (llindar %.2f m), retard de\n", DIST_OBSTACLE);
    printf("               l'actuador %.2f s descomptat\n", T_LATENCIA);
    printf("=====================================================\n");

    clock_gettime(CLOCK_REALTIME, &t0);   // origen de la columna de temps

    if (initSystem(QBOT_IP, WHEEL_VEL_MODE, &cfg) == ERROR) {   // obre els sockets i crea la cua
        printf("Error: No s'ha pogut inicialitzar el sistema.\n");
        return 1;
    }
    cua_p4 = cfg.cua;   // el Monitor i el main tambe hi han d'enviar

    pDataFile = fopen(FITXER_SORTIDA, "w");
    if (pDataFile == NULL) {
        perror("Error al obrir l'arxiu");
        closeSystem();
        return 1;
    }

    // Fil Supervisor, amb prioritat de temps real per sobre del Monitor.
    pthread_attr_init(&attrSUP);                                    // atributs per defecte
    pthread_attr_setinheritsched(&attrSUP, PTHREAD_EXPLICIT_SCHED); // no hereta la planificacio del main
    pthread_attr_setschedpolicy(&attrSUP, SCHED_RR);                // round robin de temps real
    paramSUP.sched_priority = PRIO_MON + 10;                        // relativa al Monitor, no absoluta
    pthread_attr_setschedparam(&attrSUP, &paramSUP);                // desa la prioritat als atributs
    err = pthread_create(&tidSUP, &attrSUP, Supervisor, &cfg);      // id, atributs, funcio, argument
    if (err != 0) {
        fprintf(stderr, "[ERROR] No s'ha pogut crear el fil Supervisor. Codi: %d\n", err);
        fprintf(stderr, " -> Amb prioritat de temps real cal ser root.\n");
        fclose(pDataFile);
        closeSystem();
        return 1;
    }

    // Fil Monitor, mateixos passos amb prioritat PRIO_MON.
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    param.sched_priority = PRIO_MON;
    pthread_attr_setschedparam(&attr, &param);
    err = pthread_create(&tidMON, &attr, Monitor, NULL);            // sense argument: llegeix les globals
    if (err != 0) {
        fprintf(stderr, "[ERROR] No s'ha pogut crear el fil Monitor. Codi: %d\n", err);
        fprintf(stderr, " -> Amb prioritat de temps real cal ser root.\n");
        pthread_cancel(tidSUP);
        pthread_join(tidSUP, NULL);
        fclose(pDataFile);
        closeSystem();
        return 1;
    }

    // Uns segons quiet: el fil intern de l'API ha de rebre el primer paquet del
    // robot abans que es prengui cap odometria de referencia.
    sleep(TEMPS_IDLE);
    printf("[%6.2f s] Engegant.\n", tempsAra());
    enviaEvent(EV_ENGEGA);   // dona la sortida al Supervisor

    // El main espera que el Monitor acabi les seves voltes: es el tope que para
    // la prova encara que la maniobra no avanci.
    pthread_join(tidMON, NULL);

    // Primer es tomba el Supervisor i despres es para el robot: aixi no pot
    // despertar-se amb un esdeveniment pendent i tornar a manar velocitat.
    // mq_receive es punt de cancellacio, i el join garanteix que ja es mort.
    pthread_cancel(tidSUP);
    pthread_join(tidSUP, NULL);

    // Els setters son asincrons: el fil intern transmet el valor a la propera
    // volta, per aixo s'espera abans de tancar el sistema.
    setRobotVelocity(0.0, 0.0);
    usleep(200000);

    printf("Prova acabada. Dades a %s\n", FITXER_SORTIDA);
    fclose(pDataFile);
    if (closeSystem() == ERROR) {
        fprintf(stderr, "[AVIS] closeSystem ha retornat ERROR: algun recurs no s'ha alliberat correctament.\n");
    }
    return 0;
}
