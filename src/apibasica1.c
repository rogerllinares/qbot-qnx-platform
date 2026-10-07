#include <stdio.h>       // printf, fprintf, perror
#include <stdlib.h>      // atof: la distancia que envia el pont del LiDAR
#include <string.h>      // memset
#include <unistd.h>      // close
#include <arpa/inet.h>   // inet_pton, htons
#include <sys/socket.h>  // socket, connect, send, recv, shutdown
#include <pthread.h>     // fils POSIX
#include "apibasica.h"   // interficie publica de l'API
#include <mqueue.h>      // cues de missatges POSIX
#include <signal.h>      // signal()/SIG_IGN per neutralitzar SIGPIPE
#include <time.h>        // clock_gettime: el dt real de la odometria
#include <math.h>        // cos, sin, sqrt, fabs (odometria)

//llibreries necessaries per fer les cues (mqueue.h ja esta inclos mes amunt)
#include <fcntl.h>
#include <sys/stat.h>

//Totes aquestes definicions venen de la documentacio de Quanser
#define RESEARCH_MODE 0x02 // Mode de treball: investigacio (bit definit)
#define TEACHING_MODE 0x00 // Mode de treball: docencia (sense bit d'investigacio)
#define PRIO_ACTVAR 50 // Prioritat del thread d'actualitzacio (50 / 99)
#define RESPONSE_SIZE 17 // Nombre d'elements esperats en la resposta del robot
#define REQUEST_SIZE 10 // Nombre d'elements a enviar en la peticio al robot
#define HOLD 0 // Constant 0 (Sense operacio, placeholder protocol)
#define TIMESTAMP 0 // valor del camp timestamp de la peticio (el protocol Quanser accepta 0)
#define CUA_NOM "/cua_qbot" // nom unic de la cua POSIX d'esdeveniments
#define QBOT_PORT 18888 // port TCP del driver Quanser dins del robot
#define PORT_LIDAR 18889 // port TCP del pont del LiDAR (pont_lidar.py), al mateix Jetson que el driver

// ---------------------------------------------------------------------------
// Pont del LiDAR. El pont envia una linia de text per barrido amb la distancia
// minima valida del sector frontal en metres ("0.734\n"), o "-1.000\n" si no
// veu res. El salt de linia fa de marca de trama: una lectura partida de
// recv() no pot desincronitzar res, es llegeix fins al salt de linia seguent.
// ---------------------------------------------------------------------------
#define LIDAR_MAX_LINIA    64    // una linia real fa 7 o 8 bytes
#define MARGE_REARM_LIDAR  0.10  // m per sobre del llindar per tornar a armar l'avis (anti-rebot)

// ---------------------------------------------------------------------------
// Valors per defecte usats per initSystem() quan config==NULL o un camp es
// invalid. S'apliquen camp a camp (fallback individual).
// ---------------------------------------------------------------------------
#define DEFAULT_LIMIT_BATERIA  20.0         // Llindar de bateria per defecte (%)
#define DEFAULT_CAPACITAT_CUA  10           // Capacitat de cua per defecte (missatges)
#define MAX_CAPACITAT_CUA      100          // Limit superior raonable de la cua POSIX
#define DEFAULT_DIST_OBSTACLE  0.60         // Llindar d'obstacle del LiDAR per defecte (m)
#define MAX_DIST_OBSTACLE      10.0         // Abast del M10P segons el fabricant (m)
#define MAX_VEL_RODA           15.75        // rad/s per roda en mode docencia (0.70 m/s / 0.04445 m de radi)


#define BATT_MIN_V  11.5 // shutdown del firmware (Quanser, "QBot Platform User Manual: Power", Table 1 i seccio E)
#define BATT_MAX_V  13.4

// ---------------------------------------------------------------------------
// ODOMETRIA: calibracion del biaix del giroscopo
// ---------------------------------------------------------------------------
// Un giroscopo en reposo no marca 0 exacto, y ese offset integrado hace girar
// phi solo, sin que el robot se mueva. El hilo interno lo mide promediando los
// primeros paquetes (~50 ms cada uno, o sea ~1 s) y lo resta al integrar phi.
// EL ROBOT DEBE ESTAR QUIETO MIENTRAS CORRE initSystem().
// ---------------------------------------------------------------------------
#define CAL_BIAS_MOSTRES 20   // paquetes que se promedian para medir el biaix

// ---------------------------------------------------------------------------
// VARIABLES GLOBALS
// ---------------------------------------------------------------------------


// Variables globals 'volatile' per a la comunicacio amb el robot:
//   Posicions angulars de les rodes (rad) (acumulat)
static volatile double wheelPositionAPI[NUM_WHEELS];
//   Velocitats angulars de les rodes (rad/s)
static volatile double wheelSpeedAPI[NUM_WHEELS];
//   Voltages aplicats als motors de les rodes (V)
static volatile double cmdVoltageAPI[NUM_CMD_VOLT];
//   Dades de l'accelerometre (m/s^2) en eixos [X,Y,Z]
static volatile double acceleroAPI[NUM_ACCELERO];
//   Dades del giroscop (rad/s) en eixos [X,Y,Z]
static volatile double gyroscopAPI[NUM_GYROSCOP];
//   Corrents (A) dels motors de les rodes
static volatile double currentAPI[NUM_CURRENT];
//   Nivell de bateria en Volts, tal com arriba del robot (conversio a % la fan els getters)
static volatile double batteryLevelAPI;

// Mode combinat: docencia/investigacio + mode de velocitat
static volatile double modeAPI;
//   Habilitacio del LED d'usuari (1) o no (0)
static volatile double enableUserLED_API;
// Color (RGB) per als LEDs definits en el robot
static volatile double LED_ColorAPI[NUM_LED_COLORS];
// Comandes de velocitat linear per a cada roda
static volatile double velCmdAPI[NUM_VEL_CMD];

// Resultado de la integracion que hace el hilo interno. Las escribe solo
// actualitzarVariables y las leen los getters, todo bajo mutexAPI.
//   Odometria con phi del GIROSCOPO
static volatile double odoGyroXAPI;    // m
static volatile double odoGyroYAPI;    // m
static volatile double odoGyroPhiAPI;  // rad
//   Odometria con phi de las RUEDAS
static volatile double odoRodesXAPI;   // m
static volatile double odoRodesYAPI;   // m
static volatile double odoRodesPhiAPI; // rad
//   Velocidad del cuerpo derivada del incremento de posicion (no de la
//   velocidad que reporta el robot); la retorna getRobotVelocityPos().
static volatile double velPosLinealAPI;   // m/s
static volatile double velPosAngularAPI;  // rad/s

// Flags de control d'execucio del thread:
//   arm  = el bucle continua mentre sigui TRUE. El posen a FALSE tres coses:
//          que l'aplicacio demani tancar (stop), un error de socket o que el
//          robot tanqui la connexio. A mes s'envia al robot a requestMsg[5].
static volatile BOOL arm;
//   stop = TRUE nomes quan l'aplicacio demana la parada, des de closeSystem()
static volatile BOOL stop;

// Altres variables globals de la API:
//   sockfd = descriptor del socket TCP per la comunicacio amb el robot
static int sockfd;
//   tidActVar = ID del thread d'actualitzacio de variables globals
static pthread_t tidActVar;

// LiDAR (nomes si initSystem() rep ESDE_LIDAR_OBSTACLE a la mascara):
//   lidarfd = socket TCP cap al pont del Jetson; -1 si no s'ha demanat el LiDAR
static int lidarfd = -1;
//   tidLidar = ID del fil vigilarLidar
static pthread_t tidLidar;
//   distObstacleAPI = llindar en metres. S'escriu a initSystem() abans de crear
//   el fil i nomes el llegeix vigilarLidar: no cal mutex.
static double distObstacleAPI;

// Variables per als esdeveniments

// Cua d'esdeveniments; initSystem() la copia a QBotConfig.cua
static mqd_t cua_api;


// Bandera per recordar si ja hem avisat, per no fer "spam"
static BOOL avisBateriaEnviat = FALSE;
static double llindarBateria;

// Bitmask dels esdeveniments actius (ESDE_*). Inicialitzada per initSystem()
// i llegida pel fil actualitzarVariables sota mutexAPI.
static volatile unsigned char tipusEsdevenimentsAPI;

// El mutex UNICO de la API. Protege TODAS las variables globales compartidas
// entre el hilo actualitzarVariables y las funciones publicas (getters y
// setters). NO es recursivo: ninguna seccion critica puede llamar a otra
// funcion que vuelva a bloquearlo.
static pthread_mutex_t mutexAPI = PTHREAD_MUTEX_INITIALIZER;


// ---------------------------------------------------------------------------
// FUNCIONS DE L'API
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// actualitzarVariables - cuerpo del hilo interno de la API
// ---------------------------------------------------------------------------
// Es el UNICO punto del programa que habla con el socket. Corre con SCHED_RR y
// prioridad PRIO_ACTVAR.
// No lleva temporizador propio: el ritmo lo marca el recv() bloqueante, que
// despierta cuando el robot manda su paquete (~50 ms); el resto del tiempo el
// hilo esta bloqueado y no consume CPU.
// Una vuelta: leer stop -> recv() 17 doubles -> volcar a las globales ->
// vigilar la bateria -> montar requestMsg -> send() 10 doubles -> releer arm.
// Las copias locales (stop_local, arm_local, bateria_local) sacan trabajo
// fuera de la seccion critica.
// El parametro 'none' no se usa: pthread_create exige la firma void*(void*).
// ---------------------------------------------------------------------------
void * actualitzarVariables (void *none) {
    double bateria_local;
    // Copies locals de les banderes, preses de les globals sota el mutex
    BOOL stop_local, arm_local;
    pthread_mutex_lock(&mutexAPI);
    stop_local = stop;
    arm_local  = arm;
    pthread_mutex_unlock(&mutexAPI);

    // Nombre de bytes rebuts en la darrera crida recv()
    int nb;
    // Missatge rebut del robot (buffer) de RESPONSE_SIZE valors (double)
    double responseMsg[RESPONSE_SIZE];
    // Missatge a enviar al robot (buffer) de REQUEST_SIZE valors (double)
    double requestMsg[REQUEST_SIZE];
    // Els camps 15 (watchdog) i 16 (timestamp) de la resposta no es guarden:
    // cap funcio publica els llegeix. El de la peticio va sempre a 0.

    // Estado privado de la odometria, local del hilo: en las globales solo va
    // el resultado ya integrado.
    struct timespec ara, abans = {0, 0}; // instante de este paquete y del anterior. 'abans' se
                                         // inicializa para evitar el aviso -Wmaybe-uninitialized;
                                         // solo se lee a partir del segundo paquete
    double posE_abans = 0.0;             // posicion angular de rueda izquierda del paquete anterior (rad)
    double posD_abans = 0.0;             // idem rueda derecha
    BOOL   primerPaquet = TRUE;          // en el primero no hay referencias: no se integra nada
    int    mostresBias  = 0;             // paquetes ya acumulados para el biaix
    double sumaBias     = 0.0;           // suma de gyroscopAPI[2] durante la calibracion
    double biasGyroZ    = 0.0;           // biaix medido, se resta a partir de ese momento

    // Comenca el bucle d'actualitzacio. Continua mentre 'arm' sigui TRUE.
    do {
        // Si s'activa 'stop', es posa 'arm' a FALSE per acabar el bucle.
        pthread_mutex_lock(&mutexAPI);
        if (stop_local) {
            arm = FALSE;
        }
        pthread_mutex_unlock(&mutexAPI);

        // -------------------------------------------------------------------
        // Lectura de la trama amb una sola crida. recv() NO garanteix el
        // missatge sencer: torna tan bon punt hi ha algun byte al buffer. Una
        // lectura curta es detecta mes avall ("Nombre de bytes rebuts
        // inesperat") i el paquet es descarta.
        // -------------------------------------------------------------------
        nb = recv(sockfd, responseMsg, sizeof(responseMsg), 0); // torna quan hi ha bytes, no forcosament els 17 doubles

        pthread_mutex_lock(&mutexAPI);
        stop_local = stop;
        pthread_mutex_unlock(&mutexAPI);

        if (nb==-1) {
            // si tanquem nosaltres, l'error del recv() es esperat i no s'informa
            if (!stop_local) {
                perror("Error. Hi ha hagut un problema a la xarxa");
            }
            pthread_mutex_lock(&mutexAPI);
            arm = FALSE;
            pthread_mutex_unlock(&mutexAPI);
            break;   // amb continue es tornaria a llegir un socket ja caigut
        }
        else
        if (nb==0)
        {
            // si tanquem nosaltres, el tancament remot es esperat i no s'informa
            if (!stop_local) {
                fprintf(stderr, "Connexio tancada pel remot\n");
            }
            pthread_mutex_lock(&mutexAPI);
            arm = FALSE;
            pthread_mutex_unlock(&mutexAPI);
            break;   // el remot ha tancat: amb continue es rellegiria un socket mort
        }
        else
        if (nb != sizeof(responseMsg))
        {
            // si tanquem nosaltres, la lectura curta es esperada i no s'informa
            if (!stop_local) {
                fprintf(stderr, "Error. Nombre de bytes rebuts inesperat\n");
            }
            // lectura parcial: no desarmem, nomes esperem el seguent paquet sencer
            continue;
        }

        pthread_mutex_lock(&mutexAPI);

        // Volcat de la resposta del robot a les globals (mapatge Quanser).
        // posicions angulars de roda (rad)
        wheelPositionAPI[0] = responseMsg[0];
        wheelPositionAPI[1] = responseMsg[1];
        // velocitats angulars de roda (rad/s)
        wheelSpeedAPI[0] = responseMsg[2];
        wheelSpeedAPI[1] = responseMsg[3];
        // voltatges comandats als motors (V)
        cmdVoltageAPI[0] = responseMsg[4];
        cmdVoltageAPI[1] = responseMsg[5];
        // accelerometre (m/s^2): x, y, z
        acceleroAPI[0] = responseMsg[6];
        acceleroAPI[1] = responseMsg[7];
        acceleroAPI[2] = responseMsg[8];
        // giroscop (rad/s): x, y, z
        gyroscopAPI[0] = responseMsg[9];
        gyroscopAPI[1] = responseMsg[10];
        gyroscopAPI[2] = responseMsg[11];
        // corrent de cada motor (A)
        currentAPI[0] = responseMsg[12];
        currentAPI[1] = responseMsg[13];
        // nivell de bateria en Volts
        batteryLevelAPI = responseMsg[14];
        // responseMsg[15] (watchdog) i [16] (timestamp) no es fan servir

        // -------------------------------------------------------------------
        // Integracion de la odometria y de la velocidad por posicion
        // -------------------------------------------------------------------
        // Se hace dentro de la misma seccion critica del volcado: usa los
        // valores del paquete recien recibido y escribe globales compartidas.
        // El dt es el tiempo real entre dos paquetes medido con el reloj: la
        // cadencia la marca el recv() bloqueante y varia paquete a paquete.
        clock_gettime(CLOCK_MONOTONIC, &ara);

        if (primerPaquet) {
            // No hay instante ni posicion anteriores con los que comparar:
            // solo se guardan las referencias y no se integra nada (dt = 0).
            abans        = ara;
            posE_abans   = wheelPositionAPI[0];
            posD_abans   = wheelPositionAPI[1];
            primerPaquet = FALSE;
        }
        else {
            double dt = (double)(ara.tv_sec - abans.tv_sec)
                      + (double)(ara.tv_nsec - abans.tv_nsec) / 1e9;

            if (mostresBias < CAL_BIAS_MOSTRES) {
                // Fase de calibracion: el biaix es el error fijo del giroscopo con
                // el robot quieto, media de CAL_BIAS_MOSTRES muestras tomadas
                // durante initSystem(). Mientras dura NO se integra nada.
                sumaBias += gyroscopAPI[2];
                mostresBias++;
                if (mostresBias == CAL_BIAS_MOSTRES) {
                    biasGyroZ = sumaBias / (double)CAL_BIAS_MOSTRES;
                }
            }
            else {
                // x,y salen del INCREMENTO DE POSICION de los encoders entre
                // este paquete y el anterior.
                double d_esq    = WHEEL_RADIUS * (wheelPositionAPI[0] - posE_abans); // m
                double d_dre    = WHEEL_RADIUS * (wheelPositionAPI[1] - posD_abans); // m
                double d_centre = (d_esq + d_dre) / 2.0;                             // m avanzados por el centro

                // Euler hacia adelante: se avanza d_centre en la direccion phi del
                // principio del intervalo y despues se actualiza phi. El error
                // respecto al punto medio es d_centre*dphi/2 por paso: nulo en
                // recta (dphi=0) y en giro sobre si mismo (d_centre=0).
                odoGyroXAPI   += d_centre * cos(odoGyroPhiAPI);
                odoGyroYAPI   += d_centre * sin(odoGyroPhiAPI);
                odoGyroPhiAPI += (gyroscopAPI[2] - biasGyroZ) * dt;   // phi del GIROSCOPO

                odoRodesXAPI   += d_centre * cos(odoRodesPhiAPI);
                odoRodesYAPI   += d_centre * sin(odoRodesPhiAPI);
                odoRodesPhiAPI += (d_dre - d_esq) / WHEEL_BASE;       // phi de las RUEDAS

                // La segunda version de la velocidad del cuerpo sale del mismo
                // incremento. Guarda contra dt == 0 (dos paquetes en el mismo
                // tick del reloj): se dejan los valores anteriores.
                if (dt > 0.0) {
                    velPosLinealAPI  = d_centre / dt;
                    velPosAngularAPI = (d_dre - d_esq) / (WHEEL_BASE * dt);
                }
            }

            // Referencias para la vuelta siguiente, tambien durante la calibracion.
            abans      = ara;
            posE_abans = wheelPositionAPI[0];
            posD_abans = wheelPositionAPI[1];
        }

        pthread_mutex_unlock(&mutexAPI);

        pthread_mutex_lock(&mutexAPI);
        bateria_local = batteryLevelAPI;
        pthread_mutex_unlock(&mutexAPI); // la comparacio es fa ja fora del mutex

        // Vigilant de la bateria: es genera BATERIA_BAIXA nomes si el bit
        // ESDE_BATERIA esta actiu i el nivell es <= llindar. avisBateriaEnviat
        // evita repetir l'event mentre la condicio es mante certa.
        // Sense mutex: llindarBateria i tipusEsdevenimentsAPI nomes s'escriuen
        // a initSystem abans de crear aquest fil, i avisBateriaEnviat nomes aqui.
        if ((tipusEsdevenimentsAPI & ESDE_BATERIA) && bateria_local <= llindarBateria) {
            if (avisBateriaEnviat == FALSE) {
                EVENT_T event = BATERIA_BAIXA;
                // Posa a la cua un EVENT_T amb el valor BATERIA_BAIXA; l'aplicacio el llegira.
                // (char *)&event: agafa l'adreca (&) i la tracta com un punter a bytes
                mq_send(cua_api, (char *)&event, sizeof(EVENT_T), 0);
                avisBateriaEnviat = TRUE;
            }
        }
        else {
            avisBateriaEnviat = FALSE;
        }

        pthread_mutex_lock(&mutexAPI);

        // Peticio cap al robot (10 doubles, mapatge Quanser).
        requestMsg[0] = modeAPI;            // mode docencia/investigacio + mode de velocitat
        requestMsg[1] = enableUserLED_API;  // LED d'usuari ences (1) o apagat (0)
        requestMsg[2] = LED_ColorAPI[0];    // component R [0.0, 1.0]
        requestMsg[3] = LED_ColorAPI[1];    // component G
        requestMsg[4] = LED_ColorAPI[2];    // component B
        requestMsg[5] = arm;                // mante el bucle actiu al robot
        requestMsg[6] = HOLD;               // camp fix del protocol (sempre 0)
        requestMsg[7] = velCmdAPI[0];       // velocitat roda esquerra (rad/s)
        requestMsg[8] = velCmdAPI[1];       // velocitat roda dreta (rad/s)
        requestMsg[9] = TIMESTAMP;          // timestamp de la peticio (sempre 0)
        pthread_mutex_unlock(&mutexAPI);

        // Envia el missatge de peticio complet al robot (blocking).
        int bytes_enviats = send(sockfd, requestMsg, sizeof(requestMsg), 0);
        if (bytes_enviats < 0) {
            // Es rellegeix 'stop': closeSystem() fa shutdown(sockfd) i el send()
            // pot fallar amb EPIPE per un tancament ordenat nostre, no per xarxa.
            pthread_mutex_lock(&mutexAPI);
            stop_local = stop;
            pthread_mutex_unlock(&mutexAPI);

            // si tanquem nosaltres, l'error del send() es esperat i no s'informa
            if (!stop_local) {
                perror("[ERROR XARXA] No s'han pogut enviar les ordres al robot (send)");
            }

            pthread_mutex_lock(&mutexAPI);
            arm = FALSE;
            pthread_mutex_unlock(&mutexAPI);
        }

        pthread_mutex_lock(&mutexAPI);
        arm_local = arm;
        pthread_mutex_unlock(&mutexAPI);
    } while(arm_local); // repeteix mentre el fil segueixi armat
    return NULL;
}

// Helpers interns de bateria: conversio Volts <-> percentatge amb mapping
// lineal entre BATT_MIN_V (0%) i BATT_MAX_V (100%).
static double voltsToPercentage(double volts) {
    double pct = (volts - BATT_MIN_V) / (BATT_MAX_V - BATT_MIN_V) * 100.0;
    if (pct < 0.0)   pct = 0.0;
    if (pct > 100.0) pct = 100.0;
    return pct;
}

static double percentageToVolts(double percentage) {
    return BATT_MIN_V + (percentage / 100.0) * (BATT_MAX_V - BATT_MIN_V);
}

// ---------------------------------------------------------------------------
// llegeixLinia - llegeix del socket del pont fins al salt de linia i modifica per a que sigui llegible per atof()
// ---------------------------------------------------------------------------
// Llegeix els bytes de 'fd' d'un en un fins al primer salt de linia. A 'linia'
// hi deixa el que ha llegit, sense el salt i acabat en '\0'; 'mida' es la mida
// total del buffer, comptant aquest '\0'. Llegint d'un en un i parant al salt
// no queda cap byte sobrant entre crides: la crida seguent comenca just al
// primer byte de la linia seguent.
// El recv() es bloquejant: el fil que la crida queda aturat aqui, sense
// consumir CPU, fins que arriba un byte.
// Retorna NO_ERROR amb la linia a punt per a atof(), o ERROR si el pont ha
// tancat la connexio, si la xarxa ha fallat, o si arriben mida-1 bytes sense
// cap salt de linia. No escriu cap missatge: qui la crida decideix si cal
// avisar.
// ---------------------------------------------------------------------------
static int llegeixLinia(int fd, char *linia, int mida) {
    int n = 0;   // caracters ja posats a 'linia'
    char c;      // byte que acabem de llegir

    // El - 1 deixa lloc per al '\0': aixi mai no s'escriu fora del buffer.
    while (n < mida - 1) {
        if (recv(fd, &c, 1, 0) <= 0) {   // -1 error de xarxa, 0 tancat pel remot
            return ERROR;
        }
        if (c == '\n') {        // marca de final: la linia ja es sencera
            linia[n] = '\0';    // es tanca la cadena i el salt no s'hi guarda
            return NO_ERROR;
        }
        linia[n++] = c;         // qualsevol altre byte forma part de la mesura
    }
    return ERROR;            // linia mes llarga que el buffer: no es el nostre pont
}

// ---------------------------------------------------------------------------
// vigilarLidar - cos del fil del LiDAR
// ---------------------------------------------------------------------------
// Bloquejat al recv() del pont: es desperta a cada linia, la
// converteix a metres i, si baixa del llindar, posa LIDAR_OBSTACLE a la cua UNA
// vegada. No torna a avisar fins que la distancia puja per sobre del llindar
// mes MARGE_REARM_LIDAR o el pont diu que no veu res (valor negatiu): sense
// aquesta franja, una mesura que balla al voltant del llindar enviaria
// l'esdeveniment moltes vegades seguides. Es el mateix anti-spam que
// avisBateriaEnviat.
// Si el pont tanca o la xarxa falla, el fil avisa (nomes si no es un tancament
// demanat per closeSystem) i acaba: l'aplicacio segueix, pero sense obstacles.
// ---------------------------------------------------------------------------
static void * vigilarLidar(void *none) {
    char linia[LIDAR_MAX_LINIA];     // aqui hi deixa llegeixLinia cada linia del pont
    double dist;                     // distancia frontal de la linia actual, en metres
    EVENT_T event = LIDAR_OBSTACLE;  // el valor que va a la cua: mq_send copia d'una adreca
    BOOL avisEnviat = FALSE;         // TRUE mentre l'obstacle actual ja s'ha avisat
    BOOL stop_local;                 // copia de 'stop', per saber qui ha tancat

    // Una volta per linia del pont. El bucle s'acaba quan
    // llegeixLinia torna ERROR: pont tancat, xarxa caiguda o closeSystem().
    while (llegeixLinia(lidarfd, linia, LIDAR_MAX_LINIA) == NO_ERROR) {
        dist = atof(linia);      // "-1.000" = no veu res; "0.734" = 0,734 m

        // Zona d'obstacle: mesura valida i per sota del llindar demanat.
        if (dist > 0.0 && dist <= distObstacleAPI) {
            if (avisEnviat == FALSE) {   // nomes el primer cop d'aquest obstacle
                if (mq_send(cua_api, (char *)&event, sizeof(EVENT_T), 0) == -1) {
                    perror("[ERROR - API] LiDAR: no s'ha pogut posar l'obstacle a la cua");
                }
                avisEnviat = TRUE;       // fins que no es rearmi, no s'envia cap mes
            }
        }
        // Zona lliure: el pont no veu res (valor negatiu) o la distancia ha
        // pujat per sobre del llindar mes el marge. Entre el llindar i el
        // marge no es fa res: es la franja morta que evita una rafega d'avisos
        // amb un obstacle just al llindar.
        else if (dist < 0.0 || dist > distObstacleAPI + MARGE_REARM_LIDAR) {
            avisEnviat = FALSE;  // fora de la franja morta: es rearma l'avis
        }
    }

    // Ja no arriben linies. Si el tancament l'ha demanat closeSystem(), l'error
    // es esperat i no s'informa; mateix criteri que el fil del robot.
    pthread_mutex_lock(&mutexAPI);
    stop_local = stop;
    pthread_mutex_unlock(&mutexAPI);
    if (!stop_local) {
        fprintf(stderr, "[AVIS - API] LiDAR: connexio amb el pont perduda. "
                        "No hi haura mes esdeveniments d'obstacle.\n");
    }
    return NULL;   // pthread_create exigeix la firma void*(void*): el valor no s'usa
}

// Inicialitza la connexio amb el QBot i crea el fil d'actualitzacio.
// Amb config == NULL s'apliquen els DEFAULT_*. Si no, cada camp es valida per
// separat: si esta fora de rang s'avisa per stderr i s'aplica el per defecte.
// Flux: config efectiva -> socket + connect al robot -> socket + connect al
// pont del LiDAR (nomes amb ESDE_LIDAR_OBSTACLE) -> cua POSIX (el descriptor
// surt a config->cua) -> variables d'event -> fil vigilarLidar (nomes amb
// LiDAR) -> fil actualitzarVariables.
// Retorna NO_ERROR (0) / ERROR (-1). 'config' no es const perque 'cua' es de
// sortida i l'omple aquesta funcio.
int initSystem (const char *qbot_ip, VEL_MODE velMode, QBotConfig *config) {
    // -----------------------------------------------------------------------
    // BLOC 1: Construccio de la configuracio efectiva
    // -----------------------------------------------------------------------
    // Copia local: si l'usuari ha passat NULL no hi ha struct que tocar, i si
    // n'ha passat una no li modifiquem cap valor, nomes 'cua' (bloc 3).
    QBotConfig efectiva;

    if (config == NULL) {
        // L'usuari no ha passat config: sense struct no hi ha on retornar la
        // cua, i per tant cap event (mascara 0). La resta, valors per defecte.
        efectiva.tipusEsdeveniments = 0;
        efectiva.limitBateria       = DEFAULT_LIMIT_BATERIA;
        efectiva.capacitatCua       = DEFAULT_CAPACITAT_CUA;
        efectiva.distObstacle       = DEFAULT_DIST_OBSTACLE;
    } else {
        // Copiem els camps de l'usuari i validem camp a camp.
        efectiva = *config;

        // Validacio de tipusEsdeveniments: es mascaren els bits coneguts.
        // Una mascara a 0 es valida: l'usuari no vol cap event.
        unsigned char bits_coneguts = ESDE_BATERIA | ESDE_LIDAR_OBSTACLE | ESDE_DESCONNEXIO;
        unsigned char bits_extra    = efectiva.tipusEsdeveniments & (unsigned char)(~bits_coneguts); //deixa a 1 els bits que no coneixem
        if (bits_extra != 0) {
            fprintf(stderr,
                "[AVIS - API] initSystem: tipusEsdeveniments conte bits no reconeguts "
                "(0x%02X), s'ignoraran.\n", bits_extra);
            efectiva.tipusEsdeveniments &= bits_coneguts;
        }

        // Validacio de limitBateria: ha d'estar entre 0.0 i 100.0 %
        if (efectiva.limitBateria < 0.0 || efectiva.limitBateria > 100.0) {
            fprintf(stderr,
                "[AVIS - API] initSystem: limitBateria %.2f fora de rang, "
                "s'utilitza %.2f%%\n",
                efectiva.limitBateria, DEFAULT_LIMIT_BATERIA);
            efectiva.limitBateria = DEFAULT_LIMIT_BATERIA;
        }

        // Validacio de capacitatCua: ha d'estar entre 1 i MAX_CAPACITAT_CUA
        if (efectiva.capacitatCua < 1 || efectiva.capacitatCua > MAX_CAPACITAT_CUA) {
            fprintf(stderr,
                "[AVIS - API] initSystem: capacitatCua %d fora de rang [1, %d], "
                "s'utilitza %d\n",
                efectiva.capacitatCua, MAX_CAPACITAT_CUA, DEFAULT_CAPACITAT_CUA);
            efectiva.capacitatCua = DEFAULT_CAPACITAT_CUA;
        }

        // Validacio de distObstacle: entre 0 (exclos) i l'abast del sensor
        if (efectiva.distObstacle <= 0.0 || efectiva.distObstacle > MAX_DIST_OBSTACLE) {
            fprintf(stderr,
                "[AVIS - API] initSystem: distObstacle %.2f fora de rang (0, %.1f] m, "
                "s'utilitza %.2f m\n",
                efectiva.distObstacle, MAX_DIST_OBSTACLE, DEFAULT_DIST_OBSTACLE);
            efectiva.distObstacle = DEFAULT_DIST_OBSTACLE;
        }
    }

    // -----------------------------------------------------------------------
    // BLOC 2: Configuracio del socket i connexio al robot
    // -----------------------------------------------------------------------

    pthread_attr_t attr;                // atributs de planificacio del fil
    struct sched_param param;           // prioritat del fil
    struct sockaddr_in server_addr;     // adreca IP i port del robot

    // Ajusta el mode general del robot combinant mode docencia amb velMode
    modeAPI = TEACHING_MODE | velMode;

    //Validacio basica del parametre IP
    if (qbot_ip==NULL || qbot_ip[0]=='\0') //Null no apunta enlloc, i \0 indica final de cadena
    {
        fprintf(stderr, "[ERROR - API] initSystem: IP del QBot no valida (NULL o buida)\n");
        return ERROR;
    }

    // Ignora SIGPIPE a tot el proces: escriure en un socket que l'altra banda
    // ja ha tancat fa que send() torni -1 amb EPIPE en comptes de matar el
    // programa, i aixi el guard d'error del fil arriba a executar-se. Es de
    // proces, no de fil ni de socket: es posa un sol cop, abans d'obrir cap
    // socket, i cobreix tambe el socket del pont del LiDAR.
    signal(SIGPIPE, SIG_IGN);

    sockfd = socket(AF_INET, SOCK_STREAM, 0); // AF_INET = IPv4, SOCK_STREAM = TCP
    if (sockfd < 0) {
        perror("Error al crear el socket");
        return ERROR;
    }

    memset(&server_addr, 0, sizeof(server_addr)); // neteja la struct abans d'omplir-la
    server_addr.sin_family = AF_INET; //IPv4
    server_addr.sin_port = htons(QBOT_PORT);      // htons: port en format de xarxa
    inet_pton(AF_INET, qbot_ip, &server_addr.sin_addr); // IP de text a binari

    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Error de conexio");
        close(sockfd);
        return ERROR;
    }

    // -----------------------------------------------------------------------
    // BLOC 2bis: Connexio amb el pont del LiDAR, nomes si s'ha demanat.
    // Mateixos passos que el socket del robot; mateixa IP (el pont corre al
    // Jetson) i el port PORT_LIDAR. Si el pont no respon, no s'arrenca a
    // mitges: es tanca el que hi ha obert i es retorna ERROR.
    // -----------------------------------------------------------------------
    lidarfd = -1;   // -1 = sense LiDAR: mentre valgui -1, ningu no el tanca ni l'espera
    if (efectiva.tipusEsdeveniments & ESDE_LIDAR_OBSTACLE) {   // nomes si l'usuari l'ha demanat a la mascara
        struct sockaddr_in adreca_pont;   // adreca del pont: IP del robot i PORT_LIDAR

        lidarfd = socket(AF_INET, SOCK_STREAM, 0); // AF_INET = IPv4, SOCK_STREAM = TCP, com el del robot
        if (lidarfd < 0) {
            perror("Error al crear el socket del pont del LiDAR");
            close(sockfd);   // el socket del robot ja era obert: no es deixa penjat
            return ERROR;
        }

        memset(&adreca_pont, 0, sizeof(adreca_pont)); // neteja la struct abans d'omplir-la
        adreca_pont.sin_family = AF_INET;             // IPv4
        adreca_pont.sin_port   = htons(PORT_LIDAR);   // htons: port en format de xarxa
        inet_pton(AF_INET, qbot_ip, &adreca_pont.sin_addr); // mateixa IP que el robot: el pont corre al Jetson

        if (connect(lidarfd, (struct sockaddr *)&adreca_pont, sizeof(adreca_pont)) < 0) {
            perror("Error de connexio amb el pont del LiDAR (corre pont_lidar.py al Jetson?)");
            close(lidarfd);   // es tanca el que s'acaba d'obrir
            lidarfd = -1;     // i es torna a marcar "sense LiDAR"
            close(sockfd);    // i tambe el del robot: initSystem no arrenca a mitges
            return ERROR;
        }
    }

    // -----------------------------------------------------------------------
    // BLOC 3: Obertura de la cua POSIX amb la capacitat de la config efectiva
    // -----------------------------------------------------------------------
    struct mq_attr attr_cua;
    attr_cua.mq_flags = 0; //no gasta CPU mentres espera (bloquejant)
    attr_cua.mq_maxmsg  = efectiva.capacitatCua;   // configurable via QBotConfig
    attr_cua.mq_msgsize = sizeof(EVENT_T);          // La mida es el que ocupa el nostre 'enum'

    // Esborrem la cua per si havia quedat oberta d'una execucio anterior
    mq_unlink(CUA_NOM);

    cua_api = mq_open(CUA_NOM, O_CREAT | O_RDWR, 0644, &attr_cua); //crea o llegueix i modifica si existeix, permisos rw-r--r--, amb els atributs definits a attr_cua
    if (cua_api == (mqd_t)-1) {
        perror("Error al crear la cua de missatges, falta a la maquina virual fer mqueue &");
        close(sockfd);
        if (lidarfd >= 0) { close(lidarfd); lidarfd = -1; } // si hi havia pont obert, tambe cau
        return ERROR;
    }

    // 'cua' es el parametre de sortida: la API hi escriu el descriptor de la cua
    // perque l'aplicacio pugui llegir els events. Amb config == NULL la cua es
    // crea igualment, pero no hi ha struct on escriure-la: no hi haura events.
    if (config != NULL) {
        config->cua = cua_api; // '->' segueix el punter i agafa el camp
    }

    // -----------------------------------------------------------------------
    // BLOC 4: Inicialitzacio de les variables globals d'event. Es fa abans de
    // crear el fil, i per aixo no cal mutex.
    // -----------------------------------------------------------------------
    llindarBateria        = percentageToVolts(efectiva.limitBateria); // convertim % -> Volts
    tipusEsdevenimentsAPI = efectiva.tipusEsdeveniments;              // bitmask d'events actius
    avisBateriaEnviat     = FALSE;                                    // reset de l'anti-spam
    distObstacleAPI       = efectiva.distObstacle;                    // llindar del LiDAR en metres

    // Assigna TRUE per indicar que el bucle d'actualitzacio ha d'iniciar-se
    arm = TRUE;
    // Inicia stop a FALSE per no aturar el bucle fins que s'indiqui
    stop = FALSE;

    // -----------------------------------------------------------------------
    // BLOC 5: Atributs de temps real dels fils de l'API (els mateixos per als dos)
    // -----------------------------------------------------------------------
    pthread_attr_init(&attr); //omplir fil amb valors per defecte
    // No hereta la prioritat del pare: aixi podem posar SCHED_RR i prioritat 50. Si no,
    // el fil hereta la prioritat del main() i no es garanteix que tingui CPU.
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);                // temps real, round-robin (prioridad sempre mana la CPU), amb iguals s'ho repartirien a temps iguals
    param.sched_priority = PRIO_ACTVAR;

    pthread_attr_setschedparam(&attr, &param); //per a que prioritat dins de param es guardi en el fil

    int thread_err;   // codi que torna pthread_create; el fan servir els dos fils

    // -----------------------------------------------------------------------
    // BLOC 5bis: Fil del LiDAR, nomes si hi ha pont connectat. Es crea abans
    // que el del robot perque, si falla, encara no hi ha cap fil a aturar.
    // -----------------------------------------------------------------------
    if (lidarfd >= 0) {   // nomes si el BLOC 2bis ha obert el pont
        thread_err = pthread_create(&tidLidar, &attr, vigilarLidar, NULL); // mateixos atributs de temps real que el fil del robot
        if (thread_err != 0) {
            fprintf(stderr, "[ERROR FATAL] No s'ha pogut crear el fil del LiDAR. Codi: %d\n", thread_err);
            // Cap fil no corre encara: nomes cal desfer el que s'ha obert.
            mq_close(cua_api);   // descriptor de la cua
            mq_unlink(CUA_NOM);  // i la cua del sistema
            close(sockfd);       // socket del robot
            close(lidarfd);      // socket del pont
            lidarfd = -1;        // es torna a marcar "sense LiDAR"
            return ERROR;
        }
    }

    // -----------------------------------------------------------------------
    // BLOC 6: Creacio del fil actualitzarVariables amb prioritat temps real
    // -----------------------------------------------------------------------
    thread_err = pthread_create(&tidActVar, &attr, actualitzarVariables, NULL); //on guarda identificador del fil, amb atributs definits a attr, funcio que executa el fil, parametre de la funcio (NULL)

    if (thread_err != 0) {
        fprintf(stderr, "[ERROR FATAL] No s'ha pogut crear el thread d'actualitzacio. Codi: %d\n", thread_err);

        // Neteja de recursos (rollback)
        if (lidarfd >= 0) {
            // Aqui el fil del LiDAR JA corre: no n'hi ha prou de tancar, cal
            // aturar-lo i esperar-lo abans de tornar ERROR.
            pthread_mutex_lock(&mutexAPI);
            stop = TRUE;                        // li diu que el tancament es nostre
            pthread_mutex_unlock(&mutexAPI);
            shutdown(lidarfd, SHUT_RDWR);       // desbloqueja el recv() on esta aturat
            pthread_join(tidLidar, NULL);       // espera que acabi de debo
            close(lidarfd);                     // ara ja es pot alliberar el descriptor
            lidarfd = -1;                       // es torna a marcar "sense LiDAR"
        }
        mq_close(cua_api);   // Tanquem el descriptor de la cua
        mq_unlink(CUA_NOM); // Eliminem la cua del sistema
        close(sockfd);       // Tanquem el socket de xarxa

        return ERROR;
    }

    return NO_ERROR;
}

// Tanca el sistema: envia l'ordre de parada al robot, atura el thread i tanca la connexio
// Retorna NO_ERROR (0) si tot ha anat be, ERROR (-1) si l'ordre de parada no ha sortit o algun recurs no s'ha pogut alliberar.
int closeSystem(void) {
    int resultat = NO_ERROR; // assumeix que tot va be, i si algun recurs falla es canvia a ERROR. Es per a que no surti immediatament i intenti alliberar tot el que pugui.

    // Marca 'stop' per aturar el bucle en execucio
    pthread_mutex_lock(&mutexAPI);
    stop = TRUE;
    pthread_mutex_unlock(&mutexAPI);

    // Ultima trama cap al robot, tota a zero menys el mode: arm = 0 i rodes
    // a 0 rad/s. S'envia des d'aqui perque el fil pot estar bloquejat al
    // recv() i no arribar mai al seu send(). modeAPI es llegeix sense mutex
    // perque nomes s'escriu a initSystem(), abans de crear el fil.
    double tramaParada[REQUEST_SIZE] = {0};
    tramaParada[0] = modeAPI;
    if (send(sockfd, tramaParada, sizeof(tramaParada), 0) < 0) {
        perror("[ERROR - API] closeSystem: No s'ha pogut enviar l'ordre de parada");
        resultat = ERROR;
    }

    // shutdown() i no close(): fa que el recv() pendent retorni 0 i el
    // pthread_join pugui tornar. Desbloqueja recv()/send() del socket i fa que no es quedi penjat
    shutdown(sockfd, SHUT_RDWR);

    // Espera que el thread d'actualitzacio finalitzi (join)
    pthread_join(tidActVar, NULL); //Null: no es retorna res del fil, nomes esperem que acabi

    // El fil del LiDAR, si n'hi ha: mateix mecanisme que el fil del robot.
    // 'stop' ja s'ha posat a TRUE mes amunt, aixi que en despertar-se sabra
    // que el tancament l'hem demanat nosaltres i no es queixara.
    if (lidarfd >= 0) {
        shutdown(lidarfd, SHUT_RDWR);      // desbloqueja el recv() del pont
        pthread_join(tidLidar, NULL);      // espera que el fil acabi
        if (close(lidarfd) == -1) {        // i allibera el descriptor
            perror("[ERROR - API] closeSystem: No s'ha pogut tancar el socket del LiDAR");
            resultat = ERROR;              // no s'atura el tancament: s'anota i segueix
        }
        lidarfd = -1;                      // es torna a marcar "sense LiDAR"
    }

    if (mq_close(cua_api) == -1) {
        perror("[ERROR - API] closeSystem: No s'ha pogut tancar la cua de missatges");
        resultat = ERROR;
    }
    mq_unlink(CUA_NOM); // Eliminem la cua del sistema operatiu

    if (close(sockfd) == -1) {
        perror("[ERROR - API] closeSystem: No s'ha pogut tancar el socket");
        resultat = ERROR;
    }

    return resultat;
}

// Getters: llegeixen les globals sota mutexAPI. Retornen la struct per valor i
// per tant no poden senyalitzar errors de comunicacio.
WheelPair getWheelPositions(void) {
    WheelPair wp;
    pthread_mutex_lock(&mutexAPI);
    wp.esquerra = wheelPositionAPI[0];
    wp.dreta    = wheelPositionAPI[1];
    pthread_mutex_unlock(&mutexAPI);
    return wp;
}

WheelPair getWheelSpeeds(void) {
    WheelPair wp;
    pthread_mutex_lock(&mutexAPI);
    wp.esquerra = wheelSpeedAPI[0];
    wp.dreta    = wheelSpeedAPI[1];
    pthread_mutex_unlock(&mutexAPI);
    return wp;
}

WheelPair getWheelCMDvoltages(void) {
    WheelPair wp;
    pthread_mutex_lock(&mutexAPI);
    wp.esquerra = cmdVoltageAPI[0];
    wp.dreta    = cmdVoltageAPI[1];
    pthread_mutex_unlock(&mutexAPI);
    return wp;
}

WheelPair getWheelCurrents(void) {
    WheelPair wp;
    pthread_mutex_lock(&mutexAPI);
    wp.esquerra = currentAPI[0];
    wp.dreta    = currentAPI[1];
    pthread_mutex_unlock(&mutexAPI);
    return wp;
}

Vector3D getRobotAccelero(void) {
    Vector3D v;
    pthread_mutex_lock(&mutexAPI);
    v.x = acceleroAPI[0];
    v.y = acceleroAPI[1];
    v.z = acceleroAPI[2];
    pthread_mutex_unlock(&mutexAPI);
    return v;
}

Vector3D getRobotGyroscop(void) {
    Vector3D v;
    pthread_mutex_lock(&mutexAPI);
    v.x = gyroscopAPI[0];
    v.y = gyroscopAPI[1];
    v.z = gyroscopAPI[2];
    pthread_mutex_unlock(&mutexAPI);
    return v;
}

double getBatteryLevel(void) {
    double volts;
    pthread_mutex_lock(&mutexAPI);
    volts = batteryLevelAPI;
    pthread_mutex_unlock(&mutexAPI);
    // Retorna el percentatge de bateria (conversio des de Volts)
    return voltsToPercentage(volts);
}

RobotState getRobotState(void) {
    RobotState s;
    // Un sol bloqueig de mutex garanteix coherencia temporal de tot l'estat
    pthread_mutex_lock(&mutexAPI);
    s.position.esquerra = wheelPositionAPI[0];
    s.position.dreta    = wheelPositionAPI[1];
    s.speed.esquerra    = wheelSpeedAPI[0];
    s.speed.dreta       = wheelSpeedAPI[1];
    s.voltage.esquerra  = cmdVoltageAPI[0];
    s.voltage.dreta     = cmdVoltageAPI[1];
    s.current.esquerra  = currentAPI[0];
    s.current.dreta     = currentAPI[1];
    s.accelero.x = acceleroAPI[0];
    s.accelero.y = acceleroAPI[1];
    s.accelero.z = acceleroAPI[2];
    s.gyroscop.x = gyroscopAPI[0];
    s.gyroscop.y = gyroscopAPI[1];
    s.gyroscop.z = gyroscopAPI[2];
    s.battery = voltsToPercentage(batteryLevelAPI);
    pthread_mutex_unlock(&mutexAPI);
    return s;
}

// ---------------------------------------------------------------------------
// VELOCITAT DEL COS I ODOMETRIA
// ---------------------------------------------------------------------------
// Las dos versiones de la velocidad del cuerpo estan disponibles a la vez:
// getRobotVelocity() (velocidades de rueda que reporta el robot) y
// getRobotVelocityPos() (incremento de posicion medido por el hilo interno).
// ---------------------------------------------------------------------------

// Cinematica directa a partir de las velocidades de rueda que reporta el robot.
RobotVelocity getRobotVelocity(void) {
    RobotVelocity rv;
    double wE, wD;

    // Se copia bajo el mutex y se calcula ya fuera: la seccion critica se queda
    // en dos lecturas.
    pthread_mutex_lock(&mutexAPI);
    wE = wheelSpeedAPI[0];   // index 0 = roda esquerra
    wD = wheelSpeedAPI[1];   // index 1 = roda dreta
    pthread_mutex_unlock(&mutexAPI);

    // La media de las dos ruedas da el avance del centro; la diferencia, el giro.
    rv.v     = WHEEL_RADIUS * (wD + wE) / 2.0;
    rv.omega = WHEEL_RADIUS * (wD - wE) / WHEEL_BASE;
    return rv;
}

// Las mismas magnitudes, pero derivadas del incremento de posicion que ya ha
// medido el hilo interno. Aqui no se recalcula nada: solo se leen sus globales.
RobotVelocity getRobotVelocityPos(void) {
    RobotVelocity rv;
    pthread_mutex_lock(&mutexAPI);
    rv.v     = velPosLinealAPI;
    rv.omega = velPosAngularAPI;
    pthread_mutex_unlock(&mutexAPI);
    return rv;
}

// Odometria con phi integrado del GIROSCOPO.
Odometry getOdometry(void) {
    Odometry o;
    pthread_mutex_lock(&mutexAPI);
    o.x   = odoGyroXAPI;
    o.y   = odoGyroYAPI;
    o.phi = odoGyroPhiAPI;
    pthread_mutex_unlock(&mutexAPI);
    return o;
}

// Odometria con phi deducido de la diferencia entre las dos RUEDAS.
Odometry getOdometryWheels(void) {
    Odometry o;
    pthread_mutex_lock(&mutexAPI);
    o.x   = odoRodesXAPI;
    o.y   = odoRodesYAPI;
    o.phi = odoRodesPhiAPI;
    pthread_mutex_unlock(&mutexAPI);
    return o;
}

// Retorna NO_ERROR si l'ordre s'ha acceptat correctament, ERROR si el punter era NULL.
int setLED_Color(BOOL enableUserLED, double *pLED_Color) {
    int i;
    double colors_locals[NUM_LED_COLORS];

    // Si l'usuari demana encendre el LED, es valida l'array de colors abans de
    // tocar cap global.
    if (enableUserLED) {
        // Escut anti-punter NULL
        if (pLED_Color == NULL) {
            fprintf(stderr, "[ERROR - API] setLED_Color: S'ha demanat encendre el LED pero el punter de colors es NULL.\n");
            fprintf(stderr, " -> SOLUCIO: Passa un array amb 3 valors RGB o posa enableUserLED a FALSE.\n");
            return ERROR; // Punter NULL es un error de programacio; el LED queda com estava
        }

        // Escut de saturacio: els valors fora de rang es corregeixen (clipping)
        // i NO es retorna ERROR; el LED s'encen amb el valor saturat.
        for (i = 0; i < NUM_LED_COLORS; i++) {
            double colorActual = pLED_Color[i];

            if (colorActual < 0.0) {
                fprintf(stderr, "[AVIS - API] setLED_Color: El component RGB[%d] (%.2f) es negatiu. Ajustat a 0.0.\n", i, colorActual);
                colorActual = 0.0;
            }
            else if (colorActual > 1.0) {
                fprintf(stderr, "[AVIS - API] setLED_Color: El component RGB[%d] (%.2f) es massa alt. Ajustat a 1.0.\n", i, colorActual);
                colorActual = 1.0;
            }

            colors_locals[i] = colorActual;
        }
    }

    // Estat d'encesa i colors s'escriuen junts, sota un unic mutex, perque el
    // fil intern no envii mai el LED ences amb els colors antics.
    pthread_mutex_lock(&mutexAPI);
    enableUserLED_API = enableUserLED;
    if (enableUserLED) {
        for (i = 0; i < NUM_LED_COLORS; i++) {
            LED_ColorAPI[i] = colors_locals[i];
        }
    }
    pthread_mutex_unlock(&mutexAPI);

    return NO_ERROR;
}

// Les velocitats fora de [-MAX_VEL_RODA, MAX_VEL_RODA] es saturen amb avis
// (clipping), com a setLED_Color. Retorna sempre NO_ERROR.
int setWheelSpeeds(double esquerra, double dreta) {
    if (esquerra > MAX_VEL_RODA) {
        fprintf(stderr, "[AVIS - API] setWheelSpeeds: roda esquerra %.2f rad/s fora de rang. Saturada a %.2f.\n", esquerra, MAX_VEL_RODA);
        esquerra = MAX_VEL_RODA;
    }
    else if (esquerra < -MAX_VEL_RODA) {
        fprintf(stderr, "[AVIS - API] setWheelSpeeds: roda esquerra %.2f rad/s fora de rang. Saturada a %.2f.\n", esquerra, -MAX_VEL_RODA);
        esquerra = -MAX_VEL_RODA;
    }
    if (dreta > MAX_VEL_RODA) {
        fprintf(stderr, "[AVIS - API] setWheelSpeeds: roda dreta %.2f rad/s fora de rang. Saturada a %.2f.\n", dreta, MAX_VEL_RODA);
        dreta = MAX_VEL_RODA;
    }
    else if (dreta < -MAX_VEL_RODA) {
        fprintf(stderr, "[AVIS - API] setWheelSpeeds: roda dreta %.2f rad/s fora de rang. Saturada a %.2f.\n", dreta, -MAX_VEL_RODA);
        dreta = -MAX_VEL_RODA;
    }
    pthread_mutex_lock(&mutexAPI);
    velCmdAPI[0] = esquerra;
    velCmdAPI[1] = dreta;
    pthread_mutex_unlock(&mutexAPI);
    return NO_ERROR;
}

// Cinematica inversa: se pide la velocidad del CUERPO y se reparte entre las
// dos ruedas (rad/s). Retorna lo que retorne setWheelSpeeds (siempre NO_ERROR).
int setRobotVelocity(double v, double omega) {
    double esquerra = (v - omega * WHEEL_BASE / 2.0) / WHEEL_RADIUS;
    double dreta    = (v + omega * WHEEL_BASE / 2.0) / WHEEL_RADIUS;

    // setWheelSpeeds aplica la saturacion a MAX_VEL_RODA y sus avisos. Aqui NO
    // se ha bloqueado mutexAPI, asi que la llamada es segura: el mutex no es
    // recursivo.
    return setWheelSpeeds(esquerra, dreta);
}

// Fija el origen de las DOS odometrias a la misma pose. Retorna siempre NO_ERROR.
int setOdometry(double x, double y, double phi) {
    pthread_mutex_lock(&mutexAPI);
    odoGyroXAPI    = x;
    odoGyroYAPI    = y;
    odoGyroPhiAPI  = phi;
    odoRodesXAPI   = x;
    odoRodesYAPI   = y;
    odoRodesPhiAPI = phi;
    pthread_mutex_unlock(&mutexAPI);
    return NO_ERROR;
}
