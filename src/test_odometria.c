// ---------------------------------------------------------------------------
// test_odometria.c
// ---------------------------------------------------------------------------
// Executa fases de control i registra les variables del robot en una taula.
// Cada fase reinicia l'origen de l'odometria per facilitar el tractament de dades.
// Dues bateries de fases (vegeu BATERIA_RODA): TERRA valida l'odometria contra
// el mon real; RODA valida la cinematica de l'API contra ella mateixa.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// LLIBRERIES UTILITZADES
// ---------------------------------------------------------------------------
#include <stdio.h>      // Funcions d'entrada i sortida estandard (printf, fprintf)
#include <unistd.h>     // API de sistemes POSIX (sleep, etc.)
#include <time.h>       // Funcions per mesurar i gestionar el temps del sistema
#include "apibasica.h"  // Capcalera de l'API especifica per al control del QBot

// ---------------------------------------------------------------------------
// DEFINICIONS DE CONSTANTS
// ---------------------------------------------------------------------------
// Els tres valors que canvien entre sessions de laboratori.
#define QBOT_IP "192.168.2.16"                        // llegir-la sempre del LCD
#define FITXER_SORTIDA "/tmp/Odometria_10_09.txt"     // sortida de dades

// Bateria de fases que executa el programa. Per canviar-la es canvia aquest
// valor i es torna a compilar.
//   0 = TERRA: fases 1 a 4 (quiet, recta, gir, lenta) amb el robot al terra,
//       comparant l'odometria amb el que es mesura amb el metre.
//   1 = RODA: fases 2 i 6, la mateixa recta manada per setRobotVelocity() i
//       per setWheelSpeeds(), per comparar els dos comandaments.
#define BATERIA_RODA 0

#define T_MOSTREIG 50   // Periodo de mostreig en ms, igual a la cadencia del
                        // robot: mostrejar mes rapid nomes repetiria el mateix valor

// Consignes i durades de cada fase.
#define V_RECTA      0.1    // m/s   - fase 2
#define D_RECTA      8.0    // s     - 8 s x 0.1 m/s = 0.80 m teorics
#define W_GIR        0.5    // rad/s - fase 3
#define D_GIR        6.3    // s     - 6.3 s x 0.5 rad/s = 3.15 rad = 180.5 graus
#define V_LENTA      0.03   // m/s   - fase 4
#define D_LENTA      6.0    // s     - 6 s x 0.03 m/s = 0.18 m teorics
#define D_QUIET      5.0    // s     - fase 1, sense cap consigna

// FASE 6: compara una consigna de cos amb dues consignes iguals de roda.
#define W_RODA       2.2497 // rad/s - 0.1 m/s entre 0.04445 m de radi de roda
#define D_RODA       8.0    // s     - la mateixa durada que la fase 2, per
                            // poder comparar les dues rectes directament

#define T_PARADA     2.0    // s - durada del registre posterior a la consigna
#define T_ENTRE_FASES 3     // s - pausa entre fases, sense registrar
#define T_ABANS_DE_MOURE 5  // s - avis abans del primer moviment

// ---------------------------------------------------------------------------
// VARIABLES GLOBALS
// ---------------------------------------------------------------------------
FILE * pDataFile;       // Punter al fitxer on s'escriuran les dades recollides

// ---------------------------------------------------------------------------
// executaFase - una fase completa: consigna, registre periodic i parada
// ---------------------------------------------------------------------------
// fase   - numero que va a la columna 2 del fitxer, per poder filtrar a MATLAB
// nom    - text del rotul que surt per stderr quan arrenca la fase
// v      - consigna de velocitat lineal, en m/s
// omega  - consigna de velocitat angular, en rad/s
// perRoda- FALSE: la consigna va per setRobotVelocity() i passa per la
//          cinematica inversa. TRUE: va per setWheelSpeeds(v, omega), o sigui
//          que v i omega son rad/s de la roda esquerra i de la dreta, i la
//          cinematica queda fora.
// durada - segons que dura la consigna
//
// Retorna NO_ERROR (0) / ERROR (-1). Si torna ERROR, el robot ja esta aturat.
// ---------------------------------------------------------------------------
int executaFase(int fase, const char *nom, double v, double omega, double durada,
                BOOL perRoda) {
    struct timespec interval, ts, t0_fase, t1;  // control del bucle periodic, ts -> instant que tornara a despertar; t1 -> ara;
    //t_rel = t1-t0_fase(quan comença fase)  ts=ts + interval (50 ms)
    double t_rel;       // segons des de l'inici de la fase
    int rc;             // retorn de la funcio de consigna

    // Estat del robot llegit d'una vegada amb getRobotState()
    RobotState estat;

    RobotVelocity velVel;   // v i omega a partir de la VELOCITAT reportada
    RobotVelocity velPos;   // v i omega a partir de l'INCREMENT de posicio
    Odometry      odoGyro;  // x, y d'encoders - phi del GIROSCOPI
    Odometry      odoWheel; // x, y d'encoders - phi de les RODES

    // Rotul de la fase per pantalla. fprintf(desti, format, valors...) escriu
    // al desti que se li dona: stderr es el canal d'errors i avisos, surt per
    // pantalla sense buffer (es veu a l'instant) i no es barreja amb les dades,
    // que van a pDataFile. Dins del format, %d es substitueix per un int, %s
    // per una cadena, %.4lf per un double amb 4 decimals, %% es un % literal i
    // \n es un salt de linia; els valors es prenen en ordre despres del format.
    fprintf(stderr, "\n=== FASE %d - %s ===\n", fase, nom);
    if (perRoda) {
        fprintf(stderr, " consigna wE = %.4lf rad/s   wD = %.4lf rad/s   durada = %.1lf s\n", v, omega, durada);
    } else {
        fprintf(stderr, " consigna v = %.4lf m/s   omega = %.4lf rad/s   durada = %.1lf s\n", v, omega, durada);
    }

    // Les dues odometries a zero: cada fase es llegeix des de l'origen.
    if (setOdometry(0.0, 0.0, 0.0) == ERROR) {
        fprintf(stderr, "[ERROR] setOdometry ha fallat a la fase %d.\n", fase);
        setRobotVelocity(0.0, 0.0);
        return ERROR;
    }

    // Instant de referencia de la fase i primera activacio del bucle.
    clock_gettime(CLOCK_REALTIME, &t0_fase);           // llegeix el rellotge i el desa a t0_fase
    nsec2timespec(&interval, T_MOSTREIG * 1000000LL);  // 50 ms en ns, passats a timespec, LL per ferho a 64 bits
    timespecPlusTimeInterval(&ts, &t0_fase, &interval); // ts = t0_fase + interval: primer despertar

    do {
        // -------------------------------------------------------------------
        // BLOC 1 - TEMPS I FI DE LA FASE
        // -------------------------------------------------------------------
        clock_gettime(CLOCK_REALTIME, &t1);
        t_rel = (timespec2nsec(&t1) - timespec2nsec(&t0_fase)) / 1.0e9; //realtiu perque clock realtime es des de 1970

        if (t_rel >= durada + T_PARADA) {
            break;   // consigna acabada i el frenat ja registrat
        }

        // -------------------------------------------------------------------
        // BLOC 2 - ENVIAMENT DE LA CONSIGNA
        // -------------------------------------------------------------------
        // La consigna es reenvia cada cicle; passada la durada s'envia zero i
        // es segueix registrant T_PARADA s.
        if (t_rel < durada) {
            rc = perRoda ? setWheelSpeeds(v, omega) : setRobotVelocity(v, omega); // ? es if, si es TRUE fa abans dels :
        } else {
            rc = perRoda ? setWheelSpeeds(0.0, 0.0) : setRobotVelocity(0.0, 0.0);
        }
        if (rc == ERROR) {
            fprintf(stderr, "[ERROR] la consigna ha fallat a la fase %d.\n", fase);
            setRobotVelocity(0.0, 0.0);
            return ERROR;
        }

        // -------------------------------------------------------------------
        // BLOC 3 - LECTURA DE SENSORS I D'ODOMETRIA
        // -------------------------------------------------------------------
        estat    = getRobotState();
        velVel   = getRobotVelocity();      // des de la velocitat reportada
        velPos   = getRobotVelocityPos();   // des de l'increment de posicio
        odoGyro  = getOdometry();           // phi del giroscopi
        odoWheel = getOdometryWheels();     // phi de les rodes

        // -------------------------------------------------------------------
        // BLOC 4 - REGISTRE. Una linia per cicle de T_MOSTREIG ms.
        // L'ordre de les columnes es el de la capcalera que escriu el main i
        // NO es pot tocar: el script de MATLAB hi va lligat.
        // -------------------------------------------------------------------
        fprintf(pDataFile,
                "%lf %d "         // t, fase
                "%lf %lf "        // posicio rodes:   esquerra, dreta
                "%lf %lf "        // velocitat rodes: esquerra, dreta
                "%lf %lf "        // getRobotVelocity:    v, omega
                "%lf %lf "        // getRobotVelocityPos: v, omega
                "%lf %lf %lf "    // getOdometry:       x, y, phi (phi del giroscopi)
                "%lf %lf %lf "    // getOdometryWheels: x, y, phi (phi de rodes)
                "%lf "            // giroscopi z
                "%lf %lf %lf "    // accelerometre: x, y, z
                "%lf\n",          // bateria en %
                t_rel, fase,
                estat.position.esquerra, estat.position.dreta,
                estat.speed.esquerra,    estat.speed.dreta,
                velVel.v, velVel.omega,
                velPos.v, velPos.omega,
                odoGyro.x,  odoGyro.y,  odoGyro.phi,
                odoWheel.x, odoWheel.y, odoWheel.phi,
                estat.gyroscop.z,
                estat.accelero.x, estat.accelero.y, estat.accelero.z,
                estat.battery);

        // clock_nanosleep amb TIMER_ABSTIME dorm FINS a l'instant 'ts', no
        // 'durant'. Just despres s'avanca ts un periode: bucle sense deriva.
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);
        timespecPlusTimeInterval(&ts, &ts, &interval);

    } while(1);

    // Parada explicita en acabar la fase, passi el que passi.
    setRobotVelocity(0.0, 0.0);

    // Resum per pantalla de l'odometria final de la fase.
    odoGyro  = getOdometry();
    odoWheel = getOdometryWheels();
    fprintf(stderr, " Fase %d acabada. Odometria final:\n"
                    "   giroscopi: x = %.4lf m  y = %.4lf m  phi = %.4lf rad\n"
                    "   rodes:     x = %.4lf m  y = %.4lf m  phi = %.4lf rad\n",
                    fase,
                    odoGyro.x,  odoGyro.y,  odoGyro.phi,
                    odoWheel.x, odoWheel.y, odoWheel.phi);

    return NO_ERROR;
}

// ---------------------------------------------------------------------------
// tanca - tanca el fitxer de dades i la connexio amb el robot
// ---------------------------------------------------------------------------
// codi - codi de sortida del programa que es vol retornar (0 be, 1 error)
// Retorna codi, o 1 si closeSystem() no ha pogut alliberar algun recurs.
// ---------------------------------------------------------------------------
static int tanca(int codi) {
    fclose(pDataFile);
    if (closeSystem() == ERROR) {
        fprintf(stderr, "[AVIS] closeSystem ha retornat ERROR: algun recurs no s'ha alliberat correctament.\n");
        return 1;
    }
    return codi;
}

// ---------------------------------------------------------------------------
// pausa - espera T_ENTRE_FASES segons amb un avis per pantalla del que cal fer
// ---------------------------------------------------------------------------
static void pausa(const char *missatge) {
    fprintf(stderr, "\n[pausa de %d s] %s\n", T_ENTRE_FASES, missatge);
    sleep(T_ENTRE_FASES);
}

// ---------------------------------------------------------------------------
// main - connecta amb el robot, obre el fitxer, executa la bateria i tanca
// ---------------------------------------------------------------------------
// No crea fils ni obre la cua d'esdeveniments (config a NULL): el registre es
// fa dins del bucle de executaFase(). closeSystem() es crida a tots els camins
// de sortida posteriors a initSystem(), tambe els d'error (vegeu tanca()).
// ---------------------------------------------------------------------------
int main(void) {

    // Connexio amb el robot en mode de velocitat de rodes: setRobotVelocity()
    // fa la cinematica inversa i mana rad/s a cada roda, igual que setWheelSpeeds().
    if (initSystem(QBOT_IP, WHEEL_VEL_MODE, NULL) == ERROR) {
        fprintf(stderr, "Error: No s'ha pogut inicialitzar el sistema (IP %s).\n", QBOT_IP);
        fprintf(stderr, " -> Comprova la IP al LCD del robot: canvia entre sessions.\n");
        return 1;
    }

    // Fitxer de dades en mode escriptura ("w": si ja existeix, es sobreescriu).
    pDataFile = fopen(FITXER_SORTIDA, "w");
    if (pDataFile == NULL) {
        perror("Error al obrir l'arxiu");
        closeSystem();   // initSystem() ja ha obert el socket i el fil
        return 1;
    }

    // Capcalera de columnes. El caracter de comentari es el de MATLAB, o sigui
    // que aquestes dues linies no molesten en carregar el fitxer.
    fprintf(pDataFile,
            "%% 1:t 2:fase 3:posE 4:posD 5:wE 6:wD 7:v_speed 8:omega_speed "
            "9:v_pos 10:omega_pos 11:x_gyro 12:y_gyro 13:phi_gyro "
            "14:x_wheel 15:y_wheel 16:phi_wheel 17:gyroZ 18:accX 19:accY 20:accZ "
            "21:bateria\n");
    fprintf(pDataFile,
            "%% t en s des de l'inici de CADA fase - fase 1:quiet 2:recta "
            "3:gir 4:lenta 6:roda\n");

    fprintf(stderr, "Bateria a l'arrencada: %.1lf %%\n", getBatteryLevel());
    fprintf(stderr, "Sortida de dades: %s\n", FITXER_SORTIDA);
    fprintf(stderr, "\nEL ROBOT ES MOURA d'aqui a %d segons. Deixa'l al terra,\n"
                    "amb espai lliure al davant, i marca la posicio de sortida.\n",
                    T_ABANS_DE_MOURE);
    sleep(T_ABANS_DE_MOURE);

    // Cada fase: si executaFase() torna ERROR, el robot ja esta aturat i nomes
    // queda tancar el fitxer i la connexio amb tanca(1).
    if (BATERIA_RODA) {
        // RODA: la mateixa recta dues vegades. Fase 2 per setRobotVelocity()
        // (la consigna passa per la cinematica inversa) i fase 6 per
        // setWheelSpeeds() amb la mateixa velocitat a les dues rodes.
        if (executaFase(2, "RECTA amb cinematica - setRobotVelocity",
                        V_RECTA, 0.0, D_RECTA, FALSE) == ERROR) return tanca(1);
        pausa("Mesura la desviacio lateral i torna el robot a la marca.");

        if (executaFase(6, "RECTA sense cinematica - setWheelSpeeds igual a les dues rodes",
                        W_RODA, W_RODA, D_RODA, TRUE) == ERROR) return tanca(1);
    } else {
        // TERRA: quatre fases seguides, amb una pausa entre fase i fase per
        // mesurar amb el metre i tornar el robot a la marca de sortida.
        if (executaFase(1, "QUIET - robot immobil, es mira si phi deriva",
                        0.0, 0.0, D_QUIET, FALSE) == ERROR) return tanca(1);
        pausa("Res a mesurar.");

        if (executaFase(2, "RECTA - mesura la distancia amb el metre",
                        V_RECTA, 0.0, D_RECTA, FALSE) == ERROR) return tanca(1);
        pausa("Mesura la distancia i torna el robot al lloc.");

        if (executaFase(3, "GIR - mitja volta sobre si mateix",
                        0.0, W_GIR, D_GIR, FALSE) == ERROR) return tanca(1);
        pausa("Mira quant ha girat de veritat.");

        if (executaFase(4, "LENTA - velocitat molt baixa, es busca la discrepancia",
                        V_LENTA, 0.0, D_LENTA, FALSE) == ERROR) return tanca(1);
    }

    fprintf(stderr, "\nProves acabades. Dades a %s\n", FITXER_SORTIDA);
    fprintf(stderr, "Bateria al final: %.1lf %%\n", getBatteryLevel());
    return tanca(0);
}
