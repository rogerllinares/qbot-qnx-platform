// test_lidar_api.c - escolta l'esdeveniment LIDAR_OBSTACLE de l'API sense moure el robot.
// US:  ./test_lidar_api [llindar_m]      (per defecte 0,60 m)

#include <stdio.h>      // printf, fprintf, perror
#include <stdlib.h>     // atof
#include <errno.h>      // errno, ETIMEDOUT, EINTR
#include <time.h>       // clock_gettime: marca de temps i venciment
#include <mqueue.h>     // mq_timedreceive
#include "apibasica.h"  // interficie publica de l'API del QBot

#define QBOT_IP        "192.168.2.16"   // IP del robot, llegir-la del LCD
#define DURADA         60               // segons que dura la prova
#define LIMIT_BATERIA  20.0             // % de bateria per sota del qual l'API avisa
#define CAPACITAT_CUA  10               // missatges que hi caben a la cua POSIX

// Instant d'inici: les marques de temps que s'imprimeixen son segons des d'aqui.
static struct timespec t0;

// Segons transcorreguts des de t0, amb decimals.
static double tempsAra(void) {
    struct timespec ara;
    clock_gettime(CLOCK_REALTIME, &ara);   // hora actual, amb resolucio de ns
    return (ara.tv_sec - t0.tv_sec) + (ara.tv_nsec - t0.tv_nsec) / 1000000000.0;
}

int main(int argc, char *argv[]) {
    // Mascara d'esdeveniments, llindar de bateria, capacitat de la cua i
    // llindar d'obstacle en metres; l'ultim camp (cua) l'omple initSystem.
    QBotConfig cfg = { ESDE_BATERIA | ESDE_LIDAR_OBSTACLE, LIMIT_BATERIA,
                       CAPACITAT_CUA, 0.60, 0 };
    EVENT_T event;              // esdeveniment que es treu de la cua
    ssize_t nb;                 // bytes llegits per mq_timedreceive; -1 si falla
    struct timespec venciment;  // instant absolut en que s'acaba la prova
    long obstacles = 0;         // obstacles rebuts durant la prova

    // Llindar opcional per la linia d'ordres: es pot provar un altre valor
    // sense recompilar.
    if (argc >= 2) {
        cfg.distObstacle = atof(argv[1]);
    }

    // Firma a pantalla: identifica quin binari s'esta executant.
    printf("=====================================================\n");
    printf(" test_lidar_api - esdeveniment LIDAR_OBSTACLE de l'API\n");
    printf(" build %s %s\n", __DATE__, __TIME__);
    printf(" llindar %.2f m, %d s d'escolta, el robot NO es mou\n",
           cfg.distObstacle, DURADA);
    printf("=====================================================\n");

    clock_gettime(CLOCK_REALTIME, &t0);   // a partir d'aqui compten els segons

    // initSystem obre el socket del robot i el del pont del LiDAR, i omple cfg.cua.
    if (initSystem(QBOT_IP, WHEEL_VEL_MODE, &cfg) == ERROR) {
        printf("Error: No s'ha pogut inicialitzar el sistema.\n");
        return 1;
    }
    printf("[%6.2f s] initSystem OK: robot i pont del LiDAR connectats.\n", tempsAra());
    fflush(stdout);   // que la linia surti ara, encara que la sortida vagi a un fitxer

    // Venciment absolut t0 + DURADA: mq_timedreceive espera fins a aquest
    // instant i torna -1 amb errno = ETIMEDOUT si no arriba res.
    venciment = t0;
    venciment.tv_sec += DURADA;

    // Bucle d'escolta sense espera activa: el proces esta aturat dins de
    // mq_timedreceive fins que arriba un esdeveniment o s'acaba el temps.
    while (1) {
        nb = mq_timedreceive(cfg.cua, (char *)&event, sizeof(EVENT_T), NULL, &venciment);
        if (nb < 0) {
            if (errno == EINTR) {
                continue;               // senyal: no es cap error
            }
            if (errno != ETIMEDOUT) {
                perror("mq_timedreceive");   // qualsevol altre errno si que ho es
            }
            break;                      // temps esgotat o error: s'acaba
        }
        if (event == LIDAR_OBSTACLE) {
            obstacles++;                // un per cada entrada a la zona d'obstacle
            printf("[%6.2f s] LIDAR_OBSTACLE (%ld)\n", tempsAra(), obstacles);
        } else if (event == BATERIA_BAIXA) {
            printf("[%6.2f s] BATERIA_BAIXA\n", tempsAra());
        } else {
            // La cua es unica per a tots els esdeveniments: qualsevol codi nou
            // es veu aqui en comptes de perdre's en silenci.
            printf("[%6.2f s] esdeveniment desconegut: %d\n", tempsAra(), (int)event);
        }
        fflush(stdout);
    }

    printf("[%6.2f s] Fi. Obstacles rebuts: %ld\n", tempsAra(), obstacles);
    // closeSystem atura els fils interns i tanca els sockets i la cua.
    if (closeSystem() == ERROR) {
        fprintf(stderr, "[AVIS] closeSystem ha retornat ERROR.\n");
    }
    return 0;
}
