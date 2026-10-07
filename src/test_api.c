/*
 * test_api.c - Smoke test exhaustiu de l'API QBot Platform
 * ---------------------------------------------------------------------------
 *   Si falla, imprimeix QUE s'esperava, QUE s'ha rebut i QUE vol dir.
 *   Al final sempre surt una linia de resum.
 *
 * COM S'EXECUTA (a la shell de la VM QNX, com a root):
 *   mqueue &                 <- IMPRESCINDIBLE, si no mq_open dona ENOSYS
 *   chmod +x /tmp/test_api
 *   /tmp/test_api 192.168.2.16
 *   (sense argument agafa DEFAULT_IP, pero millor passar-la)
 *
 * REQUISIT FISIC: el bloc 7 MOU EL ROBOT ~5 segons endavant.
 *   Deixa 1 metre lliure al davant. El test avisa i espera abans de moure'l.
 *
 * MAPA DE BLOCS:
 *   B1  parametres invalids     sense connexio    no toca el robot
 *   B2  connexio real           SEMPRE CORRE      sense ell no hi ha res
 *   B3  getters en repos        rangs fisics      no toca el robot
 *   B4  coherencia getRobotState                  no toca el robot
 *   B5  dades fresques          el fil viu?       no toca el robot
 *   B6  validacio de setters                      no toca el robot
 *   B7  moviment real           MOU EL ROBOT      demana confirmacio
 *   B8  LED                     visual            cal mirar el robot
 *   B9  esdeveniments (cua)     alarma bateria    no toca el robot
 *   B10 tancament ordenat       SEMPRE CORRE      valida closeSystem
 *       (si B10 acaba sense cap [ERROR XARXA], closeSystem tanca net)
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <math.h>

#include "apibasica.h"

#define DEFAULT_IP "192.168.2.16"

/* Gravetat esperada al modul de l'accelerometre amb el robot quiet i pla. */
#define G_ESPERADA   9.81
#define G_TOLERANCIA 1.5

/* ========================================================================= */
/* MINI-FRAMEWORK DE COMPROVACIONS                                           */
/* Cap funcio imprimeix res quan la comprovacio passa.                       */
/* ========================================================================= */

static int n_check = 0;   /* comprovacions fetes            */
static int n_falla = 0;   /* comprovacions fallades         */
static int n_avis  = 0;   /* coses rares pero no fatals     */
static const char *bloc_actual = "(cap)";

/* Compta una falla i imprimeix la seva capcalera: [FALLA n] bloc -> nom.
 * Sempre a stderr, que no es redirigeix. Qui crida afegeix despres les
 * linies de detall (valors esperats/obtinguts i que vol dir). */
static void cap_falla(const char *nom) {
    n_falla++;
    fprintf(stderr, "\n[FALLA %d] %s -> %s\n", n_falla, bloc_actual, nom);
}

/* Igual que cap_falla pero per a un avis: compta a n_avis i no es un error. */
static void cap_avis(const char *nom) {
    n_avis++;
    fprintf(stderr, "\n[AVIS %d] %s -> %s\n", n_avis, bloc_actual, nom);
}

/* Comprova una condicio ja avaluada (cond: 0 = falla). Si falla, compta la
 * falla i imprimeix 'nom' (que es comprovava) i 'significat' (que implica). */
static void check_true(int cond, const char *nom, const char *significat) {
    n_check++;
    if (!cond) {
        cap_falla(nom);
        fprintf(stderr, "  Que vol dir: %s\n", significat);
    }
}

/* Comprova que un codi de retorn de l'API (NO_ERROR / ERROR) es exactament
 * l'esperat. Si no coincideix, mostra els dos valors. */
static void check_ret(int obtingut, int esperat, const char *nom, const char *significat) {
    n_check++;
    if (obtingut != esperat) {
        cap_falla(nom);
        fprintf(stderr, "  Esperat: %d   Obtingut: %d\n", esperat, obtingut);
        fprintf(stderr, "  Que vol dir: %s\n", significat);
    }
}

/* Comprova que un valor cau dins del rang tancat [min, max].
 * 'unitat' nomes s'usa al missatge (rad/s, V, %...). */
static void check_rang(double v, double min, double max,
                       const char *nom, const char *unitat, const char *significat) {
    n_check++;
    if (v < min || v > max) {
        cap_falla(nom);
        fprintf(stderr, "  Esperat: entre %.3f i %.3f %s\n", min, max, unitat);
        fprintf(stderr, "  Obtingut: %.3f %s\n", v, unitat);
        fprintf(stderr, "  Que vol dir: %s\n", significat);
    }
}

/* Comprova que dos valors coincideixen: |a - b| <= tol. Serveix per comparar
 * dues rutes que haurien de donar la mateixa dada. */
static void check_igual(double a, double b, double tol,
                        const char *nom, const char *significat) {
    n_check++;
    if (fabs(a - b) > tol) {
        cap_falla(nom);
        fprintf(stderr, "  Valor A: %.4f   Valor B: %.4f   Diferencia: %.4f (tolerancia %.4f)\n",
                a, b, fabs(a - b), tol);
        fprintf(stderr, "  Que vol dir: %s\n", significat);
    }
}

/* Espera bloquejant de 'ms' mil.lisegons. nanosleep() vol un timespec:
 * segons sencers a tv_sec i la resta, passada a nanosegons, a tv_nsec. */
static void espera_ms(long ms) {
    struct timespec t;
    t.tv_sec  =  ms / 1000L;                /* part entera en segons     */
    t.tv_nsec = (ms % 1000L) * 1000000L;    /* resta: 1 ms = 1.000.000 ns */
    nanosleep(&t, NULL);
}

/* Atura't fins que l'usuari premi ENTER. Els blocs visuals (LED) l'usen: sense
 * aixo el missatge passa volant i no dona temps de mirar el robot.
 *
 * Retorna TRUE nomes si hi ha hagut una confirmacio HUMANA de veritat. Si stdin
 * esta tancat o redirigit, getchar() torna EOF de seguida i no hi ha ningu mirant:
 * en aquest cas retorna FALSE i qui crida ha de renunciar a moure el robot. */
static BOOL pausa(const char *missatge) {
    int c;
    printf("\n>>> %s  [ENTER per continuar]\n", missatge);
    fflush(stdout);
    do {
        c = getchar();
        if (c == EOF) {
            fprintf(stderr, "[AVIS] No hi ha stdin interactiu: ningu pot confirmar.\n");
            return FALSE;
        }
    } while (c != '\n');
    return TRUE;
}

/* ========================================================================= */
/* BLOCS DE PROVA                                                            */
/* ========================================================================= */

/* B1 - Validacio de parametres d'entrada, ENCARA SENSE CONNEXIO.
 * Comprova que l'API rebutja el que ha de rebutjar en comptes de petar:
 * initSystem() amb IP nul.la o buida ha de retornar ERROR abans de tocar
 * la xarxa. */
static void bloc1_parametres_invalids(void) {
    bloc_actual = "B1 parametres invalids";

    fprintf(stderr, "\n[CONTEXT] B1 prova entrades invalides a proposit: els errors\n");
    fprintf(stderr, "          de l'API que surten aqui sota son ESPERATS.\n");

    check_true(initSystem(NULL, WHEEL_VEL_MODE, NULL) == ERROR,
               "initSystem(NULL) hauria de fallar",
               "l'API accepta un punter d'IP nul: petara mes endavant amb un segfault.");

    check_true(initSystem("", WHEEL_VEL_MODE, NULL) == ERROR,
               "initSystem(\"\") hauria de fallar",
               "l'API accepta una IP buida: intentara connectar contra res.");
}

/* B2 - Connexio real: initSystem() contra el robot. Retorna 1 si connecta,
 * 0 si no (i llavors main avorta: sense connexio no hi ha res a provar). */
static int bloc2_connexio(const char *ip) {
    int ret;
    /* Config d'events: { bitmask d'events actius, llindar de bateria en %,
     * capacitat de la cua, descriptor de la cua }. El 4t camp l'omple
     * initSystem(). Declarada abans de la primera sentencia del bloc (C89:
     * totes les declaracions van al principi del bloc). */
    QBotConfig cfg = { ESDE_BATERIA, 20.0, 10, 0 };

    bloc_actual = "B2 connexio";

    ret = initSystem(ip, WHEEL_VEL_MODE, &cfg);
    if (ret != NO_ERROR) {
        cap_falla("initSystem contra el robot ha fallat");
        fprintf(stderr, "  IP provada: %s   Retorn: %d\n", ip, ret);
        fprintf(stderr, "  Que vol dir: o la IP no es la del LCD, o el driver\n");
        fprintf(stderr, "    qbot_platform_driver_physical no esta carregat (port 18888 tancat),\n");
        fprintf(stderr, "    o no estas a la wifi Quanser_UVS.\n");
        return 0;
    }
    n_check++;                  /* connectar compta com una comprovacio */
    /* Marge perque el fil d'actualitzacio ompli les variables globals. */
    espera_ms(500);
    return 1;
}

/* B3 - Tots els getters amb el robot QUIET. Valida rangs fisics.
 * Els getters tornen la darrera lectura del fil d'actualitzacio, no toquen
 * la xarxa: es llegeixen tots d'un cop i despres es comproven. */
static void bloc3_getters_repos(void) {
    WheelPair vel, volt, cur;
    Vector3D  acc, gyr;
    double    bat, modul;

    bloc_actual = "B3 getters en repos";

    vel  = getWheelSpeeds();
    volt = getWheelCMDvoltages();
    cur  = getWheelCurrents();
    acc  = getRobotAccelero();
    gyr  = getRobotGyroscop();
    bat  = getBatteryLevel();

    check_rang(vel.dreta,    -0.2, 0.2, "velocitat roda dreta en repos", "rad/s",
               "el robot es mou sol, o l'ultima consigna no s'ha netejat.");
    check_rang(vel.esquerra, -0.2, 0.2, "velocitat roda esquerra en repos", "rad/s",
               "el robot es mou sol, o l'ultima consigna no s'ha netejat.");

    /* El modul de l'acceleracio amb el robot quiet ha de ser la gravetat. */
    modul = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    check_rang(modul, G_ESPERADA - G_TOLERANCIA, G_ESPERADA + G_TOLERANCIA,
               "modul de l'acceleracio en repos", "m/s2",
               "l'IMU no llegeix be, o les unitats no son m/s2, o el robot no esta quiet.");
    check_rang(acc.z, 8.0, 11.5, "component Z de l'acceleracio", "m/s2",
               "si el modul es correcte pero la Z no, els eixos estan intercanviats.");

    check_rang(gyr.x, -0.5, 0.5, "gir X en repos", "rad/s", "deriva del giroscopi o robot en moviment.");
    check_rang(gyr.y, -0.5, 0.5, "gir Y en repos", "rad/s", "deriva del giroscopi o robot en moviment.");
    check_rang(gyr.z, -0.5, 0.5, "gir Z en repos", "rad/s", "deriva del giroscopi o robot en moviment.");

    check_rang(bat, 0.0, 100.0, "bateria dins de rang", "%",
               "voltsToPercentage esta malament o rep volts fora de l'escala LFP 11.5-13.4V.");
    if (bat < 30.0) {
        cap_avis("bateria baixa");
        fprintf(stderr, "  Bateria: %.1f%%. Per sota del 30%% el firmware pot apagar el robot\n", bat);
        fprintf(stderr, "    enmig d'una prova. Carrega'l.\n");
    }

    check_rang(volt.dreta,    -13.5, 13.5, "voltatge comandat roda dreta", "V", "fora de l'escala de la bateria.");
    check_rang(volt.esquerra, -13.5, 13.5, "voltatge comandat roda esquerra", "V", "fora de l'escala de la bateria.");
    check_rang(cur.dreta,     -5.0,  5.0,  "corrent roda dreta en repos", "A", "consum impossible amb el motor aturat.");
    check_rang(cur.esquerra,  -5.0,  5.0,  "corrent roda esquerra en repos", "A", "consum impossible amb el motor aturat.");
}

/* B4 - getRobotState ha de dir el mateix que els getters individuals.
 * getRobotState() omple tots els camps d'un cop; els getters els llegeixen
 * un a un. Si les dues rutes discrepen, hi ha un error de mapatge d'index
 * (dreta/esquerra girades, eixos creuats, volts en comptes de %). */
static void bloc4_coherencia_estat(void) {
    RobotState s;
    WheelPair  pos;
    Vector3D   acc;
    double     bat;

    bloc_actual = "B4 coherencia getRobotState";

    s   = getRobotState();
    pos = getWheelPositions();
    acc = getRobotAccelero();
    bat = getBatteryLevel();

    /* Tolerancies amplies: entre crida i crida el fil ha refrescat les dades. */
    check_igual(s.position.dreta, pos.dreta, 0.5,
                "getRobotState.position.dreta vs getWheelPositions().dreta",
                "les dues rutes llegeixen index diferents: una de les dues te les rodes girades.");
    check_igual(s.position.esquerra, pos.esquerra, 0.5,
                "getRobotState.position.esquerra vs getWheelPositions().esquerra",
                "les dues rutes llegeixen index diferents: una de les dues te les rodes girades.");
    check_igual(s.accelero.z, acc.z, 1.0,
                "getRobotState.accelero.z vs getRobotAccelero().z",
                "els eixos de l'IMU no es mapegen igual a les dues funcions.");
    check_igual(s.battery, bat, 5.0,
                "getRobotState.battery vs getBatteryLevel()",
                "una de les dues no aplica voltsToPercentage: una dona volts i l'altra %.");
}

/* B5 - El fil d'actualitzacio esta viu? Dues lectures separades han de diferir
 * pel soroll de l'IMU. Si son identiques bit a bit, el fil no refresca. */
static void bloc5_dades_fresques(void) {
    Vector3D a1, a2;
    bloc_actual = "B5 dades fresques";

    a1 = getRobotAccelero();
    espera_ms(400);
    a2 = getRobotAccelero();

    /* Comparacio exacta a proposit: dues lectures d'una IMU real mai
     * coincideixen bit a bit. */
    if (a1.x == a2.x && a1.y == a2.y && a1.z == a2.z) {
        cap_avis("l'acceleracio no ha canviat gens en 400 ms");
        fprintf(stderr, "  Valors: x=%.6f y=%.6f z=%.6f (identics a les dues lectures)\n", a1.x, a1.y, a1.z);
        fprintf(stderr, "  Nota: amb soroll real d'IMU aixo es molt improbable. Pot voler dir\n");
        fprintf(stderr, "    que el fil d'actualitzacio esta bloquejat al recv() o mort.\n");
    }
    n_check++;                  /* compta com a comprovacio feta, avisi o no */
}

/* B6 - Setters: validacio d'entrada. NO mou el robot.
 * Prova setLED_Color() amb punter nul i colors fora de rang, i setWheelSpeeds()
 * amb consigna zero i amb una consigna absurda. */
static void bloc6_setters_validacio(void) {
    double color_ok[NUM_LED_COLORS];
    double color_fora[NUM_LED_COLORS];

    bloc_actual = "B6 validacio de setters";

    /* El llindar de bateria (camp de QBotConfig) no es prova aqui: initSystem()
     * el saneja per dins (avis per stderr i valor per defecte) sense retornar
     * ERROR, i no hi ha cap getter per observar-ho des de fora de l'API. */

    check_ret(setLED_Color(TRUE, NULL), ERROR, "setLED_Color(TRUE, NULL)",
              "l'API deferenciara un punter nul: segfault garantit.");
    check_ret(setLED_Color(FALSE, NULL), NO_ERROR, "setLED_Color(FALSE, NULL)",
              "apagar el LED no hauria de necessitar cap color.");

    /* Aquest cas fa que l'API imprimeixi 2 avisos seus: son ESPERATS. */
    fprintf(stderr, "\n[CONTEXT] Ara es proven colors fora de rang: els 2 avisos\n");
    fprintf(stderr, "          de l'API que venen a continuacio son ESPERATS.\n");
    color_fora[0] =  2.0;   /* per sobre de 1.0 */
    color_fora[1] = -1.0;   /* per sota de 0.0  */
    color_fora[2] =  0.5;
    check_ret(setLED_Color(TRUE, color_fora), NO_ERROR, "setLED_Color amb RGB fora de rang",
              "hauria de retallar els valors a [0,1] i continuar, no fallar.");

    color_ok[0] = 0.0; color_ok[1] = 0.0; color_ok[2] = 0.0;
    check_ret(setLED_Color(FALSE, color_ok), NO_ERROR, "apagar el LED", "no pot fallar.");

    check_ret(setWheelSpeeds(0.0, 0.0), NO_ERROR, "setWheelSpeeds(0,0)", "aturar no pot fallar.");

    /* setWheelSpeeds NO valida res: ho documentem com a forat conegut. */
    if (setWheelSpeeds(1000.0, -1000.0) == NO_ERROR) {
        cap_avis("setWheelSpeeds accepta 1000 rad/s sense queixar-se");
        fprintf(stderr, "  Nota: no hi ha limit de velocitat a l'API. El robot no ho fara\n");
        fprintf(stderr, "    (el firmware satura) pero la llibreria hauria de validar-ho.\n");
        fprintf(stderr, "    Es un forat conegut, no un error d'aquesta execucio.\n");
    }
    setWheelSpeeds(0.0, 0.0);   /* deixar-ho segur */
}

/* B7 - Moviment real. Comprova que les rodes es mouen i que ho fan igual.
 * Sequencia: posicio inicial -> consigna 0,5 rad/s -> 4 s -> velocitat ->
 * 1 s -> posicio final -> aturar. Despres comprova la velocitat a regim,
 * que cada roda ha avancat, que han avancat igual i que s'han aturat. */
static void bloc7_moviment(void) {
    WheelPair pos0, pos1, vel;
    double d_dreta, d_esq, mitjana;

    bloc_actual = "B7 moviment real";

    /* Sense confirmacio humana NO es mou res: el robot podria estar sobre la taula. */
    if (!pausa("ATENCIO: el robot es moura endavant 5 s. Deixa 1 metre lliure.")) {
        fprintf(stderr, "[AVIS] B7 saltat: ningu ha confirmat que el robot pot moure's.\n");
        n_avis++;
        return;
    }

    pos0 = getWheelPositions();
    setWheelSpeeds(0.5, 0.5);

    /* El controlador del robot rampa la tensio: amb consigna 0,5 rad/s la roda
     * triga uns segons a arribar-hi. Es llegeix la velocitat als 4 s. */
    espera_ms(4000);
    vel = getWheelSpeeds();     /* a mitja marxa: ha d'anar a la consigna */

    espera_ms(1000);
    pos1 = getWheelPositions();
    setWheelSpeeds(0.0, 0.0);

    check_rang(vel.dreta,    0.2, 0.9, "velocitat real roda dreta amb consigna 0.5", "rad/s",
               "la roda no segueix la consigna: motor, driver o mapatge d'index.");
    check_rang(vel.esquerra, 0.2, 0.9, "velocitat real roda esquerra amb consigna 0.5", "rad/s",
               "la roda no segueix la consigna: motor, driver o mapatge d'index.");

    d_dreta = pos1.dreta    - pos0.dreta;       /* radians girats en 5 s */
    d_esq   = pos1.esquerra - pos0.esquerra;

    check_true(d_dreta > 0.5, "la roda dreta ha avancat",
               "la posicio no creix: o la roda no gira o l'encoder no compta.");
    check_true(d_esq   > 0.5, "la roda esquerra ha avancat",
               "la posicio no creix: o la roda no gira o l'encoder no compta.");

    /* Simetria: nomes te sentit si les dues rodes han girat de veritat. */
    if (d_dreta > 0.5 && d_esq > 0.5) {
        mitjana = 0.5 * (d_dreta + d_esq);
        check_igual(d_dreta, d_esq, 0.30 * mitjana,
                    "les dues rodes han avancat el mateix (marge 30%)",
                    "una roda arrossega mes que l'altra: el robot no anira recte.");
    }

    espera_ms(500);
    vel = getWheelSpeeds();
    check_rang(vel.dreta,    -0.2, 0.2, "roda dreta aturada despres de setWheelSpeeds(0,0)", "rad/s",
               "l'ordre d'aturada no arriba al robot. PERILLOS.");
    check_rang(vel.esquerra, -0.2, 0.2, "roda esquerra aturada despres de setWheelSpeeds(0,0)", "rad/s",
               "l'ordre d'aturada no arriba al robot. PERILLOS.");
}

/* B8 - LED. Unic bloc que necessita els teus ulls: encen el LED en blau,
 * espera ENTER, el torna al robot i espera ENTER una altra vegada. */
static void bloc8_led_visual(void) {
    double blau[NUM_LED_COLORS];
    bloc_actual = "B8 LED (visual)";

    blau[0] = 0.0; blau[1] = 0.0; blau[2] = 1.0;
    check_ret(setLED_Color(TRUE, blau), NO_ERROR, "encendre el LED blau", "la crida ha fallat.");

    pausa("MIRA EL ROBOT: el LED hauria d'estar BLAU. Mira'l tota l'estona que vulguis.");

    /* OJO: FALSE no apaga el LED. setLED_Color amb enableUserLED=FALSE nomes deixa
     * de fer servir el color de l'usuari (no toca LED_ColorAPI), i el robot recupera
     * el seu LED de sistema, que es VERD. */
    blau[2] = 0.0;
    check_ret(setLED_Color(FALSE, blau), NO_ERROR, "tornar el LED al control del robot",
              "la crida ha fallat.");

    pausa("El LED hauria d'haver passat a VERD (el del robot). Si segueix blau, apunta-ho.");
}

/* B9 - Sistema d'esdeveniments: la cua POSIX ha de rebre BATERIA_BAIXA.
 * El llindar de bateria nomes es pot fixar a initSystem() (camp de QBotConfig),
 * aixi que el bloc tanca el sistema i el reobre amb el llindar al 100%.
 * Deixa el sistema OBERT amb llindar 100% - qui el tanca es B10. */
static void bloc9_esdeveniments(const char *ip) {
    EVENT_T event = CAP_EVENT;
    struct timespec limit;
    ssize_t rebut;
    double bat;
    /* Llindar al 100%: el fil compara amb <=, aixi que amb qualsevol nivell de
     * bateria (fins i tot 100%) l'alarma ha de saltar. Amb 99% el test donaria
     * un fals negatiu si el robot estigues acabat de carregar. */
    QBotConfig cfg9 = { ESDE_BATERIA, 100.0, 10, 0 };

    bloc_actual = "B9 esdeveniments (cua POSIX)";

    bat = getBatteryLevel();

    /* Reobrim amb el llindar al 100%. */
    if (closeSystem() != NO_ERROR) {
        cap_falla("closeSystem previ a B9 ha fallat");
        return;
    }
    if (initSystem(ip, WHEEL_VEL_MODE, &cfg9) != NO_ERROR) {
        cap_falla("no s'ha pogut reobrir el sistema amb el llindar al 100%");
        return;
    }

    /* mq_timedreceive vol un instant absolut, no una durada: ara + 3 s. */
    clock_gettime(CLOCK_REALTIME, &limit);
    limit.tv_sec += 3;

    /* Espera bloquejant fins que arribi un event a la cua o venci el termini.
     * Retorna els bytes rebuts, o -1 si no ha arribat res (errno = ETIMEDOUT). */
    rebut = mq_timedreceive(cfg9.cua, (char *)&event, sizeof(EVENT_T), NULL, &limit);

    if (rebut == -1) {
        cap_falla("no ha arribat cap esdeveniment de bateria en 3 segons");
        fprintf(stderr, "  Bateria actual: %.1f%%   Llindar posat: 100%%\n", bat);
        fprintf(stderr, "  errno: %d (%s)\n", errno, strerror(errno));
        fprintf(stderr, "  Que vol dir: el fil no genera l'event, o el bitmask ESDE_BATERIA\n");
        fprintf(stderr, "    no esta actiu, o la cua no s'ha creat (has llancat 'mqueue &'?).\n");
    } else {
        n_check++;
        check_true(event == BATERIA_BAIXA, "l'esdeveniment rebut es BATERIA_BAIXA",
                   "arriba un event pero amb el codi equivocat.");
    }
}

/* B10 - Tancament ordenat. closeSystem() ha de retornar NO_ERROR i trigar
 * menys de 2 s: es cronometra amb el rellotge del sistema abans i despres. */
static void bloc10_tancament(void) {
    struct timespec t0, t1;
    double segons;
    int ret;

    bloc_actual = "B10 tancament";

    printf("\n>>> Tancant. A partir d'aqui NO hauria de sortir cap [ERROR XARXA].\n");
    fflush(stdout);

    clock_gettime(CLOCK_REALTIME, &t0);
    ret = closeSystem();
    clock_gettime(CLOCK_REALTIME, &t1);

    /* Diferencia t1 - t0 en segons: la part sencera mes els nanosegons / 1e9. */
    segons = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1.0e9;

    check_ret(ret, NO_ERROR, "closeSystem ha retornat NO_ERROR",
              "algun recurs no s'ha alliberat: mira quin perror ha sortit just abans.");
    check_rang(segons, 0.0, 2.0, "temps de tancament", "s",
               "closeSystem es queda penjat: el recv() no es desbloqueja amb shutdown().");
}

/* ========================================================================= */
/* MAIN                                                                      */
/* ========================================================================= */

/* Ordre d'execucio: capcalera per pantalla -> B1 -> B2 (si falla, avorta amb
 * codi 1) -> B3..B9 -> B10 -> resum per pantalla.
 * Codi de sortida: 0 si cap falla, 1 si n'hi ha (els avisos no compten). */
int main(int argc, char *argv[]) {
    const char *ip = DEFAULT_IP;
    if (argc > 1) {
        ip = argv[1];           /* IP del robot, la del LCD */
    }

    printf("=== SMOKE TEST API QBot =====================================\n");
    printf("build: %s %s   |   IP: %s\n", __DATE__, __TIME__, ip);
    printf("Silenci = tot be. Nomes surten els problemes.\n");
    printf("=============================================================\n");
    fflush(stdout);             /* que la capcalera surti abans de tocar el robot */

    bloc1_parametres_invalids();

    /* Sense connexio no hi ha res mes a provar. */
    if (!bloc2_connexio(ip)) {
        printf("\n=== RESULTAT: AVORTAT ======================================\n");
        printf("Sense connexio no es pot provar res mes. %d falla(des).\n", n_falla);
        return 1;
    }

    bloc3_getters_repos();
    bloc4_coherencia_estat();
    bloc5_dades_fresques();
    bloc6_setters_validacio();
    bloc7_moviment();
    bloc8_led_visual();
    bloc9_esdeveniments(ip);
    bloc10_tancament();         /* tanca el sistema que B2/B9 han obert */

    printf("\n=============================================================\n");
    if (n_falla == 0 && n_avis == 0) {
        printf("RESULTAT: %d/%d OK. Cap problema.\n", n_check, n_check);
    } else {
        printf("RESULTAT: %d comprovacions, %d FALLADES, %d avisos.\n", n_check, n_falla, n_avis);
    }
    printf("=============================================================\n");

    if (n_falla > 0) {
        return 1;
    }
    return 0;
}
