#ifndef _APIBASICA_H
#define _APIBASICA_H
#include <mqueue.h> // Llibreria per a les cues de missatges


// Suma un intervalo a un instante absoluto (los dos en struct timespec). Da el
// PROXIMO instante de activacion de un bucle periodico sin deriva acumulada:
// t(k+1) = t(k) + T, nunca "ahora + T".
#define timespecPlusTimeInterval(adr_ts_out, adr_ts_in, adr_ts_timeinterval) nsec2timespec(adr_ts_out, timespec2nsec(adr_ts_in)+timespec2nsec(adr_ts_timeinterval))

// Tamanos de los vectores del protocolo Quanser.
#define NUM_LED_COLORS 3
#define NUM_VEL_CMD 2
#define NUM_WHEELS 2
#define NUM_CMD_VOLT 2
#define NUM_ACCELERO 3
#define NUM_GYROSCOP 3
#define NUM_CURRENT 2

// Medidas fisicas del robot (las usa la cinematica de la API y quedan
// disponibles para la aplicacion).
#define WHEEL_RADIUS 0.04445   // r: radio de la rueda, en metros
#define WHEEL_BASE   0.3928    // L: distancia entre centros de las dos ruedas, en metros

// Codis de retorn de les funcions publiques
#define ERROR    -1
#define NO_ERROR  0

// ---------------------------------------------------------------------------
// TIPUS D'ESDEVENIMENTS
// ---------------------------------------------------------------------------
// Bitmask per a QBotConfig.tipusEsdeveniments, combinable amb OR.
#define ESDE_BATERIA          0x01   // Alarma de bateria baixa (implementat)
#define ESDE_LIDAR_OBSTACLE   0x02   // Obstacle detectat pel LIDAR (implementat)
#define ESDE_DESCONNEXIO      0x04   // Perdua de connexio amb el robot (reservat)

// ---------------------------------------------------------------------------
// CONFIGURACIO D'INITSYSTEM
// ---------------------------------------------------------------------------
// Els quatre primers camps son d'entrada i els omple l'usuari abans de cridar
// initSystem(); 'cua' es de sortida i l'omple initSystem(). Amb config == NULL
// no hi ha events: no es rep cap descriptor i la API no en genera cap.
// Amb ESDE_LIDAR_OBSTACLE a la mascara, initSystem() es connecta al pont del
// LiDAR (pont_lidar.py al Jetson, port 18889, mateixa IP que el robot) i posa
// LIDAR_OBSTACLE a la cua quan la distancia frontal baixa de distObstacle.
// ---------------------------------------------------------------------------
typedef struct {
    unsigned char tipusEsdeveniments; // [in]  Bitmask d'events actius (ESDE_*)
    double        limitBateria;       // [in]  Llindar d'alarma de bateria en % [0..100]
    int           capacitatCua;       // [in]  Nombre maxim de missatges a la cua POSIX
    double        distObstacle;       // [in]  Distancia frontal (m) per sota de la qual el LiDAR avisa d'obstacle
    mqd_t         cua;                // [out] Descriptor de la cua, el retorna initSystem()
} QBotConfig;

// Modo de velocidad que se le pide al robot en initSystem():
//   WHEEL_VEL_MODE -> se comanda cada rueda por separado (rad/s)
//   ROBOT_VEL_MODE -> se comanda el cuerpo del robot (lineal + angular)
// Viaja en requestMsg[0] combinado con TEACHING_MODE/RESEARCH_MODE.
typedef enum {WHEEL_VEL_MODE, ROBOT_VEL_MODE} VEL_MODE;

//Tipus boolea
typedef enum {FALSE,TRUE} BOOL;


// El nostre diccionari d'esdeveniments
typedef enum {
    BATERIA_BAIXA,     // bateria per sota de limitBateria
    LIDAR_OBSTACLE,    // obstacle frontal a menys de distObstacle (via el pont del LiDAR)
    CAP_EVENT          // l'aplicacio pot numerar els seus a partir d'aqui
} EVENT_T;

// Parella de valors associats a les dues rodes del robot.
typedef struct {
    double esquerra;   // roda esquerra = responseMsg index 0
    double dreta;      // roda dreta    = responseMsg index 1
} WheelPair;

// Vector tridimensional per a sensors triaxials (IMU).
typedef struct {
    double x;
    double y;
    double z;
} Vector3D;

// Estat global del robot en un instant.
typedef struct {
    WheelPair position;   // rad
    WheelPair speed;      // rad/s
    WheelPair voltage;    // V (voltatge comandat als motors)
    WheelPair current;    // A (corrent consumit per motor)
    Vector3D  accelero;   // m/s^2
    Vector3D  gyroscop;   // rad/s
    double    battery;    // percentatge 0..100
} RobotState;

// Velocidad del CUERPO del robot, no de cada rueda por separado.
typedef struct {
    double v;      // m/s   velocitat lineal del cos del robot
    double omega;  // rad/s velocitat angular del cos del robot
} RobotVelocity;

// Pose del robot en el plano. La integra el hilo interno paquete a paquete,
// desde el origen que fije setOdometry() o desde (0,0,0) si no se llama nunca.
typedef struct {
    double x;    // m
    double y;    // m
    double phi;  // rad
} Odometry;

// Cola POSIX /cua_qbot: unico canal API -> aplicacion. El hilo interno hace
// mq_send al detectar bateria baja, el hilo del LiDAR al detectar un obstaculo,
// y la aplicacion los recoge con mq_receive (bloqueante). Su descriptor se
// obtiene en QBotConfig.cua.

// Inicialitza la connexio amb el robot, el sistema d'esdeveniments i, si es
// demana ESDE_LIDAR_OBSTACLE, la connexio amb el pont del LiDAR (si el pont no
// respon, initSystem() retorna ERROR i no deixa res obert).
// Amb config == NULL s'apliquen els valors per defecte i NO es retorna cap cua.
// 'config' no pot ser const: 'cua' es un parametre de sortida.
// Retorna NO_ERROR (0) / ERROR (-1), com la resta de funcions publiques.
int initSystem(const char *qbot_ip,
               VEL_MODE velMode,
               QBotConfig *config);
int closeSystem(void);

// ---------------------------------------------------------------------------
// GETTERS
// ---------------------------------------------------------------------------
// Cada uno toma mutexAPI por dentro, copia y lo suelta: el llamante nunca ve un
// valor a medio escribir. getRobotState() lee TODO bajo un unico bloqueo, asi
// que sus campos son coherentes entre si; encadenar getters sueltos no lo es.
// Devuelven la struct por valor y por eso NO pueden senalizar error.
// ---------------------------------------------------------------------------
WheelPair  getWheelPositions(void);
WheelPair  getWheelSpeeds(void);
WheelPair  getWheelCMDvoltages(void);
WheelPair  getWheelCurrents(void);
Vector3D   getRobotAccelero(void);
Vector3D   getRobotGyroscop(void);
// Nivell de bateria en percentatge [0..100]. Mapping lineal entre 11.5V (0%) i
// 13.4V (100%).
double     getBatteryLevel(void);
RobotState getRobotState(void);

// Las dos vias de calcular la velocidad del cuerpo:
//   getRobotVelocity()    -> a partir de las velocidades de rueda que reporta
//                            el robot (wheelSpeedAPI).
//   getRobotVelocityPos() -> a partir del incremento de posicion medido por el
//                            hilo interno, dividido por el dt real del reloj.
RobotVelocity getRobotVelocity(void);
RobotVelocity getRobotVelocityPos(void);

// Las dos odometrias se integran en paralelo y solo se diferencian en de donde
// sale phi: del giroscopo (getOdometry) o de las ruedas (getOdometryWheels).
// La x,y de las dos sale del incremento de posicion de los encoders.
Odometry getOdometry(void);
Odometry getOdometryWheels(void);

// ---------------------------------------------------------------------------
// SETTERS
// ---------------------------------------------------------------------------
// Devuelven NO_ERROR (0) / ERROR (-1). No hablan con el robot: dejan el valor
// en la variable global bajo mutexAPI y lo transmite el hilo interno en su
// siguiente vuelta (<= ~50 ms), asi que son asincronos.
// ---------------------------------------------------------------------------
int setLED_Color(BOOL enableUserLED, double *pLED_Color);

int setWheelSpeeds(double esquerra, double dreta); // El 1r parametre va a l'index 0 = roda ESQUERRA real.

// Cinematica inversa: reparte (v, omega) entre las dos ruedas y termina
// llamando a setWheelSpeeds(), que aplica la saturacion a MAX_VEL_RODA y sus
// avisos.
int setRobotVelocity(double v, double omega);

// Fija el origen de las DOS odometrias a la misma pose. Devuelve siempre NO_ERROR.
int setOdometry(double x, double y, double phi);
#endif
