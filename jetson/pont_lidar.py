#!/usr/bin/env python3
# ===========================================================================
#  pont_lidar.py  --  PONT LiDAR  Jetson Orin Nano (Ubuntu)  ->  QNX
#  TFG Roger Llinares Raichs - QBot Platform / QNX - accio A69
#
#  ESTAT: PROVAT AL LABORATORI (17/09/2026) al Jetson, contra el sensor M10P
#  i la maquina QNX: desfase 92 graus, escala 1,0, ~12 Hz. Validat per Roger.
# ===========================================================================
#
#  QUE FA
#  ------
#  Un bucle i un socket. Llegeix el LSLiDAR M10P amb l'API de Quanser, es
#  queda amb LA DISTANCIA MINIMA VALIDA d'un sector frontal, i l'escup per
#  un socket TCP, una linia per barrido. Res mes. El QNX obre aquest socket
#  amb els mateixos sockets POSIX que ja fa servir per al :18888.
#
#  EL FORMAT DEL CABLE, I PER QUE AQUEST
#  -------------------------------------
#  Una linia de text ASCII acabada en salt de linia, amb la distancia en
#  metres i tres decimals:
#
#      0.734          <- obstacle mes proper del sector, a 0,734 m
#      -1.000         <- CAP punt valid al sector: "no veig res"
#
#  Tres motius, per ordre d'importancia:
#
#  1. EL SALT DE LINIA FA DE MARCA DE TRAMA. recv() no garanteix el missatge
#     sencer: el 10/09/2026 una lectura partida del :18888 va deixar el
#     socket desalineat per sempre i va costar un dia de laboratori (A88).
#     Amb una marca de final, una lectura partida NO pot desincronitzar
#     res: el lector nomes ha de seguir llegint fins al salt de linia
#     seguent. El problema desapareix per construccio, no per un bucle de
#     re-sincronitzacio.
#  2. Es trivial de parsejar des de C: atof() sobre la linia i ja esta. Cap
#     ordre de bytes, cap alineacio de doubles, cap endianisme entre
#     l'ARM64 del Jetson i la maquina QNX.
#  3. Es llegible per una persona. Al laboratori es pot provar el pont
#     SENSE el binari QNX, amb  nc <ip_jetson> 18889  des de qualsevol
#     maquina. Aixo separa "el pont va" de "la integracio va", que es
#     justament el que ha de fer una prova.
#
#  El -1 de "no veig res" NO pot ser 0: un zero es llegiria com un obstacle
#  enganxat al robot. Al lab del 04/09 fins al 31 % dels punts d'un barrido
#  tenien qualitat bona i distancia 0,0 (AI_lidar-lab-04-09.md, seccio 9.4).
#
#  ELS CINC REQUISITS MESURATS EL 04/09, I ON ES COMPLEIXEN
#  --------------------------------------------------------
#  1. device='/dev/lidar' i regla udev  -> --device, i el pas 1 de la
#     llista de laboratori (00 Docs/AI_lidar-checklist-lab.md)
#  2. descartar les primeres lectures   -> --descarta
#  3. filtrar quality==0, distancia 0 i
#     fora de rang                      -> minim_sector()
#  4. max_interpolated_angle >= 1.0     -> --interp-angle, amb guarda
#  5. el 0 del sensor no mira endavant  -> --desfase
#
#  LES DUES CONSTANTS QUE NO ES PODEN SABER DES DE CASA
#  ----------------------------------------------------
#  Son els dos primers arguments de la llista de sota: --desfase i
#  --escala. Es canvien a la linia d'ordres, sense tocar ni una linia de
#  codi ni compilar res. Com es mesuren: passos 5 i 6 de la llista de
#  laboratori.
#
#  FONTS
#  -----
#  00 Docs/AI_lidar-lab-04-09.md  4.2, 4.3, 9.1 a 9.8  (API i mesures)
#  02 References/AI_lidar-m10p-research.md  1          (specs del sensor)
# ===========================================================================

import argparse
import math
import socket
import sys
import time

# Valor que viatja pel cable quan no hi ha cap punt valid al sector.
# NO pot ser 0: un zero es llegiria com un obstacle a distancia zero.
NO_VIST = -1.0


def avis(text):
    """Tot el que no son dades va a stderr.

    El socket porta nomes numeros; els missatges d'estat van per stderr
    perque es puguin mirar a la terminal del Jetson sense embrutar el cable
    ni desincronitzar res.
    """
    sys.stderr.write(text + "\n")
    sys.stderr.flush()


def arguments():
    p = argparse.ArgumentParser(
        description="Pont LiDAR M10P -> socket TCP (una distancia per linia)")

    # --- Les DUES constants sense mesurar. Van primer a proposit. ---------
    p.add_argument("--desfase", type=float, default=90.0, metavar="GRAUS",
                   help="SENSE MESURAR. Angle del sensor que mira al FRONT "
                        "del robot. El 0 del LiDAR no mira endavant: el "
                        "04/09, amb un objecte just al davant, el sensor el "
                        "situava cinc vegades entre 89,5 i 91,7 graus. "
                        "Aquests 90 son aquella observacio a ull, NO una "
                        "mesura. Pas 5 de la llista de laboratori. "
                        "(per defecte: 90.0)")
    p.add_argument("--escala", type=float, default=1.0, metavar="FACTOR",
                   help="SENSE CALIBRAR. Factor que multiplica la distancia "
                        "que dona la llibreria. L'1,0 vol dir 'em crec el "
                        "sensor'. El 04/09 nomes es va poder comprovar "
                        "l'ordre de magnitud i el signe de la resposta, no "
                        "l'escala. Pas 6 de la llista de laboratori. "
                        "(per defecte: 1.0)")

    # --- Sector frontal i filtres ----------------------------------------
    p.add_argument("--sector", type=float, default=20.0, metavar="GRAUS",
                   help="SEMIAMPLADA del sector frontal. Amb 20 es mira de "
                        "--desfase-20 a --desfase+20, o sigui 40 graus de "
                        "con. (per defecte: 20.0)")
    p.add_argument("--dist-min", type=float, default=0.15, metavar="M",
                   help="Per sota d'aixo la mesura es descarta. El fabricant "
                        "NO dona abast minim; aquest 0,15 esta triat per "
                        "sobre dels valors impossibles vistos el 04/09 "
                        "(de 0,002 a 0,093 m). (per defecte: 0.15)")
    p.add_argument("--dist-max", type=float, default=10.0, metavar="M",
                   help="Per sobre d'aixo la mesura es descarta. Full de "
                        "specs: 10 m amb 10 %% de reflectivitat. El 04/09 "
                        "van sortir punts de 31,483 m, impossibles. "
                        "(per defecte: 10.0)")

    # --- Sensor i lectura ------------------------------------------------
    p.add_argument("--device", default="/dev/lidar", metavar="RUTA",
                   help="La llibreria de Quanser exigeix literalment "
                        "/dev/lidar: ni /dev/ttyACM0 ni /dev/lidar0 li "
                        "valen, tots dos donen error -10. "
                        "(per defecte: /dev/lidar)")
    p.add_argument("--mostres", type=int, default=1680, metavar="N",
                   help="samples_per_scan. El maxim vist en un barrido real "
                        "van ser 1671 mesures. (per defecte: 1680)")
    p.add_argument("--descarta", type=int, default=20, metavar="N",
                   help="Barridos que es llencen just despres d'obrir. Els "
                        "primers surten incomplets i amb minims fisicament "
                        "impossibles. 20 barridos son uns 1,7 s a 12 Hz. "
                        "(per defecte: 20)")
    p.add_argument("--mode", choices=["normal", "interpolated"],
                   default="normal",
                   help="normal: mesures crues, el nombre varia barrido a "
                        "barrido. interpolated: sempre --mostres mesures amb "
                        "els angles fixos. Aqui es filtra pel VALOR de "
                        "l'angle i no per l'index, aixi que NORMAL ja val i "
                        "estalvia els dos llindars d'interpolacio, que el "
                        "04/09 van deixar els 720 punts a zero. "
                        "(per defecte: normal)")
    p.add_argument("--interp-dist", type=float, default=1.0, metavar="M",
                   help="max_interpolated_distance, nomes en mode "
                        "interpolated. (per defecte: 1.0)")
    p.add_argument("--interp-angle", type=float, default=1.0, metavar="X",
                   help="max_interpolated_angle, nomes en mode interpolated. "
                        "MESURAT el 04/09: amb 0,1 les 720 mesures surten a "
                        "ZERO i amb 1,0 nomes un 11 %%. El programa NO deixa "
                        "baixar de 1,0. (per defecte: 1.0)")

    # --- Socket i ritme --------------------------------------------------
    p.add_argument("--port", type=int, default=18889, metavar="PORT",
                   help="Port TCP on escolta el pont. 18889 es el vei del "
                        "18888 del driver de Quanser, per recordar que son "
                        "dos canals diferents. (per defecte: 18889)")
    p.add_argument("--periode", type=float, default=0.02, metavar="S",
                   help="Pausa entre intents de lectura. read() NO es "
                        "bloquejant: sense pausa torna 0 vint vegades "
                        "seguides, perque el barrido tarda 83 ms. "
                        "(per defecte: 0.02)")
    p.add_argument("--estat", type=float, default=2.0, metavar="S",
                   help="Cada quants segons escriu una linia d'estat per "
                        "stderr. Amb 0 no n'escriu cap. (per defecte: 2.0)")

    # --- Mode calibratge -------------------------------------------------
    p.add_argument("--calibra", action="store_true",
                   help="Mode per MESURAR les dues constants de dalt. Mira "
                        "els 360 graus sencers (ignora --sector) i escriu "
                        "un cop per segon la distancia i L'ANGLE del punt "
                        "mes proper. Amb un objecte just davant del robot, "
                        "aquell angle ES el --desfase, i comparant la "
                        "distancia amb la cinta metrica surt l'--escala. "
                        "Passos 5 i 6 de la llista de laboratori.")

    # --- Gravacio de barridos sencers ------------------------------------
    p.add_argument("--grava", metavar="FITXER",
                   help="Desa a FITXER els --grava-n primers barridos sencers "
                        "despres dels descartats, SENSE filtrar ni escalar: "
                        "una fila per punt amb barrido, angle en graus, "
                        "distancia en m i qualitat, separats per tabulador. "
                        "Es l'evidencia dels passos 5 i 6 de la llista de "
                        "laboratori (figura polar i xifres del desfase i "
                        "l'escala). El pont segueix servint el socket "
                        "mentre grava i despres.")
    p.add_argument("--grava-n", type=int, default=10, metavar="N",
                   help="Barridos que es desen amb --grava. (per defecte: 10)")

    # --- Mode fals -------------------------------------------------------
    p.add_argument("--fals", action="store_true",
                   help="NO obre el LiDAR: inventa una distancia que puja i "
                        "baixa entre 0,30 i 1,50 m. Serveix per provar la "
                        "meitat QNX (socket, cua, esdeveniment) encara que "
                        "el sensor no s'obri, sense gastar el dia de "
                        "laboratori esperant a tenir-ho tot alhora. Les "
                        "dades NO son reals i el programa ho diu per stderr "
                        "cada vegada que escriu l'estat.")

    args = p.parse_args()

    # Guarda del requisit 4: mesurat el 04/09, per sota de 1,0 el barrido
    # sencer surt a zero. Val mes parar aqui que passar-se el laboratori
    # mirant un pont que sempre diu "no veig res".
    if args.mode == "interpolated" and args.interp_angle < 1.0:
        p.error("--interp-angle ha de ser >= 1.0 en mode interpolated: "
                "amb 0,1 les 720 mesures surten a zero (mesurat el 04/09)")

    # Calibrar es mirar-ho TOT: si el sector estigues mal orientat -que es
    # justament el que es va a mesurar- no es veuria l'objecte de referencia.
    if args.calibra:
        args.sector = 180.0
    return args


def obre_lidar(args):
    """Obre el sensor i torna (lidar, mesures, mode).

    Tot el que toca l'API de Quanser esta en aquesta funcio i a la crida
    read() de mes avall. La resta del fitxer no en sap res.
    """
    from quanser.devices import (LeishenM10P, RangingMeasurements,
                                 RangingMeasurementMode)

    # URI copiada literalment de l'ajuda de la llibreria (seccio 4.2). La
    # llibreria posa ella mateixa els 512000 baud, que es una velocitat no
    # estandard que stty no sap posar.
    uri = ("serial://localhost:0?device='%s',baud='512000',word='8',"
           "parity='none',stop='1',flow='none'" % args.device)

    lidar = LeishenM10P()
    # 'samples_per_scan' es el nom que accepta open(): el 04/09 es va cridar
    # aixi (lidar_test6.py) i la llibreria el va acceptar.
    lidar.open(uri, samples_per_scan=args.mostres)

    mesures = RangingMeasurements(args.mostres)
    mode = (RangingMeasurementMode.NORMAL if args.mode == "normal"
            else RangingMeasurementMode.INTERPOLATED)
    return lidar, mesures, mode


def minim_sector(mesures, n, args):
    """Torna (distancia, angle) del punt valid mes proper del sector.

    Torna (None, None) si no n'hi ha cap. L'ANGLE nomes es fa servir per
    calibrar: es el que permet mesurar el --desfase al laboratori en dos
    minuts (posar un objecte just al davant i mirar a quin angle surt),
    en comptes de provar valors a cegues.

    Els tres filtres del requisit 3, per ordre de com de sovint disparen:
    qualitat zero, distancia zero, i fora de rang. Es fan ABANS de mirar
    l'angle perque son mes barats.
    """
    minim = None
    angle = None
    for i in range(n):
        # .quality, .distance i .heading s'indexen per posicio i read() omple
        # les n primeres: es com les van llegir lidar_test4.py i lidar_test9.py
        # el 04/09 (m.distance[0:n], m.quality[0:n], m.heading[0:n]).
        if mesures.quality[i] == 0:
            continue                # sense eco: mesura invalida
        d = mesures.distance[i]
        if d <= 0.0:
            continue                # qualitat bona i distancia 0: fins al
                                    # 31 % d'un barrido (seccio 9.4)
        if d < args.dist_min or d > args.dist_max:
            continue                # fora de l'abast del sensor

        # heading ve en radians: el 04/09 anava de 0,0012 a 6,2807.
        graus = math.degrees(mesures.heading[i])
        # Diferencia amb signe i embolcallada a [-180, +180], per no perdre
        # el sector quan cau a cavall del 0/360.
        desviacio = (graus - args.desfase + 180.0) % 360.0 - 180.0
        if abs(desviacio) > args.sector:
            continue                # fora del con frontal

        if minim is None or d < minim:
            minim = d
            angle = graus

    # L'escala s'aplica al final, un sol cop: els llindars de rang son de
    # l'escala del sensor, i l'--escala es la correccio de la mesura.
    if minim is None:
        return None, None
    return minim * args.escala, angle


def obre_gravacio(args):
    """Obre el fitxer de --grava i hi escriu la capcalera. Torna el fitxer."""
    f = open(args.grava, "w")
    f.write("# pont_lidar.py --grava | device %s | mode %s | mostres %d | "
            "descartats %d | desfase %.1f | escala %.4f | sector %.1f\n"
            % (args.device, args.mode, args.mostres, args.descarta,
               args.desfase, args.escala, args.sector))
    f.write("# valors crus de la llibreria: sense filtrar, sense --escala\n")
    f.write("barrido\tangle_graus\tdistancia_m\tqualitat\n")
    return f


def grava_barrido(f, k, mesures, n):
    """Escriu les n mesures del barrido k, una fila per punt."""
    for i in range(n):
        f.write("%d\t%.3f\t%.4f\t%d\n"
                % (k, math.degrees(mesures.heading[i]), mesures.distance[i],
                   mesures.quality[i]))
    f.flush()


def accepta_si_cal(servidor, client):
    """Si no hi ha ningu connectat, mira si algu truca. No bloqueja.

    El bucle del LiDAR no s'atura mai esperant un client: el sensor es
    llegeix igual, hi hagi QNX a l'altra banda o no.
    """
    if client is not None:
        return client
    try:
        nou, adreca = servidor.accept()
    except (BlockingIOError, socket.timeout):
        return None
    except OSError as e:
        avis("[pont] accept ha fallat: %s" % e)
        return None
    nou.settimeout(1.0)     # que un send mai es quedi penjat per sempre
    avis("[pont] client connectat des de %s:%d" % (adreca[0], adreca[1]))
    return nou


def envia(client, valor):
    """Escup una linia. Torna el client, o None si s'ha perdut."""
    if client is None:
        return None
    try:
        client.sendall(("%.3f\n" % valor).encode("ascii"))
        return client
    except (OSError, socket.timeout) as e:
        avis("[pont] client perdut (%s). Torno a esperar-ne un altre." % e)
        try:
            client.close()
        except OSError:
            pass
        return None


def main():
    args = arguments()

    avis("[pont] sector frontal: %.1f +/- %.1f graus | escala %.4f | "
         "rang %.2f-%.2f m | mode %s"
         % (args.desfase, args.sector, args.escala,
            args.dist_min, args.dist_max, args.mode))
    avis("[pont] ATENCIO: --desfase i --escala NO estan mesurats. "
         "Passos 5 i 6 de la llista de laboratori.")
    if args.calibra:
        avis("[pont] MODE CALIBRATGE: miro els 360 graus i escric on es el "
             "punt mes proper. Posa l'objecte just davant del robot.")

    if args.fals:
        avis("[pont] MODE FALS: el LiDAR NO s'obre i les distancies son "
             "INVENTADES. Aixo no mesura res del mon real.")
        lidar, mesures, mode = None, None, None
    else:
        try:
            lidar, mesures, mode = obre_lidar(args)
        except Exception as e:
            # Si no obre, no hi ha res a fer i val mes dir-ho clar i sortir
            # que servir un socket que no dira mai res. El -10 tipic vol dir
            # que el nom del dispositiu no es /dev/lidar (seccio 9.1).
            avis("[pont] no s'ha pogut obrir el LiDAR: %r" % (e,))
            avis("[pont] mirar:  ls -l /dev/lidar   i el pas 1 de la llista")
            avis("[pont] per provar igualment la meitat QNX: afegir --fals")
            return 1
        avis("[pont] LiDAR obert a %s" % args.device)

    gravacio = None             # fitxer de --grava, mentre queden barridos
    gravats = 0
    if args.grava and lidar is not None:
        gravacio = obre_gravacio(args)
        avis("[pont] gravare %d barridos sencers a %s"
             % (args.grava_n, args.grava))
    elif args.grava:
        avis("[pont] --grava s'ignora en mode FALS: no hi ha barridos reals")

    servidor = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    servidor.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    servidor.bind(("", args.port))
    servidor.listen(1)          # un sol client: nomes hi ha una maquina QNX
    servidor.settimeout(0.0)    # accept() que no bloqueja
    avis("[pont] escoltant al port %d" % args.port)

    client = None
    descartats = 0
    fallades = 0                # lectures seguides que han petat
    darrer_estat = time.time()
    darrer_calib = time.time()  # nomes per al mode calibratge
    barridos = 0                # barridos servits des de l'ultima linia d'estat
    buits = 0                   # d'aquests, quants sense cap punt al sector
    darrera = NO_VIST
    angle = None                # angle del punt mes proper, per calibrar
    passa = 0                   # nomes per al mode fals

    # A partir d'aqui el programa no ha de morir mai per res. Una lectura
    # dolenta es una lectura dolenta, no el final del proces.
    while True:
        client = accepta_si_cal(servidor, client)

        if lidar is None:
            # MODE FALS: dent de serra de 0,30 a 1,50 m, al ritme del sensor
            # real (12 Hz). Travessa el llindar dels 0,60 m dues vegades per
            # cicle, que es el que ha de fer botar l'esdeveniment.
            time.sleep(1.0 / 12.0)
            passa = (passa + 1) % 50
            minim = 0.30 + 0.05 * (passa if passa < 25 else 50 - passa)
            angle = args.desfase
        else:
            try:
                # read() no es bloquejant: tornar 0 NO es cap error, vol dir
                # "encara no hi ha barrido nou".
                n = lidar.read(mode, args.interp_dist, args.interp_angle,
                               mesures)
                fallades = 0
            except Exception as e:
                fallades += 1
                if fallades in (1, 10, 100) or fallades % 1000 == 0:
                    avis("[pont] read ha petat %d cops seguits: %r"
                         % (fallades, e))
                time.sleep(args.periode)
                continue

            if not n:
                # Normal: el barrido tarda 83 ms i aixo pregunta cada 20 ms.
                time.sleep(args.periode)
                continue

            if descartats < args.descarta:
                descartats += 1
                if descartats == args.descarta:
                    avis("[pont] descartats els %d primers barridos"
                         % descartats)
                continue

            if gravacio is not None:
                gravats += 1
                grava_barrido(gravacio, gravats, mesures, n)
                if gravats >= args.grava_n:
                    gravacio.close()
                    gravacio = None
                    avis("[pont] gravats %d barridos a %s"
                         % (gravats, args.grava))

            minim, angle = minim_sector(mesures, n, args)

        darrera = NO_VIST if minim is None else minim
        barridos += 1
        if minim is None:
            buits += 1

        # Mode calibratge: un cop per segon, la distancia I L'ANGLE del punt
        # mes proper dels 360 graus. Amb aixo les dues constants es mesuren
        # en minuts; sense aixo, nomes es poden provar a cegues.
        if args.calibra and (time.time() - darrer_calib) >= 1.0:
            darrer_calib = time.time()
            if angle is None:
                avis("[calibra] no veig res valid en 360 graus")
            else:
                avis("[calibra] mes proper: %.3f m a %.1f graus" %
                     (darrera, angle))

        client = envia(client, darrera)

        if args.estat > 0 and (time.time() - darrer_estat) >= args.estat:
            avis("[pont]%s %d barridos (%d sense res al sector) | ultima: "
                 "%.3f m | client: %s"
                 % (" FALS" if lidar is None else "", barridos, buits,
                    darrera, "si" if client is not None else "NO"))
            darrer_estat = time.time()
            barridos = 0
            buits = 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        avis("[pont] aturat amb Ctrl-C")
        sys.exit(0)
