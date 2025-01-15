/*
 \file\t\tdeadreck.c
 \author\t${user}
 \date\t\t${date}
 \brief\t\tSimple Hello World! for the Ev3
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <ev3.h>
#include <ev3_tacho.h>
#include <ev3_port.h>
#include <ev3_sensor.h>
#include <pthread.h>
#include <time.h>
#include "print.h"
#include "time_util.h"
#include "mdd.h"
#include "communication.h"
#include "myev3.h"
#include "workers.h"
#include "bal.h"

// Shared data and mailboxes
volatile MDD_int MDD_quit;
volatile MDD_int MDD_power;
volatile MDD_int MDD_auto_command;
volatile MDD_int MDD_direct_command;
volatile MDD_generic MDD_target;
volatile MDD_generic MDD_reset;
volatile bal_t bal;

typedef struct s_reset_position {
    int x;
    int y;
    int a;
} *reset_position;

typedef struct s_target_position {
    int x;
    int y;
} *target_position;


/**
 * Initialize the communication and shared data
 */
void init_comms() {
    MDD_quit = MDD_int_init(0);
    MDD_power = MDD_int_init(0);
    MDD_auto_command = MDD_int_init(0);
    MDD_direct_command = MDD_int_init(0);
    MDD_target = MDD_generic_init(sizeof(struct s_target_position));
    MDD_reset = MDD_generic_init(sizeof(struct s_reset_position));
    bal = bal_create();
}

int get_robot_status() {
    // Cette fonction retourne un code de statut du robot
    // Exemple : 0 = Arrêté, 1 = En mouvement, 2 = En mode autonome, etc.
    // Vous pouvez utiliser une variable ou lire une information d'un capteur ou d'une structure existante.
    return 1; // Exemple : retourne "1" pour indiquer que le robot est en mouvement
}

void get_robot_position(int *x, int *y, double *angle) {
    // Cette fonction met à jour les coordonnées actuelles du robot et son orientation.
    // Les valeurs peuvent provenir de capteurs ou d'une estimation interne.

    // Exemple : simulation de positions basées sur une logique simple.
    static int simulated_x = 0;
    static int simulated_y = 0;
    static double simulated_angle = 0.0;

    simulated_x += 1;         // Incrémente la position en X pour la simulation
    simulated_y += 1;         // Incrémente la position en Y pour la simulation
    simulated_angle += 0.1;   // Fait tourner le robot légèrement

    if (simulated_angle >= 2 * M_PI) {
        simulated_angle -= 2 * M_PI; // Garde l'angle dans l'intervalle [0, 2*pi]
    }

    *x = simulated_x;
    *y = simulated_y;
    *angle = simulated_angle; // En radians
}

int autoMoveWorker(int x, int y) {
    // Cette fonction implémente le mouvement automatique vers une cible (x, y).
    // Elle retourne 1 si la cible est atteinte, 0 sinon.

    static int current_x = 0;
    static int current_y = 0;

    int threshold = 5; // Distance acceptable pour considérer que la cible est atteinte

    // Simulation de déplacement vers la cible
    if (current_x < x) {
        current_x++;
    } else if (current_x > x) {
        current_x--;
    }

    if (current_y < y) {
        current_y++;
    } else if (current_y > y) {
        current_y--;
    }

    printf("Current position: (%d, %d)\n", current_x, current_y);

    // Vérifie si la distance à la cible est inférieure au seuil
    if (abs(current_x - x) <= threshold && abs(current_y - y) <= threshold) {
        printf("Target reached at (%d, %d)\n", current_x, current_y);
        return 1; // Cible atteinte
    }

    return 0; // Cible non atteinte
}

/**
 * Thread sending information to the ground station
 * It should read the status, and, if modified, fprintf it on outStream
 * It should read the position and angle, and fprintf this information on outStream
 * Protocol format from robot to ground station:
 * Status: s status\n
 * Position: p x y a\n where x, y and a are integers!
 * Do not forget to fflush outStream at the end of each iteration, or data will be buffered but not sent
 * This thread should end when the application quits, and fclose the outStream socket
 */
void *sendThread(void *arg) {
    FILE *outStream = (FILE *)arg; // Reconversion du pointeur
    int x = 0, y = 0, a = 0;
    int status = 0;
    struct timespec horloge;
    clock_gettime(CLOCK_REALTIME, &horloge);

    while (!MDD_int_read(MDD_quit)) {
        status = get_robot_status();
        get_robot_position(&x, &y, &a);
        fprintf(outStream, "s %d\n", status);
        fprintf(outStream, "p %d %d %d\n", x, y, a);
        fflush(outStream);
        add_ms(&horloge, 200);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &horloge, 0);
    }
    fclose(outStream);
    return NULL;
}


/**
 * Thread commanding the robot in direct mode, it should wait everytime for a new command
 * As long as the application does not quit, this thread should apply the
 * received direct commands, using the current power, while updating the status
 * of the application (STATUS_STANDBY, STATUS_DIRECT_MOVE)
 * ev3dev-c functions that will be useful are:
 * set_tacho_command_inx (for commands TACHO_STOP and TACHO_RUN_DIRECT),
 * set_tacho_duty_cycle_sp in order to configure the power between -100 and 100
 */
void *directThread(void *dummy) {
    int command = 0;
    while (!MDD_int_read(MDD_quit)) {
        command = MDD_int_read(MDD_direct_command);
        switch (command) {
            case 0: // Stop
                set_tacho_command_inx(MY_RIGHT_TACHO, TACHO_STOP);
                set_tacho_command_inx(MY_LEFT_TACHO, TACHO_STOP);
                break;
            case 1: // Forward
                set_tacho_duty_cycle_sp(MY_RIGHT_TACHO, MDD_int_read(MDD_power));
                set_tacho_duty_cycle_sp(MY_LEFT_TACHO, MDD_int_read(MDD_power));
                set_tacho_command_inx(MY_RIGHT_TACHO, TACHO_RUN_DIRECT);
                set_tacho_command_inx(MY_LEFT_TACHO, TACHO_RUN_DIRECT);
                break;
            case 2: // Backward
                set_tacho_duty_cycle_sp(MY_RIGHT_TACHO, -MDD_int_read(MDD_power));
                set_tacho_duty_cycle_sp(MY_LEFT_TACHO, -MDD_int_read(MDD_power));
                set_tacho_command_inx(MY_RIGHT_TACHO, TACHO_RUN_DIRECT);
                set_tacho_command_inx(MY_LEFT_TACHO, TACHO_RUN_DIRECT);
                break;
            case 3: // Left
                set_tacho_duty_cycle_sp(MY_RIGHT_TACHO, MDD_int_read(MDD_power));
                set_tacho_duty_cycle_sp(MY_LEFT_TACHO, -MDD_int_read(MDD_power));
                set_tacho_command_inx(MY_RIGHT_TACHO, TACHO_RUN_DIRECT);
                set_tacho_command_inx(MY_LEFT_TACHO, TACHO_RUN_DIRECT);
                break;
            case 4: // Right
                set_tacho_duty_cycle_sp(MY_RIGHT_TACHO, -MDD_int_read(MDD_power));
                set_tacho_duty_cycle_sp(MY_LEFT_TACHO, MDD_int_read(MDD_power));
                set_tacho_command_inx(MY_RIGHT_TACHO, TACHO_RUN_DIRECT);
                set_tacho_command_inx(MY_LEFT_TACHO, TACHO_RUN_DIRECT);
                break;
        }
        usleep(100000); // Small delay to avoid busy-waiting
    }
    return NULL;
}

/**
 * deadrecknoning thread, should first initialize (see deadRWorkerInit)
 * and then periodically, until quit:
 * if reset has been required, reset the coords
 * call its worker (see deadRWorker)
 * and then update the shared variable of the coords
 * Note: careful, the deadRWorker assumes angles in radian, while
 * ground station assumes angles in degrees (reset, and data sent)
 */
void *deadreckoningThread(void *dummy) {
    double x = 0.0, y = 0.0; // Correction du type
    double angle = 0.0;
    struct s_reset_position *reset = NULL;

    deadRWorkerInit();
    while (!MDD_int_read(MDD_quit)) {
        if ((reset = (struct s_reset_position *)MDD_generic_read(MDD_reset)) != NULL) {
            x = (double)reset->x;
            y = (double)reset->y;
            angle = reset->a * M_PI / 180.0; // Convert to radians
            free(reset);
        }

        deadRWorker(x, y, angle, &x, &y, &angle); // Correction des types

        struct s_reset_position pos = { (int)x, (int)y, (int)(angle * 180.0 / M_PI) };
        MDD_generic_write(MDD_target, &pos);
        usleep(100000);
    }
    return NULL;
}


/**
 * auto move thread
 * when a target is defined, sets its mode to running (and updates the MDD status),
 * until STOP is received, or quit is received, or the target is reached with an acceptable error.
 */
void *autoThread(void *dummy) {
    struct s_target_position *target = NULL;
    int x = 0, y = 0;
    while (!MDD_int_read(MDD_quit)) {
        if ((target = (struct s_target_position *)MDD_generic_read(MDD_target)) != NULL) {
            int targetReached = 0;
            while (!targetReached && !MDD_int_read(MDD_quit)) {
                targetReached = autoMoveWorker(target->x, target->y);
                usleep(100000);
            }
            free(target);
        }
        usleep(100000);
    }
    return NULL;
}

/**
 * The main function will be used as one of the application thread: readGroundstationThread
 * It will be in charge of initializing the ev3: ev3_init ev3_port_init ev3_tacho_init ev3_sensor_init
 * then initializing our specific ev3 ports: my_init_ev3
 * Then it will initialize every shared data and mailbox,
 * Then it will wait for ground station connection:  WaitClient
 * Then it will create the threads
 * Then it will read the ground station orders using fscanf on the inStream
 * Protocol:
 * q\n -> quit
 * p pow\n -> current power to be set to pow
 * r x y alpha\n -> current position and heading should be reset to (x,y,alpha)
 * m 0|1\n -> set the mode to MODE_DIRECT (0) or MODE_AUTO (1)
 * S\n -> stop
 * F\n -> direct Forward
 * B\n -> direct Backward
 * L\n -> direct Left
 * R\n -> direct Right
 * g x y\n -> auto goto (x,y)
 * Note : when fscanf returns EOF, this means the ground station closed the connection, and
 * the application should stop.
 * Before exiting, we should send the quitting information to the other threads,
 * in order to cleanup (stop engines, close sockets, etc.)
 * Cleanup made in this thread, after making sure that every other thread is finished (pthread_join):
 * Every motor should be stopped, inStream should be closed
 */
int main(void) {
    char cmd;
    char buf[256];
    int mode = MODE_DIRECT;
    FILE *inStream, *outStream;

    ev3_init();
    ev3_port_init();
    ev3_tacho_init();
    ev3_sensor_init();
    if (my_init_ev3()) {
        displayError("my_init_ev3()");
        return 1;
    }
    init_comms();
    printf("Ready and waiting for incoming connection...\n");
    if (WaitClient(&outStream, &inStream)) {
        displayError("WaitClient(&outStream, &inStream)");
        return 1;
    }

    pthread_t sendThreadHandle, directThreadHandle, deadreckThreadHandle, autoThreadHandle;
    pthread_create(&sendThreadHandle, NULL, sendThread, outStream);
    pthread_create(&directThreadHandle, NULL, directThread, NULL);
    pthread_create(&deadreckThreadHandle, NULL, deadreckoningThread, NULL);
    pthread_create(&autoThreadHandle, NULL, autoThread, NULL);

    int quit = 0;
    while (!quit) {
        if (fgets(buf, 256, inStream)) {
            cmd = buf[0];
            switch (cmd) {
                case 'q':
                    printf("Received command: quit\n");
                    quit = 1;
                    break;
                case 'p': {
                    int power;
                    if (sscanf(buf + 1, "%d", &power) == 1) {
                        MDD_int_write(MDD_power, power);
                        printf("Received command: set power to %d\n", power);
                    } else {
                        printf("Invalid power command: %s\n", buf);
                    }
                    break;
                }
                case 'r': {
                    int x, y, a;
                    if (sscanf(buf + 1, "%d %d %d", &x, &y, &a) == 3) {
                        struct s_reset_position *reset = malloc(sizeof(struct s_reset_position));
                        reset->x = x;
                        reset->y = y;
                        reset->a = a;
                        MDD_generic_write(MDD_reset, reset);
                        printf("Received command: reset to (%d, %d, %d)\n", x, y, a);
                        printf("Received command: reset to (%d, %d, %d)\n", x, y, a);
                    } else {
                        printf("Invalid reset command: %s\n", buf);
                    }
                    break;
                }
                case 't': {
                    int x, y;
                    if (sscanf(buf + 1, "%d %d", &x, &y) == 2) {
                        struct s_target_position *target = malloc(sizeof(struct s_target_position));
                        target->x = x;
                        target->y = y;
                        MDD_generic_write(MDD_target, target);
                        printf("Received command: target set to (%d, %d)\n", x, y);
                    } else {
                        printf("Invalid target command: %s\n", buf);
                    }
                    break;
                }
                case 'm': {
                    int newMode;
                    if (sscanf(buf + 1, "%d", &newMode) == 1) {
                        mode = newMode;
                        printf("Received command: mode set to %d\n", mode);
                    } else {
                        printf("Invalid mode command: %s\n", buf);
                    }
                    break;
                }
                default:
                    printf("Unknown command: %c\n", cmd);
                break;
            }
        } else {
            printf("Connection closed by client.\n");
            quit = 1;
        }
    }

    printf("Shutting down...\n");
    MDD_int_write(MDD_quit, 1);

    pthread_join(sendThreadHandle, NULL);
    pthread_join(directThreadHandle, NULL);
    pthread_join(deadreckThreadHandle, NULL);
    pthread_join(autoThreadHandle, NULL);

    ev3_uninit();
    printf("Shutdown complete.\n");
    return 0;
}


