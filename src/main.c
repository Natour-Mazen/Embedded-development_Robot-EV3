/*
 * main.h
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
volatile MDD_generic MDD_position;
volatile MDD_int MDD_status;
volatile bal_t bal;

typedef struct s_position {
    double x;
    double y;
    int a;
} position, *pPosition;

typedef struct s_target_position {
    double x;
    double y;
} target_position, *pTarget_position;

/**
 * Initialize the communication and shared data
 */
void init_comms() {
    MDD_quit = MDD_int_init(0);
    MDD_power = MDD_int_init(0);
    MDD_auto_command = MDD_int_init(0);
    MDD_direct_command = MDD_int_init(0);
    MDD_target = MDD_generic_init(sizeof(struct s_target_position));
    MDD_reset = MDD_generic_init(sizeof(struct s_position));
    MDD_position = MDD_generic_init(sizeof(struct s_position));
    MDD_status = MDD_int_init(0);
    bal = bal_create();
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
void* sendThread(void* outStream) {
    int x, y, a;
    int status;
    struct timespec horloge;
    clock_gettime(CLOCK_REALTIME, &horloge);
    while (!MDD_int_read(MDD_quit)) {
        status = MDD_int_read(MDD_status);
        position *pos = (position *)MDD_generic_read(MDD_position);
        if(pos != NULL) {
            x = (int) pos->x;
            y = (int) pos->y;
            a = pos->a;
            fprintf((FILE*)outStream, "s %d\n", status);
            fprintf((FILE*)outStream, "p %d %d %d\n", x, y, a);
            fflush((FILE*)outStream);
            free(pos);
        }
        fflush((FILE*)outStream);
        add_ms(&horloge, 200);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &horloge, 0);
    }
    fclose((FILE*)outStream);
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
void* directThread(void* arg)
{
    printf("Direct thread started\n");
    int command = 1;

    while(!MDD_int_read(MDD_quit))
    {
        printf("Waiting for direct command\n");
        command = bal_get(bal);
        printf("Direct command received: %d\n", command);

        // Tell to the sendTread the status.
        if(command != CMD_STOP){
            MDD_int_write(MDD_status, STATUS_DIRECT_MOVE);
        }
        else{
            MDD_int_write(MDD_status, STATUS_STANDBY);
        }

        int power = MDD_int_read(MDD_power);
        switch (command)
        {
            case CMD_STOP:
                set_tacho_command_inx(MY_LEFT_TACHO,TACHO_STOP);
                set_tacho_command_inx(MY_RIGHT_TACHO,TACHO_STOP);
                break;
            case CMD_FORWARD:
                set_tacho_command_inx(MY_LEFT_TACHO,TACHO_RUN_DIRECT);
                set_tacho_command_inx(MY_RIGHT_TACHO,TACHO_RUN_DIRECT);
                set_tacho_duty_cycle_sp(MY_LEFT_TACHO, -power);
                set_tacho_duty_cycle_sp(MY_RIGHT_TACHO, -power);
                break;
            case CMD_BACKWARD:
                set_tacho_command_inx(MY_LEFT_TACHO,TACHO_RUN_DIRECT);
                set_tacho_command_inx(MY_RIGHT_TACHO,TACHO_RUN_DIRECT);
                set_tacho_duty_cycle_sp(MY_LEFT_TACHO, power);
                set_tacho_duty_cycle_sp(MY_RIGHT_TACHO, power);
                break;
            case CMD_LEFT:
                set_tacho_command_inx(MY_LEFT_TACHO,TACHO_RUN_DIRECT);
                set_tacho_command_inx(MY_RIGHT_TACHO,TACHO_RUN_DIRECT);
                set_tacho_duty_cycle_sp(MY_LEFT_TACHO,-power);
                set_tacho_duty_cycle_sp(MY_RIGHT_TACHO,power);
                break;
            case CMD_RIGHT:
                set_tacho_command_inx(MY_LEFT_TACHO,TACHO_RUN_DIRECT);
                set_tacho_command_inx(MY_RIGHT_TACHO,TACHO_RUN_DIRECT);
                set_tacho_duty_cycle_sp(MY_LEFT_TACHO,power);
                set_tacho_duty_cycle_sp(MY_RIGHT_TACHO,-power);
                break;
            default:
                printf("Unknown command in directThread\n");
                break;
        }
        // 10 ms.
        usleep(10000); // Small delay to avoid busy-waiting
    }
    return arg;
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
void* deadreckoningThread(void* arg) {
    double x = 0.0, y = 0.0;
    double angle = 0.0;
    position reset = {x, y, angle};

    deadRWorkerInit();
    while (!MDD_int_read(MDD_quit)) {
        // If the 'reset' value is new, then we change compute a new value.
        if (MDD_generic_read2(MDD_reset, &reset)) {
            x = reset.x;
            y = reset.y;
            angle = reset.a * M_PI / 180.0; // Convert to radians
        }

        deadRWorker(x, y, angle, &x, &y, &angle);

        position pos = { (int)x, (int)y, (int)(angle * 180.0 / M_PI) };
        MDD_generic_write(MDD_position, &pos);
        // 30 ms.
        usleep(30000);
    }
    return arg;
}

/**
 * auto move thread
 * when a target is defined, sets its mode to running (and updates the MDD status),
 * until STOP is received, or quit is received, or the target is reached with an acceptable error.
 */
void* autoThread(void* arg) {
    int x = 0, y = 0;
    target_position target = {x, y};
    position pos = {0,0,0};
    while (!MDD_int_read(MDD_quit)) {

        // Tell to the sendTread the status.
        MDD_int_write(MDD_status, STATUS_AUTO_MOVE);

        if (MDD_generic_read2(MDD_target, &target)) {
            int targetReached = 0;
            // For loop detection.
            int whileError = 0;
            float lastX = 0, lastY = 0;
            int lastA = 0;

            int power = MDD_int_read(MDD_power);
            while (!targetReached && MDD_int_read(MDD_auto_command) && !MDD_int_read(MDD_quit) && power > 0) {
                MDD_generic_read2(MDD_position, &pos);

                double alpha = (pos.a) * M_PI / 180.0;
                double error = deadreckoningGoTo(pos.x, pos.y, alpha, target.x, target.y, power);
                printf("P %.1f %.1f %d, E %.0f, T %.1f %.1f\n", pos.x, pos.y, pos.a, error, target.x, target.y);
                // To prevent loop.
                if(lastX == pos.x && lastY == pos.y && lastA == pos.a){
                    whileError++;
                }
                else{
                    lastX = pos.x;
                    lastY = pos.y;
                    lastA = pos.a;
                    whileError = 0;
                }
                // If a loop is detected, we break.
                if(whileError >= 500){
                    break;
                }
                // Target reached.
                if(error <= 0){
                    targetReached = 1;
                }
                usleep(10000);
            }
            set_tacho_command_inx(MY_LEFT_TACHO,TACHO_STOP);
            set_tacho_command_inx(MY_RIGHT_TACHO,TACHO_STOP);
        }
        // 50 ms.
        usleep(50000);
    }
    return arg;
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
int main() {
    char cmd;
    char buf[256];
    int mode = MODE_DIRECT;
    FILE *inStream, *outStream;

    ev3_init();
    ev3_port_init();
    ev3_tacho_init();
    ev3_sensor_init();
    if (my_init_ev3()) {
        return 1;
    }
    init_comms();
    printf("Ready and waiting for incoming connection...\n");
    if (WaitClient(&outStream, &inStream)) {
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
                case 'q': // quit
                    printf("Received command: quit\n");
                    MDD_int_write(MDD_quit, 1);
                    quit = 1;
                    break;
                case 'p': { // current power to be set to pow
                    int power;
                    if (sscanf(buf + 1, "%d", &power) == 1) {
                        if(power < 20 && power > 0){
                            power = 20;
                        }
                        MDD_int_write(MDD_power, power);
                        printf("Received command: set power to %d\n", power);
                    } else {
                        printf("Invalid power command: %s\n", buf);
                    }
                    break;
                }
                case 'r': { // current position and heading should be reset to (x,y,alpha)
                    int x, y, a;
                    if (sscanf(buf + 1, "%d %d %d", &x, &y, &a) == 3) {
                        position reset;
                        reset.x = x;
                        reset.y = y;
                        reset.a = a;
                        MDD_generic_write(MDD_reset, &reset);
                        printf("Received command: reset to (%d, %d, %d)\n", x, y, a);
                        printf("Received command: reset to (%d, %d, %d)\n", x, y, a);
                    } else {
                        printf("Invalid reset command: %s\n", buf);
                    }
                    break;
                }
                case 'g': { // auto goto (x,y)
                    int x, y;
                    if (sscanf(buf + 1, "%d %d", &x, &y) == 2) {
                        target_position target;
                        target.x = x;
                        target.y = y;
                        MDD_generic_write(MDD_target, &target);
                        MDD_int_write(MDD_auto_command, MODE_AUTO);
                        printf("Received command: target set to (%d, %d)\n", x, y);
                    } else {
                        printf("Invalid target command: %s\n", buf);
                    }
                    break;
                }
                case 'm': { // set the mode to MODE_DIRECT (0) or MODE_AUTO (1)
                    int newMode;
                    if (sscanf(buf + 1, "%d", &newMode) == 1) {
                        mode = newMode;
                        if(mode == MODE_DIRECT)
                        {
                            MDD_int_write(MDD_auto_command, CMD_STOP);
                        }
                        else if(mode == MODE_AUTO){
                            MDD_int_write(MDD_direct_command, CMD_STOP);
                            MDD_int_write(MDD_auto_command, MODE_AUTO);
                        }

                        printf("Received command: mode set to %d\n", mode);
                    } else {
                        printf("Invalid mode command: %s\n", buf);
                    }
                    break;
                }
                case 'F': // forward
                {
                    MDD_int_write(MDD_direct_command, CMD_FORWARD);
                    bal_put(bal, CMD_FORWARD);
                    break;
                }
                case 'B': // backward
                {
                    MDD_int_write(MDD_direct_command, CMD_BACKWARD);
                    bal_put(bal, CMD_BACKWARD);
                    break;
                }
                case 'L': // left
                {
                    MDD_int_write(MDD_direct_command, CMD_LEFT);
                    bal_put(bal, CMD_LEFT);
                    break;
                }
                case 'R': // right
                {
                    MDD_int_write(MDD_direct_command, CMD_RIGHT);
                    bal_put(bal, CMD_RIGHT);
                    break;
                }
                case 'S': // stop
                {
                    if (mode == MODE_DIRECT) {
                        MDD_int_write(MDD_direct_command, CMD_STOP);
                        bal_put(bal, CMD_STOP);
                    } else if (mode == MODE_AUTO) {
                        MDD_int_write(MDD_auto_command, CMD_STOP);
                    }
                    MDD_int_write(MDD_status, STATUS_STANDBY);
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
    // To stop direct.
    bal_put(bal, CMD_STOP);

    // Join the threads.
    pthread_join(sendThreadHandle, NULL);
    pthread_join(directThreadHandle, NULL);
    pthread_join(deadreckThreadHandle, NULL);
    pthread_join(autoThreadHandle, NULL);

    fclose(inStream);
    set_tacho_command_inx(MY_RIGHT_TACHO, TACHO_STOP);
    set_tacho_command_inx(MY_LEFT_TACHO, TACHO_STOP);
    ev3_uninit();
    printf("Shutdown complete.\n");
    CloseSockets();
    return 0;
}


