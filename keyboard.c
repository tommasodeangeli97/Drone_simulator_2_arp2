#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/shm.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include <termios.h>
#include <semaphore.h>

//definition of the special keys
#define UP 'e'
#define UP_L 'w'
#define UP_R 'r'
#define RIGHT 'f'
#define BRAKE 'd'
#define LEFT 's'
#define DOWN 'c'
#define DOWN_L 'x'
#define DOWN_R 'v'
#define QUIT 'q'

int exit_value = 0;

pid_t watch_pid = -1;  //declaration pid of the watchdog
void signalhandler(int signo, siginfo_t* info, void* contex){
    if(signo == SIGUSR1){  //SIGUSR1 
        FILE* routine = fopen("files/routine.log", "a");
        watch_pid = info->si_pid;  //initialisation watchdog's pid
        fprintf(routine, "%s\n", "KEYBOARD : command from WATCHDOG");
        kill(watch_pid, SIGUSR1);
        fclose(routine);
    }

    if(signo == SIGUSR2){
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "KEYBOARD : program terminated by WATCHDOG");
        fclose(routine);
        exit(EXIT_FAILURE);
    }
}

//function to write on the files
void RegToLog(FILE* fname, const char * message){
    time_t act_time;
    time(&act_time);
    int lock_file = flock(fileno(fname), LOCK_EX);
    if(lock_file == -1){
        perror("failed to lock the file");
        return;
    }
    fprintf(fname, "%s : ", ctime(&act_time));
    fprintf(fname, "%s\n", message);
    fflush(fname);

    int unlock_file = flock(fileno(fname), LOCK_UN);
    if(unlock_file == -1){
        perror("failed to unlock the file");
    }
}

int drone_x = 0, drone_y = 0;

void input_handler(char key, int f[]){  //function to handle the input recieved from the user and updates the shared memory
    FILE* routine = fopen("files/routine.log", "a");

    switch(key){
        case UP:
            drone_y -= 1;
            break;
        case UP_L:
            drone_y -= 1;
            drone_x -= 1;
            break;
        case UP_R:
            drone_y -= 1;
            drone_x += 1;
            break;
        case RIGHT:
            drone_x += 1;
            break;
        case LEFT:
            drone_x -= 1;
            break;
        case BRAKE:
            drone_x = 0;
            drone_y = 0;
            break;
        case DOWN:
            drone_y += 1;
            break;
        case DOWN_L:
            drone_y += 1;
            drone_x -= 1;
            break;
        case DOWN_R:
            drone_y += 1;
            drone_x += 1;
            break;
        case QUIT:
            printf("exiting the program");
            sleep(2);
            kill(watch_pid, SIGUSR2);
            exit_value = 1;
            break;
        default:
            break;
    }
    
    RegToLog(routine, "recieved key");
    f[0] = drone_x;
    f[1] = drone_y;

    fclose(routine);
}

char GetInput(){  //function to acquire the input from the user
    struct termios old, new;
    char button;

    if(tcgetattr(STDIN_FILENO, &old) == -1){
        perror("tcgetattr");
        exit(EXIT_FAILURE);
    }

    new = old;
    new.c_lflag &= ~(ICANON | ECHO);

    if(tcsetattr(STDIN_FILENO, TCSANOW, &new) == -1){
        perror("tcsetattr");
        exit(EXIT_FAILURE);
    }

    button = getchar();

    if(tcsetattr(STDIN_FILENO, TCSANOW, &old) == -1){
        perror("tcsetattr2");
        exit(EXIT_FAILURE);
    }

    return button;
}

int main(int argc, char* argv[]){ 

    FILE* routine = fopen("files/routine.log", "a");
    FILE* error = fopen("files/error.log", "a");
    if(error == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(routine == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    RegToLog(routine, "KAYBOARD : start\n");

    struct sigaction sa;  //initialize sigaction
    sa.sa_flags = SA_SIGINFO;  //use sigaction field instead of signalhandler
    sa.sa_sigaction = signalhandler;

    if(sigaction(SIGUSR1, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "KEYBOARD : error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(SIGUSR2, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "KEYBOARD : error in sigaction()");
        exit(EXIT_FAILURE);
    }

    //pipe to give to the drone process the max x and y
    int writesd3;
    sscanf(argv[1], "%d", &writesd3);

    int f[2];

    while(!exit_value){
        char ch = GetInput();  //acquire continously the keypad
        input_handler(ch, f);
        write(writesd3, &f[0], sizeof(int));
        fsync(writesd3);
        sleep(1);
        write(writesd3, &f[1], sizeof(int));
        fsync(writesd3);
    }

    RegToLog(routine, "KEYBOARD : terminated by input");

    fclose(error);
    fclose(routine);
    return 0;
}
