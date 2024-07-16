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

#define MAX_LINE_LENGHT 100

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
pid_t drone_pid, server_pid;
int check = 0;

pid_t watch_pid = -1;  //declaration pid of the watchdog
void signalhandler(int signo, siginfo_t* info, void* contex){
    if(signo == SIGUSR1){  //SIGUSR1 
        FILE* routine = fopen("files/routine.log", "a");
        FILE* error = fopen("files/error.log", "a");
        watch_pid = info->si_pid;  //initialisation watchdog's pid
        if(watch_pid == -1){
            fprintf(error, "%s\n", "KEYBOARD : error in recieving pid");
            fclose(routine);
            fclose(error);
            perror("recieving pid key");
            exit(EXIT_FAILURE);
        }
        else if(watch_pid == server_pid){
            check++;
            fclose(routine);
            fclose(error);
        }
        else{
            fprintf(routine, "%s\n", "KEYBOARD : command from WATCHDOG");
            kill(watch_pid, SIGUSR1);
            fclose(routine);
            fclose(error);
        }
    }

    if(signo == SIGUSR2){
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "KEYBOARD : program terminated by WATCHDOG");
        fclose(routine);
        exit(EXIT_FAILURE);
    }

    if(signo == SIGINT){
        printf("drone terminating return 0");
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "DRONE : terminating");
        fclose(routine);
        exit_value = 1;
    }

    if(signo == 34){  //signal to collect the pids of the others processes

        FILE* keylog = fopen("files/keyboard.log", "a"); 

        int fd, b = 0;
        const char* logfile = "files/pidlog.log";
        const char* search = "server_pid:%d";
        const char* search2 = "drone_pid:%d";
        char pidline[MAX_LINE_LENGHT];
        fd = open(logfile, O_RDONLY);
        if(fd == -1){
            perror("fp opening");
            fprintf(keylog, "error in fd opening");
            exit(EXIT_FAILURE);
        }

        int lock_file = flock(fd, LOCK_SH);
        if(lock_file == -1){
            perror("failed to lock the file pid");
            fprintf(keylog, "error in lock");
            exit(EXIT_FAILURE);
        }

        FILE* fpid = fdopen(fd, "r");
    
        while(fgets(pidline, sizeof(pidline), fpid) != NULL){
            char label[MAX_LINE_LENGHT];
            int value;
            if(sscanf(pidline, "%[^:]:%d", label, &value) == 2){
                if(strcmp(label, "server_pid") == 0){
                    server_pid = value;
                    b++;
                }
                if(strcmp(label, "drone_pid") == 0){
                    drone_pid = value;
                    b++;
                }
                if(b>=2)
                    break;
            }
            else{
                fprintf(keylog, "problems in the pid acquisation");
            }
        }

        int unlock_file = flock(fd, LOCK_UN);
        if(unlock_file == -1){
            perror("failed to unlock the file pid");
        }
        fclose(fpid);
        close(fd);
        fprintf(keylog, "server_pid: %d , drone_pid: %d \n", server_pid, drone_pid);
        fclose(keylog);
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
int f[2];

void input_handler(char key){  //function to handle the input recieved from the user and updates the shared memory
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
    return;
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
    FILE* keylog = fopen("files/keyboard.log", "a");
    if(error == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(routine == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(keylog == NULL){
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
    if(sigaction(SIGINT, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "KEYBOARD : error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(34, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "KKEYYBOARD : error in sigaction()");
        exit(EXIT_FAILURE);
    }

    //pipe to give to the drone process the max x and y
    int writesd3, readsd3, readsd4;
    int varre;
    writesd3 = atoi(argv[1]);
    readsd3 = atoi(argv[2]);
    readsd4 = atoi(argv[3]);
    //fprintf(keylog, "writesd3: %d , readsd3: %d , readsd4: %d\n", writesd3, readsd3, readsd4);
    //fflush(keylog);

    while(!exit_value){
        //while loop in case the drone is near to obstacles, it collects the new forces calculated by the server as inputs and send those to the drone process
        while(check>0){  

            varre = -1;
            while(varre == -1){
                varre = read(readsd4, &drone_x, sizeof(int));
                sleep(1); 
            }
            //fprintf(keylog, "dronex: %d \n", drone_x);
            //fflush(keylog);

            varre = -1;
            while(varre == -1){
                varre = read(readsd4, &drone_y, sizeof(int));
                sleep(1);   
            }
            //fprintf(keylog, "droney: %d \n", drone_y);
            //fflush(keylog);

            f[0] = drone_x;
            f[1] = drone_y;

            //fprintf(keylog, "f_x: %d %d, f_y: %d %d\n", f[0], drone_x, f[1], drone_y);
            //fflush(keylog);
            write(writesd3, &drone_x, sizeof(int));
            fsync(writesd3);
            //fprintf(keylog, "sent \n");
            //fflush(keylog);
            sleep(1);

            write(writesd3, &drone_y, sizeof(int));
            fsync(writesd3);
            //fprintf(keylog, "sent 2\n");
            //fflush(keylog);
            sleep(1);
            
            check = 0;
            
        }

        char ch = GetInput();  //acquire continously the keypad
        input_handler(ch);
        
        //fprintf(keylog, "f_x: %d %d, f_y: %d %d\n", f[0],drone_x, f[1],drone_y);
        //fflush(keylog);
        
        write(writesd3, &drone_x, sizeof(int));
        fsync(writesd3);
        sleep(1);
        //fprintf(keylog, "sent \n");
        //fflush(keylog);
        
        write(writesd3, &drone_y, sizeof(int));
        fsync(writesd3);
        sleep(1);
        //fprintf(keylog, "sent 2\n");
        //fflush(keylog);
        
    }

    RegToLog(routine, "KEYBOARD : terminated by input");

    close(readsd3);
    close(writesd3);
    close(readsd4);
    fclose(error);
    fclose(routine);
    fclose(keylog);
    return 0;
}
