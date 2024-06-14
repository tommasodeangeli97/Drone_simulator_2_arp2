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
#include <sys/ipc.h>
#include <semaphore.h>

#define MASS 1
#define T_STEP 0.1
#define VISCOSITY 0.1
#define FORCE 10

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

pid_t watch_pid = -1;  //declaration pid of the watchdog

void signalhandler(int signo, siginfo_t* info, void* contex){
    if(signo == SIGUSR1){  //SIGUSR1 
        FILE* routine = fopen("files/routine.log", "a");
        watch_pid = info->si_pid;  //initialisation watchdog's pid
        fprintf(routine, "%s\n", "DRONE : started success");
        kill(watch_pid, SIGUSR1);
        fclose(routine);
    }

    if(signo == SIGUSR2){
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "DRONE : program terminated by WATCHDOG");
        fclose(routine);
        exit(EXIT_FAILURE);
    }
}

double acceleration(int force){  //function to find the acceleration
    return force / MASS;
}

double velocity(double acc, double vel){
    return vel + acc * T_STEP - (vel * VISCOSITY / MASS) * T_STEP;  //the velocity is given by the inizitial velocity + acceleration*time - the viscosity*velocity
}

int position(int pos, double vel){
    return (int)round(pos + vel * T_STEP);  //position is given by actual position + velocity*time
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
    RegToLog(routine, "DRONE : start\n");

    //semaphore 
    sem_t* sm_sem;
    sm_sem = sem_open("/sm_sem1", 0);
    if(sm_sem == SEM_FAILED){
        RegToLog(error, "DRONE : semaphore faild");
        perror("semaphore");
    }

    struct sigaction sa;  //initialize sigaction
    sa.sa_flags = SA_SIGINFO;  //use sigaction field instead of signalhandler
    sa.sa_sigaction = signalhandler;

    if(sigaction(SIGUSR1, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "DRONE: error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(SIGUSR2, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "DRONE : error in sigaction()");
        exit(EXIT_FAILURE);
    }

    //use of pipe to take from the server the max of x and y
    int readsd, readsd3;
    int varre;
    int maxx, maxy;
    int writesd;
    sscanf(argv[3], "%d", &writesd);
    sscanf(argv[1], "%d", &readsd);
    sscanf(argv[2], "%d", &readsd3);

    sem_wait(sm_sem);
    varre = read(readsd, &maxx, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "DRONE : error in readsd 1");
    }
    sem_post(sm_sem);
    sleep(1);
    sem_wait(sm_sem);
    varre = read(readsd, &maxy, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "DRONE : error in readsd 2");
    }
    sem_post(sm_sem);

    int x = 0;
    int y = 0;
    double accx, accy;
    double velx = 0.0;
    double vely = 0.0;
    int posx, posy;
    int forx, fory;

    sem_wait(sm_sem);
    varre = read(readsd, &x, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "DRONE : error in readsd 3");
    }
    sem_post(sm_sem);
    sleep(1);
    sem_wait(sm_sem);
    varre = read(readsd, &y, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "DRONE : error in readsd 4");
    }
    sem_post(sm_sem);

    while(1){  //takes from the keyboard the force acting on the drone and calculates the accelleration, the velocity and the position

        sem_wait(sm_sem);
        varre = read(readsd3, &forx, sizeof(int));
        if( varre == -1){
            perror("readsd");
            RegToLog(error, "DRONE : error in readsd3 1");
        }
        sem_post(sm_sem);
        sleep(1);
        sem_wait(sm_sem);
        varre = read(readsd3, &fory, sizeof(int));
        if( varre == -1){
            perror("readsd");
            RegToLog(error, "DRONE : error in readsd3 2");
        }
        sem_post(sm_sem);

        accx = acceleration(forx*FORCE);
        velx = velocity(accx, velx);
        posx = position(x, velx);
        //the drone can't go out of the screen
        if(posx >= maxx-3){
            x = maxx-4;
            RegToLog(routine, "DRONE : x lower limit");
        }
        else if(posx <= 1){
            x = 2;
            RegToLog(routine, "DRONE : x upper limit");
        }
        else{
            x = posx;
        }
        
        accy = acceleration(fory*FORCE);
        vely = velocity(accy, vely);
        posy = position(y, vely);
        //the drone can't go out of the screen
        if(posy >= maxy){
            y = maxy-2;
            RegToLog(routine, "DRONE : y lower limit");
        }
        else if(posy <= 1){
            y = 2;
            RegToLog(routine, "DRONE : y upper limit");
        }
        else{
            y = posy;
        }
        
        write(writesd, &x, sizeof(int));
        fsync(writesd);
        sleep(1);
        write(writesd, &y, sizeof(int));
        fsync(writesd);
               
        fprintf(routine, "%d %d\n", x, y);
        sleep(1);
    }

    sem_close(sm_sem);
    fclose(error);
    fclose(routine);

    return 0;
}