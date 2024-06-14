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
#include <ncurses.h>
#include <semaphore.h>

#define MAX_LINE_LENGHT 100

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
        fprintf(routine, "%s\n", "TARGET : started success");
        kill(watch_pid, SIGUSR1);
        fclose(routine);
    }

    if(signo == SIGUSR2){
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "TARGET : program terminated by WATCHDOG");
        fclose(routine);
        exit(EXIT_FAILURE);
    }
}

int ncoord = 0;
int point_feseability( int coord[2][ncoord], int maxx, int maxy){
    int coordx[ncoord];
    int coordy[ncoord];
    for(int j=0; j<ncoord; j++){
        coordx[j] = coord[0][j];
        coordy[j] = coord[1][j];
    }

    int cc =0;
    for(int i=0; i<ncoord; i++){
        for(int g =0; g<ncoord; g++){
            if(i != g){
                if(coordx[i] == coordx[g] && coordy[i] == coordy[g])
                    cc++;
            }
            
        }
    }

    if(cc != 0)
        return 1;

    return 0;

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
    RegToLog(routine, "TARGET : start\n");

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
        RegToLog(error, "TARGET: error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(SIGUSR2, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "TARGET : error in sigaction()");
        exit(EXIT_FAILURE);
    }

    FILE* fp;
    const char* filename = "data.txt";
    const char* target_name = "N_TARGET:";
    char line[MAX_LINE_LENGHT];

    fp = fopen(filename, "r");
    if(fp == NULL){
        perror("fp opening");
        RegToLog(error, "TARGET: error in opening fp");
        exit(EXIT_FAILURE);
    }

    while(fgets(line, sizeof(line), fp)){
        line[strcspn(line, "\n")] = '\0';
        if(strcmp(line, target_name) == 0){
            sscanf(line+strlen(target_name), "%d", &ncoord);
            break;
        }
    }

    //pipe to recieve the max_x and the max_y
    int readsd4;
    int varre;
    int maxx, maxy;
    sscanf(argv[2], "%d", &readsd4);
    
    sem_wait(sm_sem);
    varre = read(readsd4, &maxx, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "TARGET : error in readsd 1");
    }
    sem_post(sm_sem);
    sleep(1);
    sem_wait(sm_sem);
    varre = read(readsd4, &maxy, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "TARGET : error in readsd 2");
    }
    sem_post(sm_sem);


    //prendere il numero di ostacoli dal file


    srand(time(NULL));  //initialise random seed
    int points[2][ncoord];
    for(int i = 0; i<2; i++){
        for(int j=0; j<ncoord; j++){
            if(i=0)
                points[i][j] = rand() %maxx;  //random column
            else
                points[i][j] = rand() %maxy;  //random random raw
        }
    }

    while(point_feseability(points, maxx, maxy)){
        for(int i = 0; i<2; i++){
            for(int j=0; j<ncoord; j++){
                if(i=0)
                    points[i][j] = rand() %maxx;  //random column
                else
                    points[i][j] = rand() %maxy;  //random random raw
            }
        }
    }

    //pipe to share the position with the server
    int writesd2;
    sscanf(argv[1], "%d", &writesd2);
    for(int i=0; i<ncoord; i++){
        write(writesd2, &points[0][i], sizeof(int));
        fsync(writesd2);
        sleep(1);
    }
    for(int j =0; j<ncoord; j++){
        write(writesd2, &points[1][j], sizeof(int));
        fsync(writesd2);
        sleep(1);
    }

    sem_close(sm_sem);
    fclose(fp);
    sleep(2);
    return 0;
}