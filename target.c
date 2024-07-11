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

#define MAX_LINE_LENGHT 256

bool sigint_rec = FALSE;
int ok = 0;

pid_t server_pid;

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
        if(watch_pid == server_pid){
            FILE* tarlog = fopen("files/targets.log", "a");
            if(ok == 0){
                ok++;
                fprintf(tarlog, "recieved 1\n");
                fflush(tarlog);
            }
            else if(ok > 0){
                ok = 0;
                fprintf(tarlog, "recieved 2\n");
                fflush(tarlog);
            }
            fclose(tarlog);
            fclose(routine);
        }
        else{
            fprintf(routine, "%s\n", "TARGET : started success");
            kill(watch_pid, SIGUSR1);
            fclose(routine);
        }
    }

    if(signo == SIGUSR2){
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "TARGET : program terminated by WATCHDOG");
        fclose(routine);
        exit(EXIT_FAILURE);
    }

    if(signo == SIGINT){
        printf("target terminating return 0");
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "TARGET : terminating");
        fclose(routine);
        sigint_rec = TRUE;
    }

    if(signo == 34){
        
        FILE* tarlog = fopen("files/targets.log", "a");
        int fd;
        const char* logfile = "files/pidlog.log";
        const char* search = "server_pid:%d";
        char pidline[MAX_LINE_LENGHT];
        fd = open(logfile, O_RDONLY);
        if(fd == -1){
            perror("fp opening");
            fprintf(tarlog, "problems in fd");
            exit(EXIT_FAILURE);
        }

        int lock_file = flock(fd, LOCK_SH);
        if(lock_file == -1){
            perror("failed to lock the file pid");
            fprintf(tarlog, "problems in the lock");
            exit(EXIT_FAILURE);
        }

        FILE* f = fdopen(fd, "r");

        while(fgets(pidline, sizeof(pidline), f) != NULL){
            char label[MAX_LINE_LENGHT];
            int value;
            if(sscanf(pidline, "%[^:]:%d", label, &value) == 2){
                if(strcmp(label, "server_pid") == 0){
                    server_pid = value;
                    break;
                }
            }
            else{
                fprintf(tarlog, "problems in the pid acquisation");
            }
        }

        int unlock_file = flock(fd, LOCK_UN);
        if(unlock_file == -1){
            perror("failed to unlock the file pid");
        }
        fclose(f);
        close(fd);
        fprintf(tarlog, "server_pid: %d \n", server_pid);
        fclose(tarlog);
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
    FILE* tarlog = fopen("files/targets.log", "a");
    if(error == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(routine == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }

    if(tarlog == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    RegToLog(routine, "TARGET : start\n");

    /*int fd;
    const char* logfile = "files/pidlog.log";
    const char* search = "server_pid:%d";
    char pidline[MAX_LINE_LENGHT];
    fd = open(logfile, O_RDONLY);
    if(fd == -1){
        perror("fp opening");
        RegToLog(error, "TARGET: error in opening fd");
        exit(EXIT_FAILURE);
    }
    FILE* f = fdopen(fd, "r");
    int lock_file = flock(fileno(f), LOCK_EX);
    if(lock_file == -1){
        perror("failed to lock the file pid");
        RegToLog(error, "TARGET; error in lock the fail");
        exit(EXIT_FAILURE);
    }

    while(fgets(pidline, sizeof(pidline), f)){
        strcpy(pidline, search);
        if(strcmp(pidline, search) == 0){
            fscanf(f, "server_pid:%d", &server_pid);
            break;
        }
    }

    int unlock_file = flock(fileno(f), LOCK_UN);
    if(unlock_file == -1){
        perror("failed to unlock the file pid");
    }
    fclose(f);
    fprintf(tarlog, "server_pid: %d \n", server_pid);*/

    //semaphore 
    /*sem_t* sm_sem;
    sm_sem = sem_open("/sm_sem1", 0);
    if(sm_sem == SEM_FAILED){
        RegToLog(error, "DRONE : semaphore faild");
        perror("semaphore");
    }*/

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
    if(sigaction(SIGINT, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "TARGET : error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(34, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "TARGET : error in sigaction()");
        exit(EXIT_FAILURE);
    }

    fprintf(tarlog, "start acquiring from data.txt \n");

    int fp;
    const char* filename = "files/data.txt";
    char line[MAX_LINE_LENGHT];

    fp = open(filename, O_RDONLY);
    if(fp == -1){
        perror("fp opening");
        RegToLog(error, "TARGET: error in opening fp");
        exit(EXIT_FAILURE);
    }
    int lock_file = flock(fp, LOCK_SH);
    if(lock_file == -1){
        perror("failed to lock the file pid");
        RegToLog(error, "TARGET; error in lock the failmsmsms");
        exit(EXIT_FAILURE);
    }
    FILE* file = fdopen(fp, "r");
    
    while(fgets(line, sizeof(line), file) != NULL){
        char label[MAX_LINE_LENGHT];
        int value;
        if(sscanf(line, "%[^:]:%d", label, &value) == 2){
            if(strcmp(label, "N_TARGET") == 0){
                ncoord = value;
                break;
            }
        }
        else{
            fprintf(tarlog, "problems in the pid acquisation");
        }
    }

    int unlock_file = flock(fp, LOCK_UN);
    if(unlock_file == -1){
        perror("failed to unlock the file pid");
    }

    fclose(file);
    fprintf(tarlog, "n_tar: %d  \n", ncoord);
    //server_pid = atoi(argv[3]);

    //pipe to recieve the max_x and the max_y
    int readsd2;
    int varre;
    int maxx, maxy;
    readsd2 = atoi(argv[2]);
    fprintf(tarlog, "redsd2: %d", readsd2);
    fflush(tarlog);
    sleep(1);
    
    //sem_wait(sm_sem);
    varre = read(readsd2, &maxx, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "TARGET : error in readsd 1");
    }
    fprintf(tarlog, "maxx: %d , maxy: %d \n", maxx, maxy);
    fflush(tarlog);
    //sem_post(sm_sem);
    sleep(1);
    //sem_wait(sm_sem);
    varre = read(readsd2, &maxy, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "TARGET : error in readsd 2");
    }
    sleep(1);
    fprintf(tarlog, "maxx: %d , maxy: %d \n", maxx, maxy);
    fflush(tarlog);
    //sem_post(sm_sem);

    srand(time(NULL));  //initialise random seed

    int points[2][ncoord];
    for(int i = 0; i<2; i++){
        for(int j=0; j<ncoord; j++){
            if(i == 0){
                points[i][j] = rand() %maxx;  //random column
                points[i][j] = rand() %maxx;  //random column
            }
            else{
                points[i][j] = rand() %maxy;  //random random raw
                points[i][j] = rand() %maxy;  //random random raw
            }
        }
    }

    while(point_feseability(points, maxx, maxy)){
        for(int ii = 0; ii<2; ii++){
            for(int jj=0; jj<ncoord; jj++){
                if(ii == 0)
                    points[ii][jj] = rand() %maxx;  //random column
                else
                    points[ii][jj] = rand() %maxy;  //random random raw
            }
        }
    }

    for(int jjj = 0; jjj < ncoord; jjj++){
        fprintf(tarlog, "coord n %d: x(%d) y (%d)\n", jjj, points[0][jjj], points[1][jjj]);
        fflush(tarlog);
    }

    //pipe to share the position with the server
    int writesd2;
    writesd2 = atoi(argv[1]);
    fprintf(tarlog, "writesd2: %d\n", writesd2);
    fflush(tarlog);
    /*for(int i=0; i<ncoord; i++){
        write(writesd2, &points[0][i], sizeof(int));
        //fsync(writesd2);
        sleep(1);
    }
    for(int j =0; j<ncoord; j++){
        write(writesd2, &points[1][j], sizeof(int));
        //fsync(writesd2);
        sleep(1);
    }*/
    
    int i =0;
    while(!sigint_rec){
        
        i = 0;
        while(ok > 0){
            write(writesd2, &points[0][i], sizeof(int));
            fsync(writesd2);
            sleep(1);
            fprintf(tarlog, "sent %d x", i);
            fflush(tarlog);

            write(writesd2, &points[1][i], sizeof(int));
            fsync(writesd2);
            sleep(1);
            fprintf(tarlog, "sent %d y\n", i);
            fflush(tarlog);
            i++;
        }

        sleep(1);

        /*while(ok > 0 && ok <= ((ncoord*2)-2)){
            if(ok == check)
                sleep(1);
            else if(ok > check && ok <= ncoord){
                fprintf(tarlog, "coord x %d: %d \n", check, points[0][check]);
                write(writesd2, &points[0][check], sizeof(int));
                fsync(writesd2);
                check++;
                sleep(1);
            }
            else if(ok > check && ok > ncoord){
                fprintf(tarlog, "coord x %d: %d \n", i, points[0][i]);
                write(writesd2, &points[1][i], sizeof(int));
                fsync(writesd2);
                i++;
                check++;
                sleep(1);
            }
            else if(ok < check){
                RegToLog(error, "TARGET: error in counters");
                ok = -1;
            }
        }
        if(ok > ((ncoord*2)-2))
            kill(server_pid, SIGUSR1);
        
        ok = 0;
        sleep(1);*/
    }

    //sem_close(sm_sem);
    close(readsd2);
    close(writesd2);
    //fclose(fp);
    fclose(tarlog);
    fclose(routine);
    fclose(error);
    sleep(1);
    return 0;
}