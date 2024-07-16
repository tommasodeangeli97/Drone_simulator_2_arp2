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
            FILE* obstlog = fopen("files/obstacles.log", "a");
            if(ok == 0){
                ok++;
                fprintf(obstlog, "recieved 1\n");
                fflush(obstlog);
            }
            else if(ok > 0){
                ok = 0;
                fprintf(obstlog, "recieved 2\n");
                fflush(obstlog);
            }
            fclose(obstlog);
            fclose(routine);
        }
        else{
            fprintf(routine, "%s\n", "OBSTACLES : started success");
            kill(watch_pid, SIGUSR1);
            fclose(routine);
        }
    }

    if(signo == SIGUSR2){
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "OBSTACLES : program terminated by WATCHDOG");
        fclose(routine);
        exit(EXIT_FAILURE);
    }

    if(signo == SIGINT){
        printf("obstacles terminating return 0");
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "OBSTACLES : terminating");
        fclose(routine);
        sigint_rec = TRUE;
    }

    if(signo == 34){  //signal to collect the pids of the others processes
        
        FILE* obstlog = fopen("files/obstacles.log", "a");

        int fd;
        const char* logfile = "files/pidlog.log";
        //const char* search = "server_pid:%d";
        char pidline[MAX_LINE_LENGHT];
        fd = open(logfile, O_RDONLY);
        if(fd == -1){
            perror("fp opening");
            fprintf(obstlog, "error in fd");
            exit(EXIT_FAILURE);
        }

        int lock_file = flock(fd, LOCK_SH);
        if(lock_file == -1){
            perror("failed to lock the file pid");
            fprintf(obstlog, "error in lock");
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
                fprintf(obstlog, "problems in the pid acquisation");
            }
        }

        int unlock_file = flock(fd, LOCK_UN);
        if(unlock_file == -1){
            perror("failed to unlock the file pid");
        }
        fclose(f);
        close(fd);
        fprintf(obstlog, "server_pid: %d \n", server_pid);
        fclose(obstlog);
    }
}

int ncoord = 0;
//function to see if the created points are by chance equal
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
    FILE* obstlog = fopen("files/obstacles.log", "a");

    if(error == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(routine == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(obstlog == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    RegToLog(routine, "OBSTACLES : start\n");

    struct sigaction sa;  //initialize sigaction
    sa.sa_flags = SA_SIGINFO;  //use sigaction field instead of signalhandler
    sa.sa_sigaction = signalhandler;

    if(sigaction(SIGUSR1, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "OBSTACLES: error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(SIGUSR2, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "OBSTACLES : error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(SIGINT, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "OBSTACLES : error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(34, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "OBSTACLES : error in sigaction()");
        exit(EXIT_FAILURE);
    }

    //fprintf(obstlog, "start acquiring from data.txt \n");
    //fflush(obstlog);

    int fp;
    const char* filename = "files/data.txt";
    char line[MAX_LINE_LENGHT];

    //reading from the data.txt file the number of obstacles to create
    fp = open(filename, O_RDONLY);
    if(fp == -1){
        perror("fp opening");
        RegToLog(error, "OBSTACLES: error in opening fp");
        exit(EXIT_FAILURE);
    }
    int lock_file = flock(fp, LOCK_SH);
    if(lock_file == -1){
        perror("failed to lock the file pid");
        RegToLog(error, "OBSTACLES; error in lock the failmsmsms");
        exit(EXIT_FAILURE);
    }
    FILE* file = fdopen(fp, "r");
    
    while(fgets(line, sizeof(line), file) != NULL){
        char label[MAX_LINE_LENGHT];
        int value;
        if(sscanf(line, "%[^:]:%d", label, &value) == 2){
            if(strcmp(label, "N_OBSTACLES") == 0){
                ncoord = value;
                break;
            }
        }
        else{
            fprintf(obstlog, "problems in the pid acquisation");
        }
    }

    int unlock_file = flock(fp, LOCK_UN);
    if(unlock_file == -1){
        perror("failed to unlock the file pid");
    }

    fclose(file);
    //fprintf(obstlog, "n_obbst: %d  \n", ncoord);
    //fflush(obstlog);

    //pipe to recieve the max_x and the max_y
    int readsd1;
    int varre;
    int maxx, maxy;
    readsd1 = atoi(argv[2]);
    //fprintf(obstlog, "readsd1: %d  \n", readsd1);
    //fflush(obstlog);

    varre = read(readsd1, &maxx, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "OBSTACLES : error in readsd 1");
    }
    //fprintf(obstlog, "maxx: %d , maxy: %d \n", maxx, maxy);
    //fflush(obstlog);
    
    sleep(1);
    
    varre = read(readsd1, &maxy, sizeof(int));
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "OBSTACLES : error in readsd 2");
    }
    sleep(1);
    //fprintf(obstlog, "maxx: %d , maxy: %d \n", maxx, maxy);
    //fflush(obstlog);

    srand(time(NULL));  //initialise random seed
    int points[2][ncoord];
    //creating the points
    for(int i = 0; i<2; i++){
        for(int j=0; j<ncoord; j++){
            if(i == 0)
                points[i][j] = rand() %maxx;  //random column
            else
                points[i][j] = rand() %maxy;  //random random raw
        }
    }

    //checking if the points are feasibles
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

    /*for(int jjj = 0; jjj < ncoord; jjj++){
        fprintf(obstlog, "coord n %d: x(%d) y (%d)\n", jjj, points[0][jjj], points[1][jjj]);
        fflush(obstlog);
    }*/

    //pipe to share the position with the server
    int writesd1;
    writesd1 = atoi(argv[1]);
    //fprintf(obstlog, "writesd1: %d \n", writesd1);
    //fflush(obstlog);

    int i =0;

    //sending the coordinates untill the server gives back the stop signal
    while(!sigint_rec){
        i = 0;
        while(ok > 0){
            write(writesd1, &points[0][i], sizeof(int));
            fsync(writesd1);
            sleep(1);
            //fprintf(obstlog, "sent %d x", i);
            //fflush(obstlog);

            write(writesd1, &points[1][i], sizeof(int));
            fsync(writesd1);
            sleep(1);
            //fprintf(obstlog, "sent %d y\n", i);
            //fflush(obstlog);
            i++;
        }

        sleep(1);

    }

    //close the pipes
    close(writesd1);
    close(readsd1);
    //close the files
    fclose(obstlog);
    fclose(routine);
    fclose(error);
    sleep(1);
    return 0;
}