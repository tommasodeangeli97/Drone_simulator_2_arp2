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

bool sigint_rec = FALSE;

int max(int a, int b){
    if(a>b)
        return a;
    else
        return b;
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

pid_t watch_pid = -1;  //declaration pid of the watchdog
void signalhandler(int signo, siginfo_t* info, void* contex){
    if(signo == SIGUSR1){  //SIGUSR1 
        FILE* routine = fopen("files/routine.log", "a");
        watch_pid = info->si_pid;  //initialisation watchdog's pid
        fprintf(routine, "%s\n", "SERVER : started success");
        kill(watch_pid, SIGUSR1);
        fclose(routine);
    }

    if(signo == SIGUSR2){
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "SERVER : program terminated by WATCHDOG");
        fclose(routine);
        exit(EXIT_SUCCESS);
    }
    if(signo == SIGINT){
        printf("server terminating return 0");
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "SERVER : terminating");
        fclose(routine);
        sigint_rec = TRUE;
    }
}

int check_ostar(int ox, int oy, int tx, int ty){
    if(ox == tx && oy == ty){
        FILE* ferr = fopen("files/error.log", "a");
        fprintf(ferr, "%s\n", "SERVER: coincident points obstacles-target");
        fclose(ferr);
        return 1;
    }
    else
        return 0;
}

int near(int cx1, int cy1, int cx2, int cy2){
    if((cx2>=cx1-2 || cx2<=cx1+2) && cy1 == cy2)
        return 1;
    else
        return 0;
}

int main(int argc, char* argv[]){
    //initialization of ncurses
    initscr();

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
    RegToLog(routine, "SERVER : start\n");

    if(has_colors()){
        start_color();  //enables the color
        init_pair(1, COLOR_BLUE, COLOR_WHITE);  //define the window color
        init_pair(2, COLOR_RED, COLOR_WHITE); //define the obstacle color
        init_pair(3, COLOR_GREEN, COLOR_WHITE); //define the target color
    }

    //semaphore opening
    sem_t * sm_sem;
    sm_sem = sem_open("/sm_sem1", O_CREAT | O_RDWR, 0666, 1);
    if(sm_sem == SEM_FAILED){
        RegToLog(error, "SERVER : semaphore faild");
        perror("semaphore");
    }

    struct sigaction sa;  //initialize sigaction
    sa.sa_flags = SA_SIGINFO;  //use sigaction field instead of signalhandler
    sa.sa_sigaction = signalhandler;

    if(sigaction(SIGUSR1, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "SERVER: error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(SIGUSR2, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "SERVER : error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(SIGINT, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "SERVER : error in sigaction()");
        exit(EXIT_FAILURE);
    }

    //pipe to give to the drone, the obstacles and the target the max x and y
    int writesd, writesd4;

    sscanf(argv[1], "%d", &writesd);
    sscanf(argv[5], "%d", &writesd4);

    int max_x, max_y;
    getmaxyx(stdscr, max_y, max_x);  //takes the max number of rows and colons

    write(writesd, &max_x, sizeof(int));
    fsync(writesd);
    sleep(1);
    write(writesd, &max_y, sizeof(int));
    fsync(writesd);
    sleep(1);
    write(writesd4, &max_x, sizeof(int));
    fsync(writesd4);
    sleep(1);
    write(writesd4, &max_y, sizeof(int));
    fsync(writesd4);

    WINDOW *win = newwin(max_y, max_x, 0, 0);  //creats the window
    box(win, 0, 0);  //adds the borders
    wbkgd(win, COLOR_PAIR(1));  //sets the color of the window
    wrefresh(win);  //prints and refreshes the window

    srand(time(NULL));  //initialise random seed

    int droneposx, droneposy;
    int droneforx, dronefory;

    droneposx = rand() % max_x;  //random column
    droneposy = rand() % max_y;  //random row
    if(droneposx <= 1)
        droneposx = 2;
    if(droneposx >= max_x-3)
        droneposx = max_x-4;
    if(droneposy <= 1)
        droneposy = 2;
    if(droneposy >= max_y)
        droneposy = max_y-1;

    //update the drone position
    write(writesd, &droneposx, sizeof(int));
    fsync(writesd);
    sleep(1);
    write(writesd, &droneposy, sizeof(int));
    fsync(writesd);

    int readsd, readsd3;
    int varre;
    sscanf(argv[2], "%d", &readsd);
    sscanf(argv[6], "%d", &readsd3);

    //takes the number of target and obstacles from the file
    FILE* fp;
    const char* filename = "data.txt";
    const char* target_name = "N_OBSTACLES:";
    const char* target_name2 = "N_TARGET:";
    int n_obst, n_tar, at = 0;
    char line[MAX_LINE_LENGHT];

    fp = fopen(filename, "r");
    if(fp == NULL){
        perror("fp opening");
        RegToLog(error, "OBSTACLES: error in opening fp");
        exit(EXIT_FAILURE);
    }

    while(fgets(line, sizeof(line), fp)){
        line[strcspn(line, "\n")] = '\0';
        if(strcmp(line, target_name) == 0){
            sscanf(line+strlen(target_name), "%d", &n_obst);
            if(at>0)
                break;
            at++;
        }
        if(strcmp(line, target_name2) == 0){
            sscanf(line+strlen(target_name2), "%d", &n_tar);
            if(at>0)
                break;
            at++;
        }
    }

    //store the obstacles and target position
    int obst[2][n_obst];
    int target[2][n_tar];
    
    int readsd1, readsd2;
    sscanf(argv[3], "%d", &readsd1);
    sscanf(argv[4], "%d", &readsd2);

    for(int i=0; i<(n_obst*2)-2; i++){
        if(i<n_obst){
            sem_wait(sm_sem);
            varre = read(readsd1, &obst[0][i], sizeof(int));
            if( varre == -1){
                perror("readsd1");
                RegToLog(error, "SERVER : error in readsd1 obst");
            }
            sem_post(sm_sem);
        }
        else{
            sem_wait(sm_sem);
            varre = read(readsd1, &obst[1][i], sizeof(int));
            if( varre == -1){
                perror("readsd1");
                RegToLog(error, "SERVER : error in readsd1 obst");
            }
            sem_post(sm_sem);
        }
    }
    for(int j=0; j<(n_tar*2)-2; j++){
        if(j<n_tar){
            sem_wait(sm_sem);
            varre = read(readsd2, &target[0][j], sizeof(int));
            if( varre == -1){
                perror("readsd2");
                RegToLog(error, "SERVER : error in readsd2 tar");
            }
            sem_post(sm_sem);
        }
        else{
            sem_wait(sm_sem);
            varre = read(readsd2, &target[1][j], sizeof(int));
            if( varre == -1){
                perror("readsd2");
                RegToLog(error, "SERVER : error in readsd2 tar");
            }
            sem_post(sm_sem);
        }
    }

    //check if the pionts are sovrapposed and print the obstacles and target
    int n = max(n_obst, n_tar);
    int arr_c[2][n];
    for(int k=0; k<n; k++){
        arr_c[0][k] = -1;
        arr_c[1][k] = -1;
    }
    int v =0;
    for(int h=0; h<n_obst; h++){
        for(int g=0; g<n_tar; g++){
            if(check_ostar(obst[0][h], obst[1][h], target[0][g], target[1][g])){
                arr_c[0][v] = h;
                arr_c[1][v] = g;
                v++;
            }
            else
                v++;
        }
    }
    for(int z=0; z<n; z++){
        if(arr_c[0][z] == -1){
            if(z<n_obst){
                wattr_on(win, COLOR_PAIR(2), NULL);
                mvwprintw(win, obst[1][z], obst[0][z], "X");  //prints the obstacles
                wattr_off(win, COLOR_PAIR(2), NULL);
                wrefresh(win);
            }
            
        }
        if(arr_c[1][z] == -1){
            if(z<n_tar){
                wattr_on(win, COLOR_PAIR(3), NULL);
                mvwprintw(win, target[1][z], target[0][z], "O");  //prints the targets
                wattr_off(win, COLOR_PAIR(3), NULL);
                wrefresh(win);
            }
            
        }
    }
    
    int r = 0, score = 0;
    int memo[2][1];
    while(!sigint_rec){  //print the drone in the new position given by the shared memory and delet the old one
        if(r >0){
            wattr_on(win, COLOR_PAIR(1), NULL);
            mvwprintw(win, droneposy, droneposx, "   ");  //prints the drone
            wattr_off(win, COLOR_PAIR(1), NULL);
            wrefresh(win);
            
            r--;
        }

        //acquire position and forces
        sem_wait(sm_sem);
        varre = read(readsd3, &droneforx, sizeof(int));
        if( varre == -1){
            perror("readsd");
            RegToLog(error, "SERVER : error in readsd3 3");
        }
        sem_post(sm_sem);
        sleep(1);
        sem_wait(sm_sem);
        varre = read(readsd3, &dronefory, sizeof(int));
        if( varre == -1){
            perror("readsd");
            RegToLog(error, "SERVER : error in readsd3 4");
        }
        sem_post(sm_sem);
        sem_wait(sm_sem);
        varre = read(readsd, &droneposx, sizeof(int));
        if( varre == -1){
            perror("readsd");
            RegToLog(error, "SERVER : error in readsd 1");
        }
        sem_post(sm_sem);
        sleep(1);
        sem_wait(sm_sem);
        varre = read(readsd, &droneposy, sizeof(int));
        if( varre == -1){
            perror("readsd");
            RegToLog(error, "SERVER : error in readsd 2");
        }
        sem_post(sm_sem);

        wattr_on(win, COLOR_PAIR(1), NULL);
        mvwprintw(win, max_y, 0, "force x:%d, force y:%d, score:%d", droneforx, dronefory, score);
        mvwprintw(win, droneposy, droneposx, "}o{");  //prints the drone
        wattr_off(win, COLOR_PAIR(1), NULL);
        wrefresh(win);
        
        //check if the drone pursuit a gol or touch a obstacles
        for(int c =0; c<n; c++){
            if(near(droneposx, droneposy, obst[0][c], obst[1][c]) && c<n_obst){
                wattr_on(win, COLOR_PAIR(2), NULL);
                mvwprintw(win, obst[1][c], obst[0][c]-2, "CRASH");  //prints the obstacles
                wattr_off(win, COLOR_PAIR(2), NULL);
                wrefresh(win);
                score--;
                sleep(2);
                wattr_on(win, COLOR_PAIR(2), NULL);
                mvwprintw(win, obst[1][c], obst[0][c], "  ");  //prints the obstacles
                wattr_off(win, COLOR_PAIR(2), NULL);
                wrefresh(win);
                memo[0][0] = c;
            }
            if(near(droneposx, droneposy, target[0][c], target[1][c]) && c<n_tar){
                wattr_on(win, COLOR_PAIR(2), NULL);
                mvwprintw(win, target[1][c], target[0][c], "V");  //prints the obstacles
                wattr_off(win, COLOR_PAIR(2), NULL);
                wrefresh(win);
                score++;
                sleep(2);
                wattr_on(win, COLOR_PAIR(2), NULL);
                mvwprintw(win, target[1][c], target[0][c], "  ");  //prints the obstacles
                wattr_off(win, COLOR_PAIR(2), NULL);
                wrefresh(win);
                memo[1][0] = c;
            }
            else{
                memo[0][0] = -1;
                memo[1][0] = -1;
            }    
        }
        //wipe out the obstacle/target already taken
        if(memo[0][0] >= 0 || memo[1][0] >= 0){
            if(memo[0][0] >= 0){
                RegToLog(routine, "SERVER: obstacle taken");
                obst[0][memo[0][0]] = -1;
                obst[1][memo[0][0]] = -1;
            }

            if(memo[1][0] >= 0){
                RegToLog(routine, "SERVER: target taken");
                target[0][memo[1][0]] = -1;
                target[1][memo[1][0]] = -1;
            }

            memo[0][0] = -1;
            memo[1][0] = -1;     
        }

        sleep(1);
        r++;
    }

    sem_close(sm_sem);
    fclose(error);
    fclose(routine);

    return 0;
}