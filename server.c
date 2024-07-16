#include <ncurses.h>
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
#include <semaphore.h>

#define MAX_LINE_LENGHT 256
#define form '%'
#define MAXF 2

bool sigint_rec = 0;
pid_t key_pid, target_pid, obst_pid, drone_pid;

//quantities for the proximity and repulsive forces
float rho0 = 8; //repulsive force range
float rho2 = 2; //takes obstacles and target range
float eta = 40;

typedef struct{  //shared memory
    int forces[2];
    double vel[2];
    int score;
    int obst;
    int target;
} SharedMemory;

//function to take the max
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
        FILE* error = fopen("files/error.log", "a");
        watch_pid = info->si_pid;  //initialisation watchdog's pid
        if(watch_pid == -1){
            fprintf(error, "%s\n", "DRONE : error in recieving pid");
            fclose(routine);
            fclose(error);
            perror("recieving pid drone");
            exit(EXIT_FAILURE);
        }
        else{
            fprintf(routine, "%s\n", "SERVER : started success");
            kill(watch_pid, SIGUSR1);
            fclose(routine);
            fclose(error);
        }
    }

    if(signo == SIGUSR2){
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "SERVER : program terminated by WATCHDOG");
        fclose(routine);
        exit(EXIT_FAILURE);
    }
    if(signo == SIGINT){
        printf("server terminating return 0");
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "SERVER : terminating");
        fclose(routine);
        sigint_rec = 1;
    }

    if(signo == 34){  //signal to collect the pids of the others processes

        FILE* serverlog = fopen("files/server.log", "a");

        int fd;
        const char* logfile = "files/pidlog.log";
        char pidline[MAX_LINE_LENGHT];
        fd = open(logfile, O_RDONLY);
        if(fd == -1){
            perror("fp opening");
            fprintf(serverlog, "error in fd");
            exit(EXIT_FAILURE);
        }

        int lock_file = flock(fd, LOCK_SH);
        if(lock_file == -1){
            perror("failed to lock the file pid");
            fprintf(serverlog, "error in lock");
            exit(EXIT_FAILURE);
        }

        FILE* f = fdopen(fd, "r");
        
        int b = 0;
        while(fgets(pidline, sizeof(pidline), f) != NULL){
            char label[MAX_LINE_LENGHT];
            int value;
            if(sscanf(pidline, "%[^:]:%d", label, &value) == 2){
                if(strcmp(label, "keyboard_pid") == 0){
                    key_pid = value;
                    b++;
                }
                if(strcmp(label, "target_pid") == 0){
                    target_pid = value;
                    b++;
                }
                if(strcmp(label, "obstacles_pid") == 0){
                    obst_pid = value;
                    b++;
                }
                if(strcmp(label, "drone_pid") == 0){
                    drone_pid = value;
                    b++;
                }
                if(b>=4)
                    break;
            }
            else{
                fprintf(serverlog, "problems in the pid acquisation");
            }
        }

        int unlock_file = flock(fd, LOCK_UN);
        if(unlock_file == -1){
            perror("failed to unlock the file pid");
        }
        fclose(f);
        close(fd);
        fprintf(serverlog, "keyboard_pid:%d , target_pid:%d , obstacles_pid:%d , drone_pid:%d\n", key_pid, target_pid, obst_pid, drone_pid);
        fclose(serverlog);
    }
}

//function to check if the obstacles and the targets are coincident
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

//function to check id the drone is near to a target
int near(int cx1, int cy1, int cx2, int cy2){
    float rho = sqrt(pow(cx1-cx2, 2)+pow(cy1-cy2, 2));
    if(rho < rho2)
        return 1;
    return 0;
}

//function to check if the drone is near to an obstacle and evalueates the new force acting on the drone
float near_obst(int cx1, int cy1, int cx2, int cy2, float v){
    float rho = sqrt(pow(cx1-cx2, 2)+pow(cy1-cy2, 2));
    float theta = atan2(cy1-cy2, cx1-cx2);
    float forz;
    if(rho < rho0){
        forz = eta*(1/rho - 1/rho0)*(1/pow(rho, 2)*cos(theta)*abs(v));
    }
    else{
        forz = 0;
    }
    if(forz > MAXF){
        forz = MAXF;
    }
    return forz;
}

int main(int argc, char* argv[]){
    //initialization of ncurses
    initscr();

    FILE* routine = fopen("files/routine.log", "a");
    FILE* error = fopen("files/error.log", "a");
    FILE* serverlog = fopen("files/server.log", "a");

    if(error == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(routine == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(serverlog == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    RegToLog(routine, "SERVER : start\n");

    if(has_colors()){
        start_color();  //enables the color
        init_pair(1, COLOR_BLUE, COLOR_WHITE);  //define the window color
        init_pair(2, COLOR_RED, COLOR_WHITE); //define the obstacle color
        init_pair(3, COLOR_GREEN, COLOR_WHITE); //define the target color
        //fprintf(serverlog, "has colors\n");
        //fflush(serverlog);
    }

    SharedMemory *sm;  //shared memory pointer
    //shared memory opening and mapping
    const char * shm_name = "/shared_memory";
    const int SIZE = 4096;
    int i, shm_fd;
    shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if(shm_fd == 1){
        perror("shared memory faild\n");
        RegToLog(error, "SERVER : shared memory faild");
        exit(EXIT_FAILURE);
    }
    else{
        //printf("SERVER : created the shared memory");
        RegToLog(serverlog, "SERVER : created the shared memory");
    }

    if(ftruncate(shm_fd, SIZE) == 1){
        perror("ftruncate");
        RegToLog(error, "SERVER : ftruncate faild");
        exit(EXIT_FAILURE);
    }

    sm = (SharedMemory *)mmap(0, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if(sm == MAP_FAILED){
        perror("map failed");
        RegToLog(error, "SERVER : map faild");
        exit(EXIT_FAILURE);
    }

    //semaphore opening
    sem_t * sm_sem;
    sm_sem = sem_open("/sm_sem1", O_CREAT | O_RDWR, 0666, 1);
    if(sm_sem == SEM_FAILED){
        RegToLog(error, "SERVER : semaphore faild");
        perror("semaphore");
    }

    struct sigaction sa;  //initialize sigaction
    memset(&sa, 0, sizeof(sa));
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
    if(sigaction(34, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "SERVWR : error in sigaction()");
        exit(EXIT_FAILURE);
    }

    //pipe to give to the drone, the obstacles and the target the max x and y
    int writesd, writesd1, writesd2;

    writesd = atoi(argv[1]);
    writesd2 = atoi(argv[5]);
    writesd1 = atoi(argv[3]);
    //fprintf(serverlog, "wwrittesd: %d , writeeesd11: %d , writeee2: %d \n", writesd, writesd1, writesd2);
    //fflush(serverlog);

    int max_x, max_y;
    getmaxyx(stdscr, max_y, max_x);  //takes the max number of rows and colons
    //fprintf(serverlog, "maxx: %d , maxy: %d \n", max_x, max_y);
    //fflush(serverlog);
    
    write(writesd, &max_x, sizeof(int));
    fsync(writesd);
    
    sleep(1);
    
    write(writesd, &max_y, sizeof(int));
    fsync(writesd);
    
    sleep(1);
    //fprintf(serverlog, "ok 1  \n");
    //fflush(serverlog);
    
    write(writesd2, &max_x, sizeof(int));
    fsync(writesd2);
    
    sleep(1);
    
    write(writesd2, &max_y, sizeof(int));
    fsync(writesd2);
    sleep(1);
    //fprintf(serverlog, "ok 2  \n");
    //fflush(serverlog);
    
    write(writesd1, &max_x, sizeof(int));
    fsync(writesd1);
    
    sleep(1);
    
    write(writesd1, &max_y, sizeof(int));
    fsync(writesd1);
    
    sleep(1);
    //fprintf(serverlog, "ok 3  \n");
    //fflush(serverlog);

    srand(time(NULL));  //initialise random seed

    int droneposx, droneposy;
    int droneforx, dronefory;

    droneposx = rand() % max_x;  //random column
    droneposy = rand() % max_y;  //random row
    if(droneposx <= 1)
        droneposx = 2;
    if(droneposx >= max_x-1)
        droneposx = max_x-2;
    if(droneposy <= 1)
        droneposy = 2;
    if(droneposy >= max_y)
        droneposy = max_y-1;

    //fprintf(serverlog, "posx: %d , posy: %d \n", droneposx, droneposy);
    //fflush(serverlog);

    //sends the drone position to the drone process
    write(writesd, &droneposx, sizeof(int));
    fsync(writesd);
    sleep(1);
    write(writesd, &droneposy, sizeof(int));
    fsync(writesd);
    sleep(1);

    int readsd;
    int varre;
    readsd = atoi(argv[2]);
    //fprintf(serverlog, "readsd: %d\n", readsd);
    //fflush(serverlog);

    //takes the number of targets and obstacles from the file
    int fp;
    const char* filename = "files/data.txt";
    int n_obst, n_tar, at = 0;
    char line[MAX_LINE_LENGHT];

    fp = open(filename, O_RDONLY);
    if(fp == -1){
        perror("fp opening");
        RegToLog(error, "SERVER: error in opening fp");
        exit(EXIT_FAILURE);
    }

    if(flock(fp, LOCK_SH) == -1){
        perror("lock");
        RegToLog(error, "OBSTACLES: error in lock");
        close(fp);
        exit(EXIT_FAILURE);
    }

    FILE* file = fdopen(fp, "r");

    int b =  0;
    while(fgets(line, sizeof(line), file) != NULL){
        char label[MAX_LINE_LENGHT];
        int value;
        if(sscanf(line, "%[^:]:%d", label, &value) == 2){
            if(strcmp(label, "N_OBSTACLES") == 0){
                n_obst = value;
                b++;
            }
            if(strcmp(label, "N_TARGET") == 0){
                n_tar = value;
                b++;
            }
            if(b >= 2)
                break;
        }
        else{
            fprintf(serverlog, "problems in the pid acquisation");
            fflush(serverlog);
        }
    }

    if(flock(fp, LOCK_UN) == -1){
        perror("unlock");
        RegToLog(error, "OBSTACLES: error in unlock");
        close(fp);
        exit(EXIT_FAILURE);
    }

    fclose(file);
    //fprintf(serverlog, "nobst: %d , ntar: %d \n", n_obst, n_tar);
    //fflush(serverlog);

    //variables to store the obstacles and target position
    int obst[2][n_obst];
    int target[2][n_tar];
    int readsd1, readsd2;
    readsd1 = atoi(argv[4]);
    readsd2 = atoi(argv[6]);
    //fprintf(serverlog, "readsd1: %d , readsd2: %d\n", readsd1, readsd2);
    //fflush(serverlog);

    WINDOW *win = newwin(max_y, max_x, 0, 0);  //creats the window
    box(win, 0, 0);  //adds the borders
    wbkgd(win, COLOR_PAIR(1));  //sets the color of the window
    wrefresh(win);  //prints and refreshes the window

    kill(obst_pid, SIGUSR1);
    sleep(1);

    //fprintf(serverlog, "taking the obstacles\n");
    //fflush(serverlog);

    //takes the obstacles and send the signal to stop
    for(int m = 0; m < n_obst; m++){
        varre = -1;
        while(varre == -1){
            varre = read(readsd1, &obst[0][m], sizeof(int));
            sleep(1);
        }

        varre = -1;
        while(varre == -1){
            varre = read(readsd1, &obst[1][m], sizeof(int));
            sleep(1);
        }

        //fprintf(serverlog, "obst coord %d --> x %d  y %d\n", m, obst[0][m], obst[1][m]);
        //fflush(serverlog);
    }
    kill(obst_pid, SIGUSR1);
    sleep(1);

    kill(target_pid, SIGUSR1);
    sleep(1);

    //fprintf(serverlog, "taking the targets\n");
    //fflush(serverlog);

    //takes the targets and send the signal to stop
    for(int l = 0; l < n_tar; l++){
        varre = -1;
        while(varre == -1){
            varre = read(readsd2, &target[0][l], sizeof(int));
            sleep(1);
        }

        varre = -1;
        while(varre == -1){
            varre = read(readsd2, &target[1][l], sizeof(int));
            sleep(1);
        }

        //fprintf(serverlog, "tar coord %d --> x %d  y %d\n", l, target[0][l], target[1][l]);
        //fflush(serverlog);
    }
    
    kill(target_pid, SIGUSR1);
    sleep(1);
    
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
                wrefresh(win);
                wattr_off(win, COLOR_PAIR(2), NULL);
            }
            
        }
        if(arr_c[1][z] == -1){
            if(z<n_tar){
                wattr_on(win, COLOR_PAIR(3), NULL);
                mvwprintw(win, target[1][z], target[0][z], "O");  //prints the targets
                wrefresh(win);
                wattr_off(win, COLOR_PAIR(3), NULL);
            }
            
        }
    }
    
    int r = 0, score = 0, obt = 0, trt = 0;
    int memo[2][1];
    while(!sigint_rec){  //print the drone in the new position and delet the old one
        if(r >0){
            //deleting the old drone's podition
            wattr_on(win, COLOR_PAIR(1), NULL);
            mvwprintw(win, droneposy, droneposx, " ");  //prints the drone
            wattr_off(win, COLOR_PAIR(1), NULL);
            wrefresh(win);
            
            r--;
        }

        //taking the new drone position
        varre = -1;
        while(varre == -1){
            varre = read(readsd, &droneposx, sizeof(int));
            sleep(1);
        }
        
        varre = -1;
        
        while (varre == -1){   
            varre = read(readsd, &droneposy, sizeof(int));
            sleep(1);
        }
        
        //fprintf(serverlog, "posx: %d ,, posy: %d \n", droneposx, droneposy);
        //fflush(serverlog);

        sem_wait(sm_sem);
        sm->score = score;
        sm->obst = obt;
        sm->target = n_tar-trt;
        sem_post(sm_sem);

        wattr_on(win, COLOR_PAIR(1), NULL);
        mvwprintw(win, droneposy, droneposx, "%c", form);  //prints the drone
        wattr_off(win, COLOR_PAIR(1), NULL);
        wrefresh(win);

        sem_wait(sm_sem);
        float vx = sm->vel[0];
        float vy = sm->vel[1];
        sem_post(sm_sem);

        //check if the drone is near to an obstacles, in that case the repulse forse act and the forces are shared whit the keyboard
        for(int f = 0; f<n_obst; f++){
            float a = near_obst(droneposx, droneposy, obst[0][f], obst[1][f], vx);
            float b = near_obst(droneposx, droneposy, obst[0][f], obst[1][f], vy);
            if(a != 0 || b != 0){
                a = ceil(a);
                b = ceil(b);
                int writesd4 = atoi(argv[7]);
                //fprintf(serverlog, "writesd4:%d , a:%f , b:%f", writesd4, a, b);
                //fflush(serverlog);

                kill(key_pid, SIGUSR1);
                sleep(1);

                write(writesd4, &a, sizeof(int));
                fsync(writesd4);
                sleep(1);
                write(writesd4, &b, sizeof(int));
                fsync(writesd4);
                sleep(1);

                close(writesd4);
            }
        }
        
        //check if the drone pursuit a gol or touch a obstacles and stores in the memo double array the witch target or obstacle is involved
        for(int c =0; c<n; c++){
            if(near(droneposx, droneposy, obst[0][c], obst[1][c]) && c<n_obst){
                wattr_on(win, COLOR_PAIR(2), NULL);
                mvwprintw(win, obst[1][c], obst[0][c]-2, "CRASH");  //prints the obstacles
                wattr_off(win, COLOR_PAIR(2), NULL);
                wrefresh(win);
                score--;
                sleep(2);
                wattr_on(win, COLOR_PAIR(2), NULL);
                mvwprintw(win, obst[1][c], obst[0][c]-2, "     ");  //prints the obstacles
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
                mvwprintw(win, target[1][c], target[0][c], " ");  //prints the obstacles
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
                obt++;
                obst[0][memo[0][0]] = -1;
                obst[1][memo[0][0]] = -1;
                
            }

            if(memo[1][0] >= 0){
                RegToLog(routine, "SERVER: target taken");
                trt++;
                target[0][memo[1][0]] = -1;
                target[1][memo[1][0]] = -1;
                
            }

            memo[0][0] = -1;
            memo[1][0] = -1;     
        }

        //takes the values from the shared memory and prints them on top of the window
        sem_wait(sm_sem);
        double velx = sm->vel[0];
        double vely = sm->vel[1];
        int sco = sm->score;
        int obstmanc = sm->obst;
        int tartaken = sm->target;
        int forx = sm->forces[0];
        int fory = sm->forces[1];
        sem_post(sm_sem);
        wattr_on(win, COLOR_PAIR(1), NULL);
        mvwprintw(win, 0, 0, "fx:%d fy:%d vx:%f vy:%f target missing:%d obstacles taken:%d SCORE:%d",forx,fory,velx,vely,tartaken,obstmanc,sco);
        wattr_off(win, COLOR_PAIR(2), NULL);
        wrefresh(win);

        sleep(1);
        r++;
    }

    //routine to close the shared memory, the files, the pipes and the semaphore
    if(shm_unlink(shm_name) == 1){
        printf("okok");
        exit(EXIT_FAILURE);
    }
    if(close(shm_fd) == 1){
        perror("close");
        RegToLog(error, "SERVER : close faild");
        exit(EXIT_FAILURE);
    }

    sem_close(sm_sem);
    close(readsd1);
    close(readsd2);
    close(readsd);
    close(writesd);
    close(writesd1);
    close(writesd2);
    munmap(sm, SIZE);
    fclose(error);
    fclose(routine);
    fclose(serverlog);
    return 0;
}