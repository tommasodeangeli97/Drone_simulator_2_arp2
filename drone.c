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
#define FORCE 25
#define MAX_LINE_LENGHT 256

//int check_near = 0;
pid_t server_pid, key_pid;
bool sigint_rec = 0;

typedef struct{  //shared memory
    int forces[2];
    double vel[2];
    int score;
    int obst;
    int target;
} SharedMemory;

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
            //check_near++;
            fclose(routine);
        }
        else{
            fprintf(routine, "%s\n", "DRONE : started success");
            kill(watch_pid, SIGUSR1);
            fclose(routine);
        }
    }

    if(signo == SIGUSR2){
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "DRONE : program terminated by WATCHDOG");
        fclose(routine);
        exit(EXIT_FAILURE);
    }

    if(signo == SIGINT){
        printf("drone terminating return 0");
        FILE* routine = fopen("files/routine.log", "a");
        fprintf(routine, "%s\n", "DRONE : terminating");
        fclose(routine);
        sigint_rec = 1;
    }

    if(signo == 34){
        
        FILE* dronelog = fopen("files/drone.log", "a");

        int fd;
        const char* logfile = "files/pidlog.log";
        const char* search = "server_pid:%d";
        const char* search2 = "keyboard_pid:%d";
        char pidline[MAX_LINE_LENGHT];
        fd = open(logfile, O_RDONLY);
        if(fd == -1){
            perror("fp opening");
            fprintf(dronelog, "error in fd");
            exit(EXIT_FAILURE);
        }
    
        int lock_file = flock(fd, LOCK_SH);
        if(lock_file == -1){
            perror("failed to lock the file pid");
            fprintf(dronelog, "error in lock");
            exit(EXIT_FAILURE);
        }
        FILE* f = fdopen(fd, "r");
        int b = 0;
        while(fgets(pidline, sizeof(pidline), f) != NULL){
            char label[MAX_LINE_LENGHT];
            int value;
            if(sscanf(pidline, "%[^:]:%d", label, &value) == 2){
                if(strcmp(label, "server_pid") == 0){
                    server_pid = value;
                    b++;
                }
                if(strcmp(label, "keyboard_pid") == 0){
                    key_pid = value;
                    b++;
                }
                if(b>=2)
                    break;
            }
            else{
                fprintf(dronelog, "problems in the pid acquisation");
                fflush(dronelog);
            }
        }

        int unlock_file = flock(fd, LOCK_UN);
        if(unlock_file == -1){
            perror("failed to unlock the file pid");
        }
        fclose(f);
        close(fd);
        fprintf(dronelog, "server_pid: %d , keyboard_pid: %d \n", server_pid, key_pid);
        fclose(dronelog);
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

//calculates the repulsive forse acting on the drone like a uniform repulsive force alway double respect the force acting already on the drone or by a cont value if the forze is 0
/*int new_for(int forz, double vel){
    if(vel > 0 && forz > 0){
        forz = forz-2*forz;
    }
    else if(vel > 0 && forz < 0){
        forz = 2*forz;
    }
    else if(vel > 0 && forz == 0){
        forz = -4;
    }
    else if(vel < 0 && forz > 0){
        forz = 2*forz;
    }
    else if(vel < 0 && forz < 0){
        forz = forz+2*forz;
    }
    else if(vel < 0 && forz == 0){
        forz = +4;
    }
    return forz;
}*/
int forx = 0, fory = 0;
int main(int argc, char* argv[]){
    
    FILE* routine = fopen("files/routine.log", "a");
    FILE* error = fopen("files/error.log", "a");
    FILE* dronelog = fopen("files/drone.log", "a");
    /*fd_set read_fds;
    fd_set write_fds;
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);*/

    if(error == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(routine == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    if(dronelog == NULL){
        perror("fopen");
        exit(EXIT_FAILURE);
    }
    RegToLog(routine, "DRONE : start\n");

    SharedMemory *sm;  //shared memory pointer
    //shared memory opening and mapping
    const char * shm_name = "/shared_memory";
    const int SIZE = 4096;
    int i, shm_fd;
    shm_fd = shm_open(shm_name, O_RDWR, 0666);
    if(shm_fd == -1){
        perror("shared memory faild\n");
        RegToLog(error, "DRONE : shared memory faild");
        exit(EXIT_FAILURE);
    }

    sm = (SharedMemory *)mmap(0, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if(sm == MAP_FAILED){
        perror("map failed");
        RegToLog(error, "DRONE : map faild");
        return 1;
    }

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
    if(sigaction(SIGINT, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "DRONE : error in sigaction()");
        exit(EXIT_FAILURE);
    }
    if(sigaction(34, &sa, NULL) == -1){
        perror("sigaction");
        RegToLog(error, "DRONE : error in sigaction()");
        exit(EXIT_FAILURE);
    }

    /*int fd;
    const char* logfile = "files/pidlog.log";
    const char* search = "server_pid:%d";
    const char* search2 = "keyboard_pid:%d";
    char pidline[MAX_LINE_LENGHT];
    fd = open(logfile, O_RDONLY);
    if(fd == -1){
        perror("fp opening");
        RegToLog(error, "DRONE: error in opening fd");
        exit(EXIT_FAILURE);
    }
    
    int lock_file = flock(fd, LOCK_SH);
    if(lock_file == -1){
        perror("failed to lock the file pid");
        RegToLog(error, "DRONE: error in lock");
        exit(EXIT_FAILURE);
    }
    FILE* f = fdopen(fd, "r");
    int b = 0;
    while(fgets(pidline, sizeof(pidline), f) != NULL){
        char label[MAX_LINE_LENGHT];
        int value;
        if(sscanf(pidline, "%[^:]%d", label, &value) == 2){
            if(strcmp(label, "server_pid") == 0){
                server_pid = value;
                b++;
            }
            if(strcmp(label, "keyboard_pid") == 0){
                key_pid = value;
                b++;
            }
        }
        else{
            fprintf(dronelog, "problems in the pid acquisation");
            RegToLog(error, "problems in the pid acquisition");
        }
        if(b>=2)
            break;
    }

    int unlock_file = flock(fd, LOCK_UN);
    if(unlock_file == -1){
        perror("failed to unlock the file pid");
    }
    fclose(f);
    fprintf(dronelog, "server_pid: %d , keyboard_pid: %d \n", server_pid, key_pid);*/

    //use of pipe to take from the server the max of x and y
    int readsd, readsd3;
    int varre = -1;
    int maxx, maxy;
    int writesd;
    writesd = atoi(argv[3]);
    readsd = atoi(argv[1]);
    readsd3 = atoi(argv[2]);
    
    fprintf(dronelog, "writesd: %d , readsd: %d , readsd3: %d\n", writesd, readsd, readsd3);
    fflush(dronelog);
    //FD_SET(readsd, &read_fds);

    //sem_wait(sm_sem);
    //do{
    varre = read(readsd, &maxx, sizeof(int));
    //}while(varre == -1 && errno == EINTR);
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "DRONE : error in readsd 1");
    }
    sleep(1);
    //sem_post(sm_sem);
    //sleep(1);
    //sem_wait(sm_sem);
    fprintf(dronelog, "max x: %d \n", maxx);
    fflush(dronelog);
    //do{
    varre = read(readsd, &maxy, sizeof(int));
    //}while(varre == -1 && errno == EINTR);
    if( varre == -1){
        perror("readsd");
        RegToLog(error, "DRONE : error in readsd 2");
    }
    sleep(1);
    fprintf(dronelog, "max y: %d \n", maxy);
    fflush(dronelog);
    //sem_post(sm_sem);

    int x = 0;
    int y = 0;
    double accx, accy;
    double velx = 0.0;
    double vely = 0.0;
    int posx, posy;
    //int forx = 0, fory = 0;

    sleep(1);
    
    while(x == 0){
    //sem_wait(sm_sem);
    //do{
        varre = read(readsd, &x, sizeof(int));
    //}while(varre == -1 && errno == EINTR);
        if( varre == -1){
           perror("readsd");
            RegToLog(error, "DRONE : error in readsd 3");
        }
        sleep(1);
    }
    fprintf(dronelog, "x: %d \n", x);
    fflush(dronelog);
    //sem_post(sm_sem);
    //sleep(1);
    //sem_wait(sm_sem);
    //do{
    while((y == 0)){
        varre = read(readsd, &y, sizeof(int));
    //}while(varre == -1 && errno == EINTR);
        if( varre == -1){
            perror("readsd");
            RegToLog(error, "DRONE : error in readsd 4");
        }
        sleep(1);
    }
    fprintf(dronelog, "y: %d \n", y);
    fflush(dronelog);
    close(readsd);
    //sem_post(sm_sem);

    //FD_SET(writesd, &write_fds);
    //FD_SET(readsd3, &read_fds);
    //sleep(1);
    varre = -1;
    while(!sigint_rec){  //takes from the keyboard the force acting on the drone and calculates the accelleration, the velocity and the position
        //FD_SET(readsd3, &read_fds);
        //sem_wait(sm_sem);
        //do{
        //kill(key_pid, SIGUSR1);
        //sleep(1);
        varre = -1;
        while(varre == -1){
            varre = read(readsd3, &forx, sizeof(int));
            RegToLog(dronelog, "DRONE : error in readsd3 1\n");
            sleep(1);
        }
            
        //}while(varre == -1 && errno == EINTR);
        /*if( varre == -1){
            perror("readsd");
            RegToLog(error, "DRONE : error in readsd3 1");
        }
        sleep(1);*/
        fprintf(dronelog, "forx: %d \n", forx);
        sem_wait(sm_sem);
        sm->forces[0] = forx;
        sem_post(sm_sem);

        //kill(key_pid, SIGUSR1);
        sleep(1);
        //sem_post(sm_sem);
        //sleep(1);
        //sem_wait(sm_sem);
        //do{
        varre = -1;
        while(varre == -1){
            varre = read(readsd3, &fory, sizeof(int));
            RegToLog(dronelog, "DRONE : error in readsd3 2\n");
            sleep(1);
        }
        //kill(key_pid, SIGUSR1);
        sleep(1);
        //}while(varre == -1 && errno == EINTR);
        /*if( varre == -1){
            perror("readsd");
            RegToLog(error, "DRONE : error in readsd3 2");
        }
        sleep(1);*/
        fprintf(dronelog, "fory: %d \n", fory);
        sem_wait(sm_sem);
        sm->forces[1] = fory;
        sem_post(sm_sem);
        //varre = -1;

        accx = acceleration(forx*FORCE);
        velx = velocity(accx, velx);
        sem_wait(sm_sem);
        sm->vel[0] = velx;
        sem_post(sm_sem);
        posx = position(x, velx);
        //the drone can't go out of the screen
        if(posx >= maxx-1){
            x = maxx-2;
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
        sem_wait(sm_sem);
        sm->vel[1] = vely;
        sem_post(sm_sem);
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

        /*if(check_near>0){
            RegToLog(routine, "DRONE: repulsive force acting");

            int forxx = new_for(forx, velx);
            int foryy = new_for(fory, vely);
            sm->forces[0] = forxx;
            sm->forces[1] = foryy;
            accx = acceleration(forxx*FORCE);
            velx = velocity(accx, velx);
            sm->vel[0] = velx;
            posx = position(x, velx);
            //the drone can't go out of the screen
            if(posx >= maxx-1){
                x = maxx-2;
                RegToLog(routine, "DRONE : x lower limit");
            }
            else if(posx <= 1){
               x = 2;
                RegToLog(routine, "DRONE : x upper limit");
            }
            else{
                x = posx;
            }
            accy = acceleration(foryy*FORCE);
            vely = velocity(accy, vely);
            sm->vel[1] = vely;
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
            fprintf(dronelog, "forxx: %d , foryy: %d \n", forxx, foryy);
            fflush(dronelog);

            kill(key_pid, SIGUSR1);
            sleep(1);

            write(writesd3, &forxx, sizeof(int));
            fsync(writesd3);
            sleep(1);
            
            write(writesd3, &foryy, sizeof(int));
            fsync(writesd3);
            sleep(1);

            check_near = 0;
        }*/
        //if(check_near == 0){
            //FD_SET(writesd, &write_fds);
        fprintf(dronelog, "x:%d  y:%d\n", x, y);
        fflush(dronelog);
        write(writesd, &x, sizeof(int));
        fsync(writesd);
        sleep(1);
        write(writesd, &y, sizeof(int));
        fsync(writesd);
        sleep(1);
        //}
            
        //fprintf(routine, "%d %d\n", x, y);
        //sleep(1);
    }

    //routine to close the shared memory, the files and the semaphore
    if(shm_unlink(shm_name) == 1){
        printf("okok");
        exit(EXIT_FAILURE);
    }
    if(close(shm_fd) == 1){
        perror("close");
        RegToLog(error, "DRONE : close faild");
        exit(EXIT_FAILURE);
    }

    sem_close(sm_sem);
    close(writesd);
    //close(writesd3);
    //close(readsd);
    close(readsd3);
    munmap(sm, SIZE);
    fclose(dronelog);
    fclose(error);
    fclose(routine);

    return 0;
}