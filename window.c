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
#include <termios.h>

typedef struct{  //shared memory
    int forces[2];
    double vel[2];
    int score;
    int obst;
    int target;
} SharedMemory;

bool checkout = FALSE;

#define CLOSE 'a'

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

void inputhandler(char key){
    FILE* routine = fopen("files/routine.log", "a");

    switch(key){
        case CLOSE:
            checkout = TRUE;
            break;
        default:
            break;
    }
}

/*char GetInput(){  //function to acquire the input from the user
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
}*/

int main(){
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
    RegToLog(routine, "WINDOW : start\n");

    if(has_colors()){
        start_color();  //enables the color
        init_pair(1, COLOR_BLUE, COLOR_WHITE);  //define the window color
    }

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

    WINDOW *win2 = newwin(15, 15, 0, 0);  //creats the window
    box(win2, 0, 0);  //adds the borders
    wbkgd(win2, COLOR_PAIR(1));  //sets the color of the window
    wrefresh(win2);  //prints and refreshes the window

    /*double velx = sm->vel[0];
    double vely = sm->vel[1];
    int sco = sm->score;
    int obstmanc = sm->obst;
    int targmanc = sm->target;
    int forx = sm->forces[0];
    int fory = sm->forces[1];

    wattr_on(win2, COLOR_PAIR(1), NULL);
    mvwprintw(win2, 1, 1, "force on x: %d", forx);
    mvwprintw(win2, 2, 1, "force on y: %d", fory);
    mvwprintw(win2, 3, 1, "velocity on x: %f", velx);
    mvwprintw(win2, 4, 1, "velocity on y: %f", vely);
    mvwprintw(win2, 6, 1, "target missing: %d", targmanc);
    mvwprintw(win2, 7, 1, "obstacles: %d", obstmanc);
    mvwprintw(win2, 9, 1, "TOTAL SCORE: %d", sco);
    wattr_off(win2, COLOR_PAIR(2), NULL);
    wrefresh(win2);*/

    while(!checkout){
        sem_wait(sm_sem);
        double velx = sm->vel[0];
        double vely = sm->vel[1];
        int sco = sm->score;
        int obstmanc = sm->obst;
        int tartaken = sm->target;
        int forx = sm->forces[0];
        int fory = sm->forces[1];
        sem_post(sm_sem);

        wattr_on(win2, COLOR_PAIR(1), NULL);
        mvwprintw(win2, 1, 1, "force on x: %d", forx);
        mvwprintw(win2, 2, 1, "force on y: %d", fory);
        mvwprintw(win2, 3, 1, "velocity on x: %f", velx);
        mvwprintw(win2, 4, 1, "velocity on y: %f", vely);
        mvwprintw(win2, 6, 1, "target missing: %d", tartaken);
        mvwprintw(win2, 7, 1, "obstacles: %d", obstmanc);
        mvwprintw(win2, 9, 1, "TOTAL SCORE: %d", sco);
        wattr_off(win2, COLOR_PAIR(2), NULL);
        wrefresh(win2);

        char ch = getch();  //acquire continously the keypad
        inputhandler(ch);
        sleep(1);
    }

    //routine to close the shared memory, the files and the semaphore
    if(shm_unlink(shm_name) == 1){
        printf("okok");
        exit(EXIT_FAILURE);
    }
    if(close(shm_fd) == 1){
        perror("close");
        RegToLog(error, "WINDOW : close faild");
        exit(EXIT_FAILURE);
    }

    sem_close(sm_sem);
    munmap(sm, SIZE);
    fclose(error);
    fclose(routine);

    return 0;
}