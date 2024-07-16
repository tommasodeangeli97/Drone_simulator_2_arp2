# Drone_simulator_2_arp2
Second assignament of Advanced and Robot programming: is a drone simulator implemented totally in c language and controllable in forces, the window is created using the *ncurses* library, in the environment random targets and random obstacles spawn, the obstacles create a repulsive force, the *SCORE* is a funtion of how many targets and obstacles are encountered by the drone. The drone is controlled by key inputs that change the total force applied to the drone.

## Description
The program is composed by seven different processes that cooperate in real time and share information using mostly pipes but also shared memory and files.
The final result gives to the user the possibility to move a drone in a free environment where the friction force, the forces intruduced to control it and the obstacles' repulsive forces are acting.
Furthermore the drone is unable to go out the screen.

These are the key to control the robot, however the `master` process visualises them befor starting the application:
```
UP 'e'
UP_LEFT 'w'
UP_RIGHT 'r'
RIGHT 'f'
0 FORCES 'd'
LEFT 's'
DOWN 'c'
DOWN_LEFT 'x'
DOWN_RIGHT 'v'
QUIT 'q'
```

## Architecture
![Slide1](https://github.com/user-attachments/assets/39a474c6-425d-43a7-8c00-ef46c628144a)

`Master` -> the process that fork and exec all the others and wait they're end; it creates the `pilog.log` file, it updates this file with the pid's number of the processes and sends a signal to collect them

`Keyboard` -> the process that collects the inputs from the user; it changes the inputs into forces and send them to the `drone` process using pipes; if the drone is too close to an obstacle the `server` process sent to the `keyboard` process a signal, when signal is recieved the keyboard takes the new values of the forces directly from the `server`

`Drone` -> the process that actually drive the drone; it takes the random position of the drone and the dimentions of the window by the `server` process trought pipes; it collects the force values from the `keyboard` process, it transforms the forces into acceleration, velocity and position of the drone; it updates the shared memory with the velocity and force values and shares the position with the `server` process using pipes

`Target` -> the process that randomly creates the targets; it takes from the `server` process the dimentions of the windows and from the `data.txt` file the number of targets; it creates randomly the targets and checks if one or more have the same coordinates; it waits a signal from the `server` to start sharing the points and another signal to stop the sharing phase

`Obstacles` -> very similar to the `target` process is the process that creates the random obstacles; it takes from the `server` process the dimentions of the window using pipes and reads the number of obstacles to create from the `data.txt` file; it creates the obstacles and check if one or more share the same coordinates; it waits a signal from the `server` to start sharing the points and another to stop the sharing phase

`Watchdog` -> the process that controls the functioning of the others; it checks if the others processes are still working or not sendig periodically signals to them, if one or more processes do not respond to the signal it closes the application

`Server` -> the main process of the application; it reads from the `data.txt` file the number of obstacles and targets, it sends to the `obstacles` and `target` process the start and the finish signal, it collects the points and prints them, if the points are coincident it doesn't print them; it shares with the `drone`, `target` and `obstacles` process the dimentions of the window and with the `drone` process the random initial position of the drone using pipes; it updates the shared memory with the number of target and obstacles taken by the drone and it calculates the user score; it recieves from the `drone` process the updated position of the drone using pipes and prints it deleting also the old one, it checks if the drone's position id too close to an obstacle and calculates the repulsive force, in this case it sends a signal to the `keyboard` process and shares with it the new force values always using pipes; it reads from the shared memory the updated values and prints them on top of the window

## Overview


https://github.com/user-attachments/assets/917ee4b3-059f-4f8f-ba47-d88c1b00ed11


## Installation
To properly install the application assure you to have installe *konsole* and *ncurses* on your computer
```
$ sudo apt install konsole
$ sudo apt install libncurses-dev
```

then clone the repository on your pc
```
$ git clone https://github.com/tommasodeangeli97/Drone_simulator_2_arp2.git
```

by the *konsole* terminal go inside the folder
```
$ cd Drone_simulator_2_arp2/
```

make executable the compiler and run it
```
$ chmod +x compile.sh
$ ./compile.sh
```

now you are able to start the application
```
$ ./master
```

### important note
remenber to delet the `pidlog.log` file every time before start the application in order to have only the actual pids of the processes

## Possibles implementation
1. the velocity of the start of the application could be improved using the select() function or implementing the comunication phase of the processes

2. The drone could be further impruved to make it capable to rotate on itself

3. Even if it is not possible in reality could be implemented a instant block of the robot, a key to stops the robot regardless of the forces applied and the velocity accumulated

4. For the user point of view a special window can be impleted to avoid the closure of the application if the 'q' button is accidentally pressed

5. the score value can take in account more then only the targets and the obstacles incountered by the drone

## Author and contact
Author: Tommaso De Angeli (https://github.com/tommasodeangeli97)

contact: tommaso.deangeli.97@gmail.com
