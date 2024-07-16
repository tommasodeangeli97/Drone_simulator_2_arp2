cc -o "master" "master.c"
if [ $? -eq 0 ]; then
        echo "compilazione di MASTER completata"
    else
        echo "errore compilazione MASTER"
    fi

cc -o "server" "server.c" "-lncurses" -lm
if [ $? -eq 0 ]; then
        echo "compilazione di SERVER completata"
    else
        echo "errore compilazione SERVER"
    fi

cc -o "drone" "drone.c" -lm
if [ $? -eq 0 ]; then
        echo "compilazione di DRONE completata"
    else
        echo "errore compilazione DRONE"
    fi

cc -o "keyboard" "keyboard.c" "-lncurses"
if [ $? -eq 0 ]; then
        echo "compilazione di KEYBOARD completata"
    else
        echo "errore compilazione KEYBOARD"
    fi

cc -o "watchdog" "watchdog.c"
if [ $? -eq 0 ]; then
        echo "compilazione di WATCHDOG completata"
    else
        echo "errore compilazione WATCHDOG"
    fi

cc -o "obstacles" "obstacles.c"
if [ $? -eq 0 ]; then
        echo "compilazione di OBSTACLES completata"
    else
        echo "errore compilazione OBSTACLES"
    fi

cc -o "target" "target.c"
if [ $? -eq 0 ]; then
        echo "compilazione di TARGET completata"
    else
        echo "errore compilazione TARGET"
    fi