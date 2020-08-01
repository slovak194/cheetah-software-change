#/bin/bash

QT_VER="$(ls ~/Qt5.14.0/ | grep 5 -m1)"

printf "${HOME}/Qt5.14.0/${QT_VER}/gcc_64/"

