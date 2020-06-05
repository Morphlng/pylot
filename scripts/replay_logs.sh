#!/bin/bash
# $1 Directory where the log files are

cd ${CARLA_HOME}/PythonAPI/examples
for FILE_NAME in `ls $1`; do
    HERO_ID=`python3 show_recorder_file_info.py -f ${1}/${FILE_NAME} | grep -B 5 hero | head -n 1 | cut -d' ' -f3 | cut -d':' -f1`
    python3 start_replaying.py -f ${1}/${FILE_NAME} -c ${HERO_ID} -x 5
    read -p "Press any key when the replay completes... " -n1 -s
done
