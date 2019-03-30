#!/bin/bash
cd CodeCraft-2019
car_path=/home/xi/Documents/code/competition/2019huawei/2019软挑-初赛-SDK/SDK/SDK_python/CodeCraft-2019/1-map-exam-1/car.txt
road_path=/home/xi/Documents/code/competition/2019huawei/2019软挑-初赛-SDK/SDK/SDK_python/CodeCraft-2019/1-map-exam-1/road.txt
cross_path=/home/xi/Documents/code/competition/2019huawei/2019软挑-初赛-SDK/SDK/SDK_python/CodeCraft-2019/1-map-exam-1/cross.txt
answer_path=/home/xi/Documents/code/competition/2019huawei/2019软挑-初赛-SDK/SDK/SDK_python/CodeCraft-2019/1-map-exam-1/answer.txt
time python src/CodeCraft-2019.py  $car_path  $road_path $cross_path $answer_path

python ../simulator.py $car_path  $road_path $cross_path $answer_path



