#!/bin/sh

mkdir origin_test
cd origin_test

g++ ../*.cpp -o origin_eval1 -std=c++11 -DTEST_DEF=1
g++ ../*.cpp -o origin_eval2 -std=c++11 -DTEST_DEF=2
g++ ../*.cpp -o origin_eval3 -std=c++11 -DTEST_DEF=3
g++ ../*.cpp -o origin_eval4 -std=c++11 -DTEST_DEF=4
g++ ../*.cpp -o origin_eval5 -std=c++11 -DTEST_DEF=5
g++ ../*.cpp -o origin_eval6 -std=c++11 -DTEST_DEF=6
g++ ../*.cpp -o origin_eval7 -std=c++11 -DTEST_DEF=7
g++ ../*.cpp -o origin_eval8 -std=c++11 -DTEST_DEF=8


mkdir test_def1
cp ./origin_eval1 ./test_def1
cd ./test_def1
mkdir eval_data
./origin_eval1 &
sleep 2
cd ..

mkdir test_def2
cp ./origin_eval2 ./test_def2
cd ./test_def2
mkdir eval_data
./origin_eval2 &
sleep 2
cd ..

mkdir test_def3
cp ./origin_eval3 ./test_def3
cd ./test_def3
mkdir eval_data
./origin_eval3 &
sleep 2
cd ..


mkdir test_def4
cp ./origin_eval4 ./test_def4
cd ./test_def4
mkdir eval_data
./origin_eval4 &
sleep 2
cd ..


mkdir test_def5
cp ./origin_eval5 ./test_def5
cd ./test_def5
mkdir eval_data
./origin_eval5 &
sleep 2
cd ..


mkdir test_def6
cp ./origin_eval6 ./test_def6
cd ./test_def6
mkdir eval_data
./origin_eval6 &
sleep 2
cd ..


mkdir test_def7
cp ./origin_eval7 ./test_def7
cd ./test_def7
mkdir eval_data
./origin_eval7 &
sleep 2
cd ..

mkdir test_def8
cp ./origin_eval8 ./test_def8
cd ./test_def8
mkdir eval_data
./origin_eval8 &
cd ..

