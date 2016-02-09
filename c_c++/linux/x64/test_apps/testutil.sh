#!/bin/bash

PATH=$PATH:~/Research/repositories/git/riffa/c_c++/linux/x64/test_apps/
RESULT_PATH=~/results 

if [ "$#" -lt 2 ]; then 
    echo "usage: $0 <fpga id> <board name>"
    exit
fi


# Run full bandwidth test
testutil 2 $1 0 536970912 | tee $RESULT_PATH/$2_bw.txt

# Run RX Sweep Test
testutil 3 $1 0 2048 | tee $RESULT_PATH/$2_rx.txt

# Run TX Sweep Test
testutil 4 $1 0 2048 | tee $RESULT_PATH/$2_tx.txt



