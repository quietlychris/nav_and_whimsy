#!/bin/bash
echo '***Beginning script run process, deleting previous log'
rm run_log.csv
clear
echo '***Running cargo'
cargo run
echo '***Plotting graph'
python graph.py
#echo 'Opening graph'
#atom graph.png
