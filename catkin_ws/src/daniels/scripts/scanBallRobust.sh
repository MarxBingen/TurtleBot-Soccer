#!/bin/sh

RC=1

while [ $RC -ne 0 ];
do
	rosrun daniels ScanBall
	RC=$?
done
