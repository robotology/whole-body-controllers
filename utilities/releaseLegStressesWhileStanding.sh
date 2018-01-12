#!/bin/bash

list="4 5"

for a in $list; do
	echo "set icmd cmod $a idl"  | yarp rpc /icub/right_leg/rpc:i
	echo "set icmd cmod $a idl"  | yarp rpc /icub/left_leg/rpc:i
done

sleep 0.25

for a in $list; do
	echo "set icmd cmod $a pos"  | yarp rpc /icub/right_leg/rpc:i
	echo "set icmd cmod $a pos"  | yarp rpc /icub/left_leg/rpc:i
done

list="1"

for a in $list; do
	echo "set icmd cmod $a idl"  | yarp rpc /icub/right_leg/rpc:i
	echo "set icmd cmod $a idl"  | yarp rpc /icub/left_leg/rpc:i
done

sleep 0.25

for a in $list; do
	echo "set icmd cmod $a pos"  | yarp rpc /icub/right_leg/rpc:i
	echo "set icmd cmod $a pos"  | yarp rpc /icub/left_leg/rpc:i
done

yarp clean --timeout 0.2
