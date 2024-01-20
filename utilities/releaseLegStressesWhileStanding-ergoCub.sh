#!/bin/bash

# Release the internal stresses when the robot is balancing on two feet,
# due to the closed kinematic chain of the legs. The legs are put in
# `idle` mode for 0.25 seconds.
list="4 5"

for a in $list; do
	echo "set icmd cmod $a idl"  | yarp rpc /ergocub/right_leg/rpc:i
	echo "set icmd cmod $a idl"  | yarp rpc /ergocub/left_leg/rpc:i
done

sleep 0.25

for a in $list; do
	echo "set icmd cmod $a pos"  | yarp rpc /ergocub/right_leg/rpc:i
	echo "set icmd cmod $a pos"  | yarp rpc /ergocub/left_leg/rpc:i
done

list="1"

for a in $list; do
	echo "set icmd cmod $a idl"  | yarp rpc /ergocub/right_leg/rpc:i
	echo "set icmd cmod $a idl"  | yarp rpc /ergocub/left_leg/rpc:i
done

sleep 0.25

for a in $list; do
	echo "set icmd cmod $a pos"  | yarp rpc /ergocub/right_leg/rpc:i
	echo "set icmd cmod $a pos"  | yarp rpc /ergocub/left_leg/rpc:i
done

yarp clean --timeout 0.2