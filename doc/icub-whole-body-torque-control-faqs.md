Frequently Asked Questions on Whole-Body Torque Control with the iCub Robot  {#icub-whole-body-control-faqs}
==================================

 ## Preliminary checks on the iCub
 
 ### What to check on the iCub robot before performing whole-body torque control experiments? 
 
# Complete update of information

## Update yarp, icub-main and codyco-superbuild
First, update the yarp, icub-main and codyco\-superbuild directories on the icub laptop, as well as on the computer used for launching controllers (if different), with  
`git pull`  
You don't have to repeat this operation on computers which share sources with the icub laptop.

Then, if there are some updates, build them on all the computers with  
`cd build`  
`ccmake  ../`  
`make`  

To do the required updates on the iCub board (pc104 or icub-head), first connect to it using the command  
`ssh -X pc104` (\-X option is used to redirect the graphic output to your local machine)  
or  
`ssh -X icub-head` 

Additionnally, update codyco-superbuild on the computer from which you will launch controllers with  
`cd build`  
`make update-all`  

## Firmware update

*iCub software version anterior to 1.8.0*:  
This is only a quick summary of what you have to do. For more information, look [here](http://wiki.icub.org/wiki/Firmware).

`cd /usr/local/src/robot/icub-firmware-build/`  
`git pull`  

Then, launch the cluster with  
`icub_cluster.py`  (or a script that configures and launches the icub_cluster.py)  
Now, you can open _eth_loader_ and click “_discover_” to make sure that the server is connected (where yarpserver is launched). Now, you can check that all computers are connected together (if not, you just have to select the unconnected computer and click  “_connect_").
Open [_can_loader_](http://wiki.icub.org/wiki/CanLoader) and update the version of devices by searching for the corresponding EMS_file and uploading it.  

*Latest iCub software version*:   
Look [here](https://github.com/robotology/QA/issues/240) for instructions on how to perform an update using `firmware-updater`.

## Devices update
Before proceeding to the calibration, you have to update the device information.
You can find information about CAN-bus numbers and CAN-bus device drivers [here](http://wiki.icub.org/wiki/Can_addresses_and_associated_firmware#PCAN2:_Right_arm). Example for icubDarmstadt01:  
- Select EMS and can2: you can see the F/T sensors information (_strain_).  
- For ID 1,3,6,7,8,9, we select the "strain.hex" files one by one and update them.  
- For ID 2, 4 it is different because they correspond to the skin sensors: download the "skin.hex" file, if needed.  
- Select EMS and can1, which is about motion control: type MAIS for the skin and RM 4DC for motors. Download all corresponding "\.hex" files by clicking the _download_ button.  
- If you select _net0_ can\-bus _cfw2_, you have information about the chest: Download the ".out.s" files linked to BLL (motor\-driver card) and RMHDC.  
- The net 9 is for the neck.  
- There are BLL and RMHDC drivers. Download all corresponding ".out.s" files, of which the name corresponds to the current version. 

# Calibration of the robot

**Joint calibration**

With a new iCub, or after modifying a component of the robot, it is required to calibrate the “zero position” associated to the joints of the robot. The robot should be fixed on the pole for calibration. After moving it to home position with _yarpmotorgui_, fine calibration can be done with the help of a level tool, as described in the [wiki](http://wiki.icub.org/wiki/Manual#Three._Calibration). First the torso must be calibrated, before proceeding to the head, arms and legs.

The following lines provide further information, as well as an example.  
In _yarpmotorgui_, switch to the tab associated to the body part you want to calibrate. Then, click the "_idle_" button for the joints to calibrate, allowing you to move them freely. With the help of a level tool, move the joint such that you can measure that it is level. Then click the "_run_" button for the considered joint, activating joint position control. Note the _encoder_ value.  
This value will be added in the specific file associated to the joint. Go to the folder containing the "\.xml" calibration files for your robot and open the corresponding file (example: `.../robots/$ROBOT_NAME/calibrators/left\_leg\_calib.xml`). Add the _encoder_ value that you have previously noted to the current "calibrationDelta" of the joint (each number of the parameter "calibrationDelta" corresponds to a joint). For example, we add some values to the line:  
`<param name="calibrationDelta">         -5.0        8.7        -11.4        -0.6        0.0        0.0       3.0        0.0        0.0        0.0        0.0        0.0        0.0        0.0        0.0        0.0        </param>`  

Thus, the source file shared between the load\_computer and the icub board (pc104 or icub-head) has been changed. Now, you have to update the icub board with the same values, for example with  
`ssh -X pc104`  
`cd $YARP_DIR/icub_main/build`  
`ccmake ../`  
`make`  

After restarting the robot, check that the joints were successfully calibrated.

**IMU calibration**

For IMU calibration, the robot has to be on the pole, in home position (i.e. all joints to 0).

You need to receive the acceleration values of the head to be able to calibrate it: you may launch (or create) a program which writes the acceleration data. Just for that, codyco-superbuild provides the Simulink file _calibrateIMU.mdl_ in `codyco-superbuild/main/wBIToolboxControllers/utilities`. When running _calibrateIMU.mdl_, the IMU scope shows the acceleration values, which need to be around  
- 0 for _x_ (yellow)  
- 0 for _y_ (blue)  
- 9.8 for _z_ (red)  
In other words, gravity has to be sensed only on the z axis.  

In _yarpmotorgui_, switch to the tab associated to the head. Then, click the "_idle_" button for the joints to calibrate: _neck_pitch_ and _neck_roll_ (_neck_yaw_ is usually not required), allowing you to move them freely. Then, move the head joints to bring signals are near their desired values. If the _x_ and _y_ positions have an error of less than 0.1, it is fine. Click the "_run_" button for the two joints. Note the _encoder_ values. 

If the head uses ETH, the _encoder_ values can be added in the corresponding joints in "calibrationDelta" of the head\-calib.xml file. (It may be useful to restart the robot and check again the calibration, since some small error may remain in the IMU values).

If the head uses CAN, the head\-calib.xml file would not be loaded properly. The configuration file for the head would instead be in another folder (for example `.../robots/$ROBOT_NAME/hardware/mechanicals`), and the parameter to modify would be _zero parameter_.

**Commit the changes**

At the end, if all is correct, commit the changes.  
_Remarks_:  
It is better to do many commits, rather than to commit everything at the end of the calibration procedure, in order to avoid errors.  
If you have a copy of the calibration configuration files in the .local folder, these files are the ones which will be loaded by _robotinterface_. In that case, when you do tests, edit the file in the .local folder. When you are sure of the calibration, modify the original calibration files to the new values and commit your changes.  


 ### move these parts to an installation tips and tricks page. 

# Things to know before downloading the project.
* Install only Gazebo from binaries. The other programs will have to be installed from sources (not binaries). Thanks to that, you can choose where they are installed, where the libraries are located, and you are able to update them easily.  
* To be able to clean up easily the whole project, it is recommended to avoid installing programs in the _/usr_ folder and to avoid installing libraries in the _/lib_ folder. To do that, follow these recommendations:  
1. Create a new folder (for example “software”) in your home directory and install all programs in this folder.  
2. When you download the source files of a program, a new folder is created. Go to this folder and create two directories called "build" and "install". Then, in the "build" folder, build the program using the command line `ccmake [_options_] ../`. You should have something like: `~/software/$yourProgram/build`. Moreover, all librairies of your programm have to be installed in the "install" folder that you should have created. To do that, add to the previous command line the option `\-DCMAKE\_INSTALL\_PREFIX=~/software/$yourProgram/install`).  
3. Normally, after the command line `make`, we write the command `make install`. Don't use this last command because we want to keep programs in the current directories without installing them in the "/usr" and "/lib" folders. Thanks to that, it is easier to clean up the programs. Due to this specification, programs are not accessible from other folders. 
4. Now, configure some variables to allow a program to be found from anywhere. These configurations are precised during the installation guide. For example you will be informed to add to the "~/.bashrc" file the command lines:    
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/software/$yourProgram/install/lib)`  
`export PATH=$PATH:~/software/$yourProgram/build`

# Launch the first iCub test
Once all is correctly updated and calibrated,  
1. Switch on the power supplies (wait a little for the power to come to the iCub).  
2. Switch on the CPU.  
3. It is recommended to create three terminals, called yarpserver/yarpmanager/yarpmotorgui to avoid making some mistakes between them.  
4. Go to the yarpserver terminal and launch "_icub\_cluster.py_" that launches yarpserver + yarprun (or, if you have it, launch the file that configures the cluster correctly. It should be called "launchApplicationGui.sh").  
5. Check that the main computer is correctly connected: correct the name of the computer if it is not correct, and click the "_verify_" button.  
Then, click “_check all_” for all computers that you want to connect. If one of them is not connected, select it and connect it.  
6. Now, you can switch on the motors.  
7. Launch yarpmanager and run, for example, the basic application “_icubstartup_”. For some applications that require to connect components, you have to click on “connect”.  
 
# Some little things to know 
* Be careful with the head and hands of the robot.  
* If you modify some source files on one computer of the network linked to the robot, these files will not be taken into account if their names are also in the ./local directory. Indeed, when you build a program, files are first read in the /local directory, then in the source folder.  
* How to hide DEBUG information: in the cmake information, put the variable DEBUG from DEBUG to RELEASE  
