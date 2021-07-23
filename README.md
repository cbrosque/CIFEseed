# CIFEseed
This repo will be used to explore haptics for Human-Robot Collaboration in Construction
  - For running the tasks (from the main directory): 
      ```
      mkdir build
      cd build && cmake .. && make -j4
      ```
      Then to run the executables go to the bin directory that will be generated
     
  - For running the GUI (from the gui-construction directory): 
      ```
      python3 interface/servery.py ../construction/gui-construction.html & SERVER_PID=$!
      ```
      Remember to first run ./simviz_gui and ./controller_gui from the bin/gui-construction directory
      
   - You can find/add new tools, worlds and robot designs as urdf files in the "models" folder
   
# 1. Construction Tasks 
Controllers, worlds and simulations for 3 construction tasks: bolting, joint connection, and welding

## bolting_one_arm_CS225a: 
Construction project from CS225a Experimental Robotics Spring 2019-20. The team designed a mobile platform with a panda and a bolting tool to perform bolting in a parking structure. This folder can be used for reference for creating new 1 arm tasks.
  - Project report: https://tinyurl.com/cs225aReport
  - Final project video: https://tinyurl.com/cs225aVideo
  
## bolting_two_arms: 
- Task description: 2 arm bolting task
- World: 2 fixed base Panda robot arms
   
## joint_autonomous:
- Demo video: 
- Task description: Find joints between concrete slabs and pour material to seal them together 
- World: the file world.urdf uses joint_robot.urdf which has a holonomic mobile base and a 1 dof tool
- Notes: Goldbeck has a preference for a robot design that's specific to this task instead of the regular Panda arm solution
- To run the code (manually):  
  1) From the CIFEseed/build directory: ```cmake .. && make -j4 ```
  2) From any directory in a tab of its own: ```redis-server```
  3) From CIFEseed/bin/joint_autonomous: ```cd ../../bin/joint && ./simviz_joint  ```
  4) From CIFEseed/bin/joint_autonomous in another tab: ```cd ../../bin/joint && ./controller_joint```
- Alternatively, you can run the code using the clean_build and launch scripts: 
  1) From CIFEseed: ```./clean_build```
  2) From any directory in a tab of its own: ```redis-server ```
  3) From CIFEseed/bin/joint_autonomous: ```./launch```

## joint_haptics:
- Demo video: 
- Haptic feature: the user can intervene through haptic teleoperation (which allows them to feel the joint) and when they are done, resume and continue the autonomous task.
- To run the code:  
  1) From the CIFEseed/build directory: ```cmake .. && make -j4 ```
  2) From any directory in a tab of its own: ```redis-server```
  3) From CIFEseed/bin/joint_haptics: ```cd ../../bin/joint_haptics && ./simviz_joint_haptics ``` 
  4) From CIFEseed/bin/joint_haptics in another tab: ```cd ../../bin/joint_haptics && ./controller_joint_haptics```
  5) Launch redis client in another tab: ```redis-cli```; to set haptics: ```set active_state HAPTICS```; to continue autonomous state: ```set active_state RESUME```


## weld_haptics: 
- Demo video: https://tinyurl.com/weldingHaptics
- Task description: Autonomously navigate towards the perimiter of a rectangular steel anchor plate, allow for haptic exploration of the plate's perimeter, then weld and move to the next anchor
- World: the file world.urdf uses mmp_panda_weld.urdf which is a mobile base (visuals are a small cart), with a panda robot, and a welding tool end-effector
- Notes: combines autonomous navitation with haptic exploration of the steel anchor
- To run the code:  
  1) From the CIFEseed/build directory: ```cmake .. && make -j4 ```
  2) From any directory in a tab of its own: ```redis-server```
  3) From CIFEseed/bin/weld_haptics: ```cd ../../bin/weld_haptics && ./simviz_weld  ```
  4) From CIFEseed/bin/weld_haptics in another tab: ```cd ../../bin/weld_haptics && ./controller_weld```
  5) Launch redis client in another tab: ```redis-cli```; to set haptics: ```set active_state HAPTICS```; to continue autonomous state lift the haptic device
    
# 2. Graphical User Interfaces
Uses sai2-interfaces to modify controllers in a more intuitive, visual way: https://github.com/manips-sai-org/sai2-interfaces
## bolting_two_arms_gui: 
- work in progress to make a GUI for 2 arms for the 2 arm bolting task
- needs debugging
## weld_gui: 
- GUI that allows you to control one arm on a mobile base 

# 3. task_starter_code
Folder with potentially useful models (like the kuka_iiwa urdf), worlds, and controller examples for creating new tasks

 
