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
   
# 1. Construction task folders
Controllers, worlds and simulations for 3 construction tasks: bolting, joint connection, and welding

## bolting_one_arm_CS225a: 
Construction project from CS225a Experimental Robotics Spring 2019-20. The team designed a mobile platform with a panda and a bolting tool to perform bolting in a parking structure. This folder can be used for reference for creating new 1 arm tasks.
  - Project report: https://docs.google.com/document/d/1BbVCC_011EWPjpUB3UtXwLP9la92q5Q-05Ds-F9n8mg/edit
  - Final project video: https://drive.google.com/file/d/1mXaT3YowrKBNImmpgqJyRvpyA87n6WvC/view
  
## bolting_two_arms: 
- Task description: 2 arm bolting task
- World: 2 fixed base Panda robot arms
   
## joint_autonomous:
- Task description: Find joints between concrete slabs and pour material to seal them together 
- World: the file world.urdf uses joint_robot.urdf which has a holonomic mobile base and a 1 dof tool
- Notes: Goldbeck has a preference for a robot design that's specific to this task instead of the regular Panda arm solution
- To run the code:  
  1) From the CIFEseed/build directory: cmake .. && make -j4 
  2) From any directory in a tab of its own: redis-server
  3) From CIFEseed/bin/joint: cd ../../bin/joint && ./simviz_joint  
  4) From CIFEseed/bin/joint in another tab: cd ../../bin/joint && ./controller_joint

## joint_haptics:
- Haptic feature: the user can intervene through haptic teleoperation (which allows them to feel the joint) and when they are done, resume and continue the autonomous task.

## weld_haptics: 
- Task description: Find the perimiter of a rectangular steel anchor, find and weld two corners, then weld the entire perimiter and move to the next anchor
- World: the file world.urdf uses mmp_panda_weld.urdf which is a mobile base (visuals are a small cart), with a panda robot, and a welding tool end-effector
- Notes: combines autonomous navitation with haptic exploration of the steel anchor
    
# 2. Graphical User Interfaces
Uses sai2-interfaces to modify controllers in a more intuitive, visual way: https://github.com/manips-sai-org/sai2-interfaces
## bolting_two_arms_gui: 
- work in progress to make a GUI for 2 arms for the 2 arm bolting task
- needs debugging
## weld_gui: 
- GUI that allows you to control one arm on a mobile base 

# 3. task_starter_code
Folder with potentially useful models (like the kuka_iiwa urdf), worlds, and controller examples for creating new tasks

 
