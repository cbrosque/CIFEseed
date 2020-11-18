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

## bolting_CS225a: 
Construction project from CS225a Experimental Robotics Spring 2019-20. The team designed a mobile platform with a panda and a bolting tool to perform bolting in a parking structure. This folder can be used for reference for creating new 1 arm tasks.
  - Project report: https://docs.google.com/document/d/1BbVCC_011EWPjpUB3UtXwLP9la92q5Q-05Ds-F9n8mg/edit
  - Final project video: https://drive.google.com/file/d/1mXaT3YowrKBNImmpgqJyRvpyA87n6WvC/view
  
## bolting_two_arms: 
- Task description: 2 arm bolting task
- World: 2 fixed base Panda robot arms
   
## joint:
- Task description: Find joints between concrete slabs and pour material to seal them together 
- World: the file world2.urdf uses mmp_panda2.urdf which has a holonomic mobile base and the visuals of a tool, but no arm and no moving joints
- Notes: 
    - currently using: simviz2, controller3, world2
    - not yet working with a GUI. You have to modify controller3 directly.
    - Goldbeck has a preference for a robot design that's specific to this task instead of the regular Panda arm solution
- TODO: 
    - maybe add an option with a different world that has a panda arm and not just a tool
    - add tool mobility to the world2.urdf
    - change names of files
    - create GUI
 
## weld: 
- Task description: Find the perimiter of a rectangular steel anchor, find and weld two corners, then weld the entire perimiter and move to the next anchor
- World: the file world3.urdf uses mmp_panda3.urdf which is a mobile base (visuals are a small cart), with a panda robot, and a welding tool end-effector
- Notes: 
    - currently using: simviz3, controller4, world3
    - works with gui_weld
- TODO: 
    - debug controller4 so that it makes the robot go to the coordinates selected using the GUI
    - change names of files
    
# 2. Graphical User Interfaces
Uses sai2-interfaces to modify controllers in a more intuitive, visual way: https://github.com/manips-sai-org/sai2-interfaces
## gui_two_arms: 
- work in progress to make a GUI for 2 arms for the 2 arm bolting task
- not currently working 
## gui_weld: 
- GUI that allows you to control one arm on a mobile base 
- used for the welding task

# 3. project_starter_code
Folder with potentially useful models (like the kuka_iiwa urdf), worlds, and controller examples for creating new tasks

 
