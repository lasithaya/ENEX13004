# Create Catkin Workspace
> In this exercise, we will create a ROS catkin workspace.


### Create a Catkin Workspace

1. Create the root workspace directory (we'll use `catkin_ws`)

   ```
   cd ~/
   mkdir --parents catkin_ws/src
   cd catkin_ws
   ```

1. Initialize the catkin workspace

   ```
   catkin init
   ```
   * _Look for the statement "Workspace configuration appears valid", showing that your catkin workspace was created successfully.  If you forgot to create the `src` directory, or did not run `catkin init` from the workspace root (both common mistakes), you'll get an error message like "WARNING: Source space does not yet exist"._

1. Build the workspace. This command may be issued anywhere under the workspace root-directory (i.e. `catkin_ws`).

   ```
   catkin make
   ls
   ```

   * _See that the `catkin_ws` directory now contains additional directories (build, devel, logs)._
   
1. These new directories can be safely deleted at any time (either manually, or using `catkin clean`).  Note that catkin never changes any files in the `src` directory.  Re-run `catkin build` to re-create the build/devel/logs directories.

   ```
   catkin clean
   ls
   catkin make
   ls
   ```

1. Make the workspace visible to ROS. Source the setup file in the devel directory.

   ```
   source devel/setup.bash
   ```

   * _This file MUST be sourced for every new terminal._
   * To save typing, add this to your `~/.bashrc` file, so it is automatically sourced for each new terminal:

     1. `gedit ~/.bashrc`
     1. add to the end: `source ~/catkin_ws/devel/setup.bash`
     1. save and close the editor
