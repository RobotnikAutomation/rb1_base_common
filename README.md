rb1_base_common
===============
Common packages of the RB1 base: URDF description of the RB1 Base, platform messages and other files for simulation.

<p align="center">
  <img src="https://www.roscomponents.com/760-big_default/rb-1-base.jpg" width="275" />
  <img src="  https://www.roscomponents.com/761-thickbox_default/rb-1-base.jpg
" width="275" />
  <img src="https://www.roscomponents.com/759-thickbox_default/rb-1-base.jpg" width="275" /> 
</p>

<h1>Packages</h1>

<h2>rb1_base_description</h2>

The urdf, meshes, and other elements needed in the description are contained here. This package includes the description of the RB1 Base mobile platforms.
The package includes also some launch files to publish the robot state and to test the urdf files in rviz.

<h2>rb1_base_localization</h2>

Contains the configuration and launch files to use the robot localiztion packages along the real or simulated robot.

<h2>rb1_base_navigation</h2>

Contains the configuration and launch files to work with the ROS navigation stack along the real or simulated robot.

<h2>rb1_base_pad</h2>

This package contains the node that subscribes to /joy messages and publishes command messages for the robot platform including speed level control. The joystick output is feed to a mux (http://wiki.ros.org/twist_mux) so that the final command to the robot can be set by different components (move_base, etc.)

The node allows to load different types of joysticks (PS4, PS3, Logitech, Thrustmaster). New models can be easily added by creating new .yaml files.

<!-- For RB-1 instructions and tutorials, please see ... -->
