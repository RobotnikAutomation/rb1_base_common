For Gmapping:

1-Check correct location of laser - in some positions it can interfere with the
camera or the wheels.

2-Check correct orientation of the laser (up / down)

3-Check that the coherence of the laser range:
3.1 Check that the parameters in the urdf file match the real laser
the parameter maxRange of the urdf file 
    <maxRange>30.0</maxRange>
must be > than the maxUrange of the gmapping node param.
    <param name="maxUrange" value="16.0"/>

Simple test:

>roslaunch rb1_base_sim_bringup rb1_base_complete.launch

>roslaunch rb1_base_navigation move_base_amcl.launch id_robot:=rb1_base_a
