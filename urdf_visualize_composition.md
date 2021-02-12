## Urdf_visualize composition

    <param name="robot_description" command="cat $(arg model)" />
Here it is loading the URDF file to the param server variable called "robot_description." Bear in mind that if you are loading more than one robot, you will have to load them in different variables, like "robot1_description" and "robot2_description."

```xml
<!-- send fake joint values -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="TRUE"/>
  </node>

  <!-- Combine joint values -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
```
Start the jointstate publisher and the robotstate publisher. These will publish the TFs(tranformations) of the URDF of the robot links and joints.
To know more about how this works, please go to [TF-ROS course](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/tf-ros-101/).
```xml
<!-- Show in Rviz   -->
  <!--<node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_mira_description)/rviz_config/urdf.rviz"/>-->
  <node name="rviz" pkg="rviz" type="rviz" args=""/>
```
Run RVIZ. The part about loading your own RVIZ is commented. The first time you launch this, just save the RVIZ config file and then you will have all that is needed.