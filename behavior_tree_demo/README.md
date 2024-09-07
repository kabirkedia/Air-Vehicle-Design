# Behavior Tree Demo

## Running
The easiest way to run the code is to use docker, however, the code can also be run on any Ubuntu 22.04 installation with ROS2 Humble Desktop installed.

```
git clone git@bitbucket.org:castacks/behavior_tree_demo.git
cd behavior_tree_demo
./docker_run.sh
ros2 launch behavior_tree_example behavior_tree_example.xml
```

## Modifying the demo

The config file for the behavior tree is in `workspace/src/behavior_tree_example/config/demo_base.tree`.

You can edit this from outside docker and it will be reflected inside.

If you want to create a new config file you can place it in the `workspace/src/behavior_tree_example/config/` folder and modify the config parameter in `workspace/src/behavior_tree_example/launch/behavior_tree_example.xml` but you will have to rebuild with `colcon build --symlink-install` for the change to be reflected.

## Lecture and Exercise Slides for Behavior Trees

https://docs.google.com/presentation/d/1cggQA8HyMsL1Jx7vrRdrmhpls3RMNXKjXDaYVfQnqFY/edit#slide=id.p
