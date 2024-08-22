colcon build 
source install/setup.bash
ros2 run eai_pkg self_learning_node

# layout={"orange_block":(150,250,0), "purple_block":(250,350,0)}, actions={grab('purple_block',150,250,0),place('purple_block',230,370,0)}