#!/bin/bash
#Данный bush-скрипт с помощью пакета ROS "turtlesim" выводит табельный номер 243984, повторяя шрифт найденного
#на просторах сети Интернет примера (файл font_template.jpg в текущей директории)

#Очищаем поле от старых воспроизведений программы
rosservice call /reset
rosservice call /kill turtle1

#Рисуем цифры, при этом задавая размер и цвет линии

#Цифра 2    
rosservice call /spawn 0.2 8.0 0.0 turtle1
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[3.0, 0.0, 0.0]' '[0.0, 0.0, -3.1]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rosservice call /turtle1/set_pen 0 0 0 2 1
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -2.4]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[1.9, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -2.8]'
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[3.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 2.1]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[1.4, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rosservice call /kill turtle1
#Цифра 4    
rosservice call /spawn 3.4 6.2 3.15 turtle1
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -2.2]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -2.5]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[3.15, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -1.5]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[1.5, 0.0, 0.0]' '[0.0, 0.0, -1.5]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[-3.0, 0.0, 0.0]' '[0.0, 0.0, 2.9]'
rosservice call /kill turtle1
#Цифра 3 
rosservice call /spawn 3.4 8.0 0.0 turtle1
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[3.0, 0.0, 0.0]' '[0.0, 0.0, -3.1]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rosservice call /turtle1/set_pen 0 0 0 2 1
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -1.6]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -1.57]'
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[3.0, 0.0, 0.0]' '[0.0, 0.0, -3.1]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rosservice call /kill turtle1
#Цифра 9 
rosservice call /spawn 5.95 8.0 0.0 turtle1
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[6.0, 0.0, 0.0]' '[0.0, 0.0, -6.1]'
rosservice call /turtle1/set_pen 0 0 0 2 1
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -0.7]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[1.2, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -1.4]'
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[3.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rosservice call /kill turtle1
#Цифра 8
rosservice call /spawn 8.0 8.0 0.0 turtle1
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[6.0, 0.0, 0.0]' '[0.0, 0.0, -6.1]'
rosservice call /turtle1/set_pen 0 0 0 2 1
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[3.1, 0.0, 0.0]' '[0.0, 0.0, -3.2]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -1.6]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -1.57]'
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[6.0, 0.0, 0.0]' '[0.0, 0.0, -6.1]'
rosservice call /kill turtle1
#Цифра 4 
rosservice call /spawn 10.9 6.2 3.15 turtle1
rosservice call /turtle1/set_pen 0 0 0 2 0
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -2.2]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -2.5]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[3.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, -1.5]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[1.5, 0.0, 0.0]' '[0.0, 0.0, -1.5]'
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[-3.0, 0.0, 0.0]' '[0.0, 0.0, 2.9]'
rosservice call /kill turtle1
