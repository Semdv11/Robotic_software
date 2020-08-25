#!/usr/bin/env python
#Данный скрипт реализовывает идею создания системы автоматического исследования окружающей местности роботом.
#Ключевой момент программы начинается со строчки 81.

#Подключение нужных библиотек
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid 
from move_base_msgs.msg import MoveBaseActionGoal
#from sensor_msgs.msg import LaserScan
import threading # Needed for Timer
from std_msgs.msg import String

PI = 3.1415926535897

class GetGoal:

    #Initial pose
    pose_stamped=PoseWithCovarianceStamped()
    pose_stamped.pose.pose.position.x=5.0
    pose_stamped.pose.pose.position.y=5.0
    pose_stamped.pose.pose.orientation.z=0
    
    #Настройка карты
    map=OccupancyGrid()
    map_array=[[i for i in range(4000)],[j for j in range(4000)]]
    #map_array=[[],[]]
	
    Goal_is_Obtained = False

    #Подключение и создание нужных нод. Объявление подписчиков и паблишеров.
    def __init__(self):

        # Creates a node with
        rospy.init_node('GetGoal', anonymous=True)

        # Publisher which will publish to the topic 'Goal'.
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

        # A subscriber to the topics 'cmd_vel'.
	#self.velocity_subscriber=rospy.Subscriber("/cmd_vel",Twist, self.timer_callback) # When receiving a message, call timer_callback()
	self.velocity_subscriber=rospy.Subscriber("/cmd_vel",Twist)
 
	# A subscriber to the topics '/amcl_Pose'
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)

	# A subscriber to the topics '/map'
        self.pose_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.update_map)
	
    #Обеспечение обратной связи скрипта.
    def timer_callback(self, msg):
  	global timer
	print("Velocity message received")
	timer.cancel()
	timer = threading.Timer(2,self.PublishGoal) #2 seconds elapse
	timer.start()
	return

    def update_pose(self, msg):
	print("Map message recieved")
	self.pose_stamped=msg
	#self.pose_stamped.pose.pose.position.x
	#self.pose_stamped.pose.pose.position.y
	#self.pose_stamped.pose.pose.orientation.z

    def update_map(self, msg):
	print("Map message recieved")
	self.map=msg
	if not self.map.data: print("Map data is empty"); return
	if self.map.info.width == 640: return
	w=self.map.info.width
	h=self.map.info.height
	for j in range(h): #Перевод одномерного массива карты в двумерный.
		for i in range(w):
			try:
				self.map_array[i][j]=self.map.data[j*w+i]
			except IndexError:
				#print(1)
				print(i,j,self.map_array[i][j],self.map.data[j*w+i]) 
	
	#Задание цели для робота
        self.PublishGoal

    def PublishGoal(self):
	x=self.pose_stamped.pose.pose.position.x
	y=self.pose_stamped.pose.pose.position.y
	theta=self.pose_stamped.pose.pose.orientation.z*PI
	#res=self.map.info.resolution
	res=0.05	

	i=3
	sum=0
	self.Goal_is_Obtained = True
	xc=int(x/res)-1 #x-check coordinate
	yc=int(y/res)-1
	#map analysys
	
	#Данная часть кода задает перебор и проверку элементов (пикселей) карты в массиве, который формируется при ее исследовании
	#на наличие отрицательного значения, так как -1 присваивается элементарной единице карты, которая еще не исследована
	#(0 и 1 - для статуса "пусто" и "стена/объект соответственно). Перебор осуществляется по принципу сравнения сумм значений, присваемыевых
	#каждому пикселю карты: берется сумма значений вокруг пикселей, находящихся по дигонали, по вертикальной и горизонтальной осям относительно рассматриваеого
	#пикселя. Если сумма меньше, чем раннее записанное значение, то в данной области присутствует большей неизвестных элементов карты, рассматриваемая область
	#(пиксель) назначается новой целью, и робот едет в ее сторону.
	
	while self.Goal_is_Obtained: #checks 8 sqrs around current position 
		try:
			if self.check_sqr(xc+i, yc) < sum: sum=self.check_sqr(xc+i , yc); xg=xc+i; yg=yc;
			if self.check_sqr(xc-i, yc) < sum: sum=self.check_sqr(xc-i , yc); xg=xc-i; yg=yc;
			if self.check_sqr(xc, yc+i) < sum: sum=self.check_sqr(xc , yc+i); xg=xc; yg=yc+i;
			if self.check_sqr(xc, yc-i) < sum: sum=self.check_sqr(xc+i , yc-i); xg=xc; yg=yc-i;
			if self.check_sqr(xc+i, yc+i) < sum: sum=self.check_sqr(xc+i , yc+i); xg=xc+i; yg=yc+i;
			if self.check_sqr(xc+i, yc-i) < sum: sum=self.check_sqr(xc+i , yc-i); xg=xc+i; yg=yc-i;
			if self.check_sqr(xc-i , yc+i) < sum: sum=self.check_sqr(xc-i , yc+i); xg=xc-i; yg=yc+i;
			if self.check_sqr(xc-i , yc-i) < sum: sum=self.check_sqr(xc-i , yc-i); xg=xc-i; yg=yc-i;
			if sum < 0: self.Goal_is_Obtained = False 
			else: i=i+1
		except IndexError:
			pass
	#publishing goal
	print("Publishing goal")
	pose_msg=MoveBaseActionGoal
	pose_msg.goal.pose.position.x = xg*res
	pose_msg.goal.pose.position.y = yg*res
	#print(xg*res, yg*res)
	self.goal_publisher.pubish(pose_msg)
	return
    #Задание изначального значения проверяемой суммы пикселей: она равна сумме значений всех пикселей вокруг исследуемого.
    def check_sqr(self,x,y): #asssumes sqr 3x3 is known or not summurazing coast map
	#self.map_array=self.map.data
	sum=self.map_array[x][y]+self.map_array[x+1][y]+self.map_array[x][y+1]+self.map_array[x-1][y]+self.map_array[x][y-1]+self.map_array[x+1][y+1]+self.map_array[x+1][y-1]+self.map_array[x-1][y+1]+self.map_array[x-1][y-1]
	return sum


if __name__ == '__main__':
    try:
	x = GetGoal()
	x.PublishGoal()
    except rospy.ROSInterruptException:
	pass
