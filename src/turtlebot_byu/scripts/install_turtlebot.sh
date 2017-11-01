function install_turtlebot {

sudo apt-get install -y linux-headers-generic
sudo sh -c 'echo "deb-src http://us.archive.ubuntu.com/ubuntu/ xenial main restricted
deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-updates main restricted
deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-backports main restricted universe multiverse
deb-src http://security.ubuntu.com/ubuntu xenial-security main restricted" > \
      /etc/apt/sources.list.d/official-source-repositories.list'
sudo apt-get update
sudo apt-get install -y ros-kinetic-librealsense
sudo apt-get install -y ros-kinetic-librealsense-camera
sudo apt-get install -y ros-kinetic-turtlebot
sudo apt-get install -y ros-kinetic-turtlebot-apps
sudo apt-get install -y ros-kinetic-turtlebot-gazebo 
sudo apt-get install -y ros-kinetic-turtlebot-rviz-launchers  

}

install_turtlebot
