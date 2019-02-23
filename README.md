# src
# sudo -H apt-get install -y ros-melodic-geographic-msgs
# sudo -H apt-get install -y libgeographic-dev
# sudo -H apt-get install -y geographiclib-tools
# sudo -H apt-get install -y python-future

## requirement
安装G2o
安装Ceres
opencv ros 会自带不用安装

git clone https://github.com/wjwwood/serial.git
cd serial/
mkdir build
cmake ..
make
sudo make install

