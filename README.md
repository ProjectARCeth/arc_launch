# arc_launch

Use following commands on the terminal before building the package:
wstool merge
wstool merge arc_launch/dependencies.rosinstall 
wstool update

For velodyne:
sudo apt-get install libpcap-dev

For arc_viewer:
##install qt
sudo apt-get install qt5-default
wget http://download.qt.io/official_releases/qt/5.7/5.7.0/qt-opensource-linux-x64-5.7.0.run
chmod +x qt-opensource-linux-x64-5.7.0.run
./qt-opensource-linux-x64-5.7.0.run

For ps3-driver:
sudo apt-get install libusb-dev
