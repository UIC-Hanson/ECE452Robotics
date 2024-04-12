<b><i>UIC ECE 452 Spring 2024 - Professor Milos Zefran</b></i>

<b>Install script modified from SunFounder</b>

echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list

curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -

sudo apt update
sudo apt install network-manager network-manager-gnome python3 git python3-pip python3-setuptools python3-smbus keychain python3-pandas python3-tflite-runtime libatlas-base-dev libgtk2.0-dev pkg-config python3-pycoral -y
sudo pip3 install mediapipe mediapipe-rpi4 PyYAML opencv-contrib-python==4.5.5.64 opencv-python==4.5.5.64
sudo apt full-upgrade -y

sudo systemctl disable --now dhcpcd
sudo systemctl enable --now NetworkManager

sudo sudo systemctl restart NetworkManager
systemctl status NetworkManager

sudo raspi-config
#Enable VNC, I2C, update WiFi

sh <(curl -sSf https://downloads.nordcdn.com/apps/linux/install.sh)
sudo usermod -aG nordvpn $USER

sudo reboot now

nordvpn login --token XXXXXXXX

nordvpn set meshnet on
nordvpn meshnet set nickname ECE542Lab
nordvpn meshnet peer list

sudo reboot now

cd ~/
git clone -b v2.0 https://github.com/sunfounder/robot-hat.git
git clone -b picamera2 https://github.com/sunfounder/vilib.git
git clone -b v2.0 https://github.com/sunfounder/picar-x.git

cd ~/robot-hat
sudo python3 setup.py install

cd ~/vilib
sudo python3 install.py

cd ~/picar-x
sudo python3 setup.py install

cd ~/picar-x
sudo bash i2samp.sh -y


cd ~/picar-x/example
sudo python3 servo_zeroing.py

cd ~/picar-x/example/calibration
sudo python3 calibration.py

sudo python3 grayscale_calibration.py

mkdir .ssh cd ~/.ssh

ssh-keygen -t rsa -b 4096 -C "XXXXX@uic.edu"

chmod 600 *

eval `ssh-agent -s`

ssh-add 452SSH

nano ~/.bashrc
At the end add: eval $(keychain --eval --noask 452SSH)

nano ~/.bash_profile
eval `keychain --agents ssh --eval 452SSH`

cd ~/

git clone git@github.com:UIC-Hanson/ECE452Robotics.git

Add peers to meshnet https://meshnet.nordvpn.com/features/linking-devices-in-meshnet/adding-external-meshnet-devices-on-linux


<b>Project 02:</b>

Run robot_control.py, which will output data to ~/alpha_data.csv

Run alpha_estimation.py, which will average the values and create a table of alpha values by decade ~/alpha_estimation.csv

weighted_alpha.py is desiged for the function weighted_alpha_for_power() to be called from other functions to provide the alpha value for each speed.

Line_Tracking_2m depends on robot_control for get_power_level()
