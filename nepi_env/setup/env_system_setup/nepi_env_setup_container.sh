#!/bin/bash


############################################
# PRE FILE SYSTEM SETUP (Container File System Only)
############################################
# DO THIS BEFORE FILE SYSTEM SETUP

#########
# Define some system paths

HOME_DIR=$PWD
REPO_DIR=${HOME_DIR}/nepi_engine
CONFIG_DIR=${REPO_DIR}/nepi_env/config
ETC_DIR=${REPO_DIR}/nepi_env/etc

NEPI_DIR=/opt/nepi
NEPI_RUI=${NEPI_DIR}/nepi_rui
NEPI_CONFIG=${NEPI_DIR}/config
NEPI_ENV=${NEPI_DIR}/ros
NEPI_ETC=${NEPI_ENV}/config

NEPI_DRIVE=/mnt/nepi_storage


#########
# Env Setup
#########
# Preliminary checks
# Internet connectivity:

if ! ping -c 2 google.com; then
    echo "ERROR: System must have internet connection to proceed"
    exit 1
fi


#########
# Add nepi user and group if does not exist
group="nepi"
user="nepi"
read -p "enter group name: " group
if grep -q $group /etc/group
  then
       echo "group exists"
else
       echo "group $group does not exist, creating"
       addgroup nepi
fi


if id -u "$user" >/dev/null 2>&1; then
  echo "User $user exists."
else
  echo "User $user does not exist, creating"
  adduser --ingroup nepi nepi
  echo "nepi ALL=(ALL:ALL) ALL" >> /etc/sudoers

  su nepi
  passwd
  nepi
  nepi
fi
exit


# Add nepi user to dialout group to allow non-sudo serial connections
sudo adduser nepi dialout

#or //https://arduino.stackexchange.com/questions/74714/arduino-dev-ttyusb0-permission-denied-even-when-user-added-to-group-dialout-o
#Add your standard user to the group "dialout'
sudo usermod -a -G dialout nepi
#Add your standard user to the group "tty"
sudo usermod -a -G tty nepi


# Clear the Desktop
sudo rm /home/nepi/Desktop/*


# Fix gpu accessability
#https://forums.developer.nvidia.com/t/nvrmmeminitnvmap-failed-with-permission-denied/270716/10
sudo usermod -aG sudo,video,i2c nepi


# Install some required applications
sudo apt update
sudo apt install git -y
git --version



#############
# Install NEPI Config files

# Clone the nepi_engine repo
cd ~/
git clone https://github.com/nepi-engine/nepi_engine.git




sudo mkdir -p ${NEPI_DIR}
sudo mkdir -p ${NEPI_RUI}
sudo mkdir -p ${NEPI_CONFIG}
sudo mkdir -p ${NEPI_ENV}
sudo mkdir -p ${NEPI_ETC}

sudo cp ${CONFIG_DIR}/* ${NEPI_CONFIG}
sudo chown -R nepi:nepi ${NEPI_DIR}

# Install low-level drivers
sudo cp ${CONFIG_DIR}/lib/wifi_drivers/* /lib/firmware

# Update the Desktop background image
echo "Updating Desktop background image"
gsettings set org.gnome.desktop.background picture-uri file:///${CONFIG_DIR}/home/nepi/nepi_wallpaper.png

# Update the login screen background image - handled by a sys. config file
# No longer works as of Ubuntu 20.04 -- there are some Github scripts that could replace this -- change-gdb-background
#echo "Updating login screen background image"
#sudo mv /usr/share/gnome-shell/theme/ubuntu.css /usr/share/gnome-shell/theme/ubuntu.css.bak
#sudo ln -sf ${CONFIG_DIR}/usr/share/gnome-shell/theme/ubuntu.css /usr/share/gnome-shell/theme/ubuntu.css

# Set up static IP addr.
sudo mv ${CONFIG_DIR}/etc/network/interfaces.d /etc/network/interfaces.d.bak
sudo ln -sf ${CONFIG_DIR}/etc/network/interfaces.d /etc/network/interfaces.d

# Set up DHCP
sudo mv ${CONFIG_DIR}/etc/dhcp/dhclient.conf /etc/dhcp/dhclient.conf.bak
sudo ln -sf ${CONFIG_DIR}/etc/dhcp/dhclient.conf /etc/dhcp/dhclient.conf
sudo dhclient


# Set up SSH
sudo mv ${CONFIG_DIR}/etc/ssh/sshd_config /etc/ssh/sshd_config.bak
sudo ln -sf ${CONFIG_DIR}/etc/ssh/sshd_config /etc/ssh/sshd_config
# And link default public key - Make sure all ownership and permissions are as required by SSH
mkdir -p /home/nepi/.ssh
sudo chown nepi:nepi /home/nepi/.ssh
chmod 0700 /home/nepi/.ssh
sudo chown nepi:nepi ${CONFIG_DIR}/home/nepi/ssh/authorized_keys
chmod 0600 ${CONFIG_DIR}/home/nepi/ssh/authorized_keys
ln -sf ${CONFIG_DIR}/home/nepi/ssh/authorized_keys /home/nepi/.ssh/authorized_keys
sudo chown nepi:nepi /home/nepi/.ssh/authorized_keys
chmod 0600 /home/nepi/.ssh/authorized_keys

# Set up some udev rules for plug-and-play hardware
  # IQR Pan/Tilt
sudo ln -sf ${CONFIG_DIR}/etc/udev/rules.d/56-iqr-pan-tilt.rules /etc/udev/rules.d/56-iqr-pan-tilt.rules
  # USB Power Saving on Cameras Disabled
sudo ln -sf ${CONFIG_DIR}/etc/udev/rules.d/92-usb-input-no-powersave.rules /etc/udev/rules.d/92-usb-input-no-powersave.rules

# Disable apport to avoid crash reports on a display
sudo systemctl disable apport

# Now start installing stuff... first, update all base packages
sudo apt update
sudo apt upgrade

# Install static IP tools
echo "Installing static IP dependencies"
sudo apt install ifupdown net-tools

# Convenience applications
sudo apt install nano

# Install and configure chrony
echo "Installing chrony for NTP services"
sudo apt install chrony
sudo mv ${CONFIG_DIR}//chrony/chrony.conf /etc/chrony/chrony.conf.bak
sudo ln -sf ${CONFIG_DIR}/etc/chrony/chrony.conf.num_factory /etc/chrony/chrony.conf


################################
# Install and configure samba with default passwords

echo "Installing samba for network shared drives"
sudo apt install samba
sudo mv ${CONFIG_DIR}//samba/smb.conf /etc/samba/smb.conf.bak
sudo ln -sf ${CONFIG_DIR}/etc/samba/smb.conf /etc/samba/smb.conf
printf "nepi\nepi\n" | sudo smbpasswd -a nepi

#Create nepi_storage folder
sudo mkdir ${NEPI_DRIVE}

# Create the mountpoint for samba shares (now that sambashare group exists)
sudo chown -R nepi:sambashare ${NEPI_DRIVE}
sudo chmod -R 0775 ${NEPI_DRIVE}



##############
# Install Baumer GenTL Producers (Genicam support)
echo "Installing Baumer GAPI SDK GenTL Producers"
# Set up the shared object links in case they weren't copied properly when this repo was moved to target
NEPI_BAUMER_PATH=${CONFIG_DIR}/opt/baumer/gentl_producers
ln -sf $NEPI_BAUMER_PATH/libbgapi2_usb.cti.2.14.1 $NEPI_BAUMER_PATH/libbgapi2_usb.cti.2.14
ln -sf $NEPI_BAUMER_PATH/libbgapi2_usb.cti.2.14 $NEPI_BAUMER_PATH/libbgapi2_usb.cti
ln -sf $NEPI_BAUMER_PATH/libbgapi2_gige.cti.2.14.1 $NEPI_BAUMER_PATH/libbgapi2_gige.cti.2.14
ln -sf $NEPI_BAUMER_PATH/libbgapi2_gige.cti.2.14 $NEPI_BAUMER_PATH/libbgapi2_gige.cti
# And the master link
sudo ln -sf ${CONFIG_DIR}/opt/baumer /opt/baumer
sudo chown nepi:nepi /opt/baumer




#################################################################
#Start service at runtime in docker file
https://stackoverflow.com/questions/25135897/how-to-automatically-start-a-service-when-running-a-docker-container
In your Dockerfile, add at the last line

ENTRYPOINT service nepi_rui.service restart && bash

# The normal method doesn't work doesn't work?
# After testing add startup service
sudo cp /opt/nepi/nepi_rui/etc/nepi_rui.service /etc/systemd/system
sudo chmod +x /etc/systemd/system/nepi_rui.service

#Once you have a unit file, you are ready to test the service:

sudo systemctl start nepi_rui.service
#Check the status of the service:

sudo systemctl status nepi_rui.service


#______________
# copy startup scripts
# Install nepi start scripts in root folder


sudo cp /opt/nepi/config/etc/supervisord/nepi_start_all.sh /
sudo chmod +x /nepi_start_all.sh

sudo cp /mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_edge_sdk_base/etc/nepi_engine_start.sh /
sudo chmod +x /nepi_engine_start.sh

sudo cp /mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_rui/etc/nepi_rui_start.sh /
sudo chmod +x /nepi_rui_start.sh

sudo cp /opt/nepi/config/etc/samba/nepi_storage_samba_start.sh /
sudo chmod +x /nepi_storage_samba_start.sh

sudo cp /opt/nepi/config/etc/storage/nepi_storage_init.sh /
sudo chmod +x /nepi_storage_init.sh

sudo cp /opt/nepi/config/etc/license/nepi_check_license_start.sh /
sudo dos2unix /opt/nepi/config/etc/license/nepi_check_license.py
sudo chmod +x /nepi_check_license_start.sh

sudo cp /opt/nepi/config/etc/storage/nepi_storage_init.sh /
sudo chmod +x /nepi_storage_init.sh




#-------------------
# Install setup supervisord
#https://www.digitalocean.com/community/tutorials/how-to-install-and-manage-supervisor-on-ubuntu-and-debian-vps
#https://test-dockerrr.readthedocs.io/en/latest/admin/using_supervisord/

sudo apt update && sudo apt install supervisor
sudo vi /etc/supervisor/conf.d/nepi.conf
# Add these lines
[supervisord]
nodaemon=false

[program:nepi_engine]
command=/bin/bash /nepi_engine_start.sh
autostart=true
autorestart=true


[program:nepi_rui]
command=/bin/bash /nepi_rui_start.sh
autostart=true
autorestart=true

[program:nepi_storage_samba]
command=/bin/bash /nepi_storage_samba_start.sh
autostart=true
autorestart=true


###

sudo /usr/bin/supervisord


