#!/bin/bash

# Open a terminal on the device to install on
# Or ssh in if available

# Jetson-specific NEPI rootfs setup steps. This is a specialization of the base NEPI rootfs
# and calls that parent script as a pre-step.

# Run the parent script first
sudo ./nepi_file_syustem_setup.sh

############################################
# NEPI File System Setup (Container)
############################################


#########
# Define some system paths
# NOTE: THESE SHOULD HAVE BEEN Created in one of the ENV setup scripts above


REPO_DIR=${HOME_DIR}/nepi_engine
CONFIG_DIR=${REPO_DIR}/nepi_env/config
ETC_DIR=${REPO_DIR}/nepi_env/etc

NEPI_DIR=/opt/nepi
NEPI_RUI=${NEPI_DIR}/nepi_rui
NEPI_CONFIG=${NEPI_DIR}/config
NEPI_ENV=${NEPI_DIR}/ros
NEPI_ETC=${NEPI_DIR}/etc

NEPI_DRIVE=/mnt/nepi_storage






########
# 


sudo rm -R /opt/nepi/config
sudo cp -r /mnt/nepi_storage/tmp/nepi/config/ ./
sudo chown -R nepi:nepi /opt/nepi/config
sudo chown -R nepi:nepi /mnt/nepi_storage/tmp/nepi/config

sudo /opt/nepi/config/etc/license/setup_nepi_license.sh

######
# install ssh server
sudo apt-get install -y openssh-server
# Set up SSH
sudo mv /etc/ssh/sshd_config /etc/ssh/sshd_config.bak
sudo ln -sf /opt/nepi/config/etc/ssh/sshd_config /etc/ssh/sshd_config
# And link default public key - Make sure all ownership and permissions are as required by SSH
mkdir -p /home/nepi/.ssh
sudo chown nepi:nepi /home/nepi/.ssh
chmod 0700 /home/nepi/.ssh
sudo chown nepi:nepi /opt/nepi/config/home/nepi/ssh/authorized_keys
chmod 0600 /opt/nepi/config/home/nepi/ssh/authorized_keys
ln -sf /opt/nepi/config/home/nepi/ssh/authorized_keys /home/nepi/.ssh/authorized_keys
sudo chown nepi:nepi /home/nepi/.ssh/authorized_keys
chmod 0600 /home/nepi/.ssh/authorized_keys
sudo service ssh restart
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



##########
#____________
Clone the current container

"exit" twice


Clone container
sudo docker ps -a
Get <ID>
sudo docker commit <ID> nepi1

# Clean out <none> Images
sudo docker rmi $(sudo docker images -f “dangling=true” -q)

# Export/Import Flat Image as tar
sudo docker export a1e4e38c2162 > /mnt/nepi_storage/tmp/nepi3p0p4p1_jp5p0p2.tar
sudo docker import /mnt/nepi_storage/tmp/nepi3p0p4p1_jp5p0p2.tar 
docker tag <IMAGE_ID> nepi_ft1

# Save/Load Layered Image as tar
sudo docker save -o /mnt/nepi_storage/tmp/nepi1.tar a1e4e38c2162
sudo docker tag 7aa663d0a1e3 nepi_ft1

_________
#Clean the linux system
#https://askubuntu.com/questions/5980/how-do-i-free-up-disk-space
sudo apt-get clean
sudo apt-get autoclean
sudo apt-get autoremove


#_____________
Setup Host 

#(Recommended) Set your host ip address to nepi standard
Address: 192.168.179.103
Netmask: 255.255.255.0

#(Recommended) Setup dhcp service
apt install netplan.io

# Copy zed camera config files to 
/mnt/nepi_storage/usr_cfg/zed_cals/

#Install chromium on 
# On host machine open chromium and enter http://127.0.0.1:5003/ to access the RUI locally
# On 


#_____________
# setup nepi_storage folder

# Create a nepi_storage folder on mounted partition with at least 100 GB of free space
mkdir <path_to_nepi_parent_folder>/nepi_storage

# Run the nepi containers nepi_storage_init.sh script using the following command  
sudo docker run --rm --mount type=bind,source=/mnt/nepi_storage,target=/mnt/nepi_storage -it --net=host -v /tmp/.X11-unix/:/tmp/.X11-unix nepi /bin/bash -c "/nepi_storage_init.sh"

#then
exit


#_____________
# Run Nepi Engine
# Dev
sudo docker run --privileged -e UDEV=1 --user nepi --gpus all --mount type=bind,source=/mnt/nepi_storage,target=/mnt/nepi_storage --mount type=bind,source=/dev,target=/dev -it --net=host --runtime nvidia -v /tmp/.X11-unix/:/tmp/.X11-unix nepi1 /bin/bash

volumes - /dev:/dev

#Run