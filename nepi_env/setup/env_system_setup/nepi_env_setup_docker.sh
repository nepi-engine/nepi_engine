#!/bin/bash


############################################
# PRE FILE SYSTEM SETUP (docker File System Only)
############################################
# DO THIS BEFORE FILE SYSTEM SETUP


# Set up the NEPI docker ROOTFS (Typically on External Media (e.g SD, SSD, SATA))

# This script is tested to run from a fresh Ubuntu 18.04 install based on the L4T reference rootfs.
# Other base rootfs schemes may work, but should be tested.

# Run this script from anywhere on the device

# This is a specialization of the base NEPI rootfs
# and calls that parent script as a pre-step.

# Run the parent script first
sudo ./nepi_env_setup.sh


#########
# Define some system paths

HOME_DIR=/home/nepi
REPO_DIR=${HOME_DIR}/nepi_engine
CONFIG_DIR=${REPO_DIR}/nepi_env/config
ETC_DIR=${REPO_DIR}/nepi_env/etc

NEPI_DIR=/opt/nepi
NEPI_RUI=${NEPI_DIR}/nepi_rui
NEPI_CONFIG=${NEPI_DIR}/config
NEPI_ENV=${NEPI_DIR}/ros
NEPI_ETC=${NEPI_DIR}/etc

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


' ' This doesn't work
#################################################################
#Start service at runtime in docker file
https://stackoverflow.com/questions/25135897/how-to-automatically-start-a-service-when-running-a-docker-docker
In your Dockerfile, add at the last line

ENTRYPOINT service nepi_rui.service restart && bash

# The normal method doesn't work doesn't work?
# After testing add startup service
sudo cp ${NEPI_RUI}/etc/nepi_rui.service /etc/systemd/system
sudo chmod +x /etc/systemd/system/nepi_rui.service

#Once you have a unit file, you are ready to test the service:

sudo systemctl start nepi_rui.service
#Check the status of the service:

sudo systemctl status nepi_rui.service
'

#______________
# copy startup scripts
# Install nepi start scripts in root folder


sudo cp ${NEPI_CONFIG}/etc/supervisord/nepi_start_all.sh ${NEPI_ETC}/nepi_start_all.sh
sudo chmod +x ${NEPI_ETC}/nepi_start_all.sh
sudo ln -sf ${NEPI_ETC}/nepi_start_all.sh /nepi_start_all.sh

sudo cp ${NEPI_CONFIG}/etc/docker/nepi_engine_start.sh ${NEPI_ETC}/nepi_engine_start.sh
sudo chmod +x ${NEPI_ETC}/nepi_engine_start.sh
sudo ln -sf ${NEPI_ETC}/nepi_engine_start.sh /nepi_engine_start.sh

sudo cp ${NEPI_CONFIG}/etc/docker/nepi_rui_start.sh ${NEPI_ETC}/nepi_rui_start.sh
sudo chmod +x ${NEPI_ETC}/nepi_rui_start.sh
sudo ln -sf ${NEPI_ETC}/nepi_rui_start.sh /nepi_rui_start.sh

sudo cp ${NEPI_CONFIG}/etc/docker/nepi_storage_samba_start.sh ${NEPI_ETC}/nepi_storage_samba_start.sh
sudo chmod +x ${NEPI_ETC}/nepi_storage_samba_start.sh
sudo ln -sf ${NEPI_ETC}/nepi_storage_samba_start.sh /nepi_storage_samba_start.sh

sudo cp ${NEPI_CONFIG}/etc/docker/nepi_storage_init.sh ${NEPI_ETC}/nepi_storage_init.sh
sudo chmod +x ${NEPI_ETC}/nepi_storage_init.sh
sudo ln -sf ${NEPI_ETC}/nepi_storage_init.sh /nepi_storage_init.sh

sudo cp ${NEPI_CONFIG}/etc/docker/nepi_check_license_start.sh ${NEPI_ETC}/nepi_check_license_start.sh
sudo chmod +x ${NEPI_ETC}/nepi_check_license_start.sh
sudo ln -sf ${NEPI_ETC}/nepi_check_license_start.sh /nepi_check_license_start.sh

sudo chown -R nepi:nepi /opt/nepi/etc
cd $NEPI_DIR
sudo cp -R etc etc.factoryls

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


