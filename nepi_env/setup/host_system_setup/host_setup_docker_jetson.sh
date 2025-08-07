
#!/bin/bash



######################################################################
### Some Tools
# sudo docker images -a
# sudo docker ps -a
# sudo docker start  `nepi_test ps -q -l` # restart it in the background
# sudo docker attach `nepi_test ps -q -l` # reattach the terminal & stdin
## https://phoenixnap.com/kb/how-to-commit-changes-to-docker-image
######################################################################




https://developer.nvidia.com/embedded/jetson-cloud-native
https://gitlab.com/nvidia/container-images/l4t-jetpack

Resize image
sudo resize2fs /dev/nvme0n1p2
sudo resize2fs /dev/nvme0n1p3

On PC
git clone https://gitlab.com/nvidia/container-images/l4t-jetpack

Copy /home/engineering/Code/l4t-jetpack folder to nepi_storage/tmp

## CONNECT NEPI DEVICE TO INTERNET BEFORE PROCEEDING
sudo apt-get install apt-utils

cd /mnt/nepi_storage/tmp
sudo chown -R nepi:nepi l4t-jetpack/
cd l4t-jetpack/
sudo make image

# run network config sciprt
sudo python /mnt/nepi_storage/tmp/nepi/config/etc/network/tune_ethernet_interfaces.py

#Install visual studio if not present
#https://unix.stackexchange.com/questions/765383/e-unable-to-locate-package-code
sudo apt install wget gpg apt-transport-https
curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo install -o root -g root -m 644 microsoft.gpg /usr/share/keyrings/microsoft-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/usr/share/keyrings/microsoft-archive-keyring.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code




# Install chromium if not present
#https://forum.snapcraft.io/t/try-to-update-snapd-and-refresh-the-core-snap/20725
sudo install snap
sudo snap install core snapd
#https://www.cyberciti.biz/faq/install-chromium-browser-on-ubuntu-linux/#google_vignette
snap install chromium
_______________
# Install docker if not present
#https://www.forecr.io/blogs/installation/how-to-install-and-run-docker-on-jetson-nano

# Install nvidia toolkit
#https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
sudo apt-get install -y nvidia-container-toolkit
sudo apt-get install nvidia-container-run
#runtime configure --runtime=docker --config=$HOME/.config/docker/daemon.json

#Stop docker
sudo systemctl stop docker
sudo systemctl stop docker.socket

# Set docker image location
#https://tienbm90.medium.com/how-to-change-docker-root-data-directory-89a39be1a70b

mkdir /mnt/nepi_storage/docker
sudo mv /var/lib/docker /mnt/nepi_storage/docker



sudo vim /etc/docker/daemon.json
#Add
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
    
}


#### Don't do
,

    "data-root": "/mnt/nepi_storage/docker"
}
''''




sudo vi /etc/default/docker
# Edit this line and uncomment
DOCKER_OPTS="--dns 8.8.8.8 --dns 8.8.4.4"  -g /mnt/nepi_storage/docker


# Set docker service root location
#https://stackoverflow.com/questions/44010124/where-does-docker-store-its-temp-files-during-extraction
sudo vi /usr/lib/systemd/system/docker.service
#Comment out ExecStart line and add below
ExecStart=/usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock --data-root=/mnt/nepi_storage/docker
#Then reload
sudo systemctl daemon-reload

#start docker
sudo systemctl start docker.socket
sudo systemctl start docker
sudo docker info





###########################################
## Build nepi container

Run container
https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack

sudo docker run -it --net=host --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix nvcr.io/nvidia/l4t-jetpack:r35.1.0


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

# Save Layered Image as tar
sudo docker save -o /mnt/nepi_storage/tmp/nepi1.tar a1e4e38c2162
sudo docker tag 7aa663d0a1e3 nepi_ft1
