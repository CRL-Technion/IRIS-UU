#chmod u+x InstallIRISRequirements
sudo apt install wget
#Install git
sudo apt-get update
sudo apt-get install git-core
git --version

#Install gcc
sudo apt install -y build-essential
sudo apt-get install -y manpages-dev
gcc --version

#Install boost
sudo apt install -y libboost-dev
sudo apt install -y libboost-all-dev
dpkg -s libboost-dev | grep 'Version'

#Install Cmake
sudo apt-get install -y cmake
cmake --version

#Install ompl
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh --python

#Clone IRIS
mkdir Projects
cd Projects
git clone https://github.com/CRL-Technion/IRIS-UU.git
cd IRIS-UU
git submodule update --init --recursive

#download data of IRIS
export fileid=19DGtog4D4hAgwFu1bV_ct0h_n-G4BR1Z
export filename=data.tar.gz
wget --save-cookies cookies.txt 'https://docs.google.com/uc?export=download&id='$fileid -O- \
     | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt

wget --load-cookies cookies.txt -O $filename \
     'https://docs.google.com/uc?export=download&id='$fileid'&confirm='$(<confirm.txt)

rm -rf confirm.txt
rm -rf cookies.txt

tar -xvf data.tar.gz
rm -rf data.tar.gz

#Install IRIS
mkdir build
cd build

#The ompl version may change
cmake -DOMPL_INCLUDE_DIRS=/usr/local/include/ompl-1.5/ -DOMPL_LIBRARIES=/usr/local/lib/libompl.so -DCMAKE_BUILD_TYPE=Debug ..
make

