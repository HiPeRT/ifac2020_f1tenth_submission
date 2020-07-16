sudo apt-get install libyaml-cpp-dev libeigen3-dev cmake
sudo apt-get install libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev
#if you have problems installing armadillo, go here -> http://arma.sourceforge.net/download.html
sudo apt-get install libarmadillo-dev libarmadillo9 libarmadillo9-dbgsym
sudo apt install python-pip

sudo pip install casadi

#if you have any problem about cmake see the c++ code chapter in readme here https://github.com/kctess5/range_libc
git clone https://github.com/kctess5/range_libc
cd range_libc
mkdir build
cd build
cmake ..
make

