#!/bin/bash
# Update the system package manager
sudo apt update;
sudo apt upgrade;
# Install packages necessary for build
sudo apt install -y cmake;
sudo apt install -y gcc g++;
sudo apt install -y python3 python3-dev python3-tk;
sudo apt install -y libhdf5-serial-dev;
sudo apt install -y libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libglfw3-dev;
sudo apt install -y doxygen;
sudo apt install -y swig;
sudo apt install -y lcov;               # For coverage
sudo apt install -y graphviz clang;     # For docs
sudo apt install -y asciidoctor;        # For docs
# Install git-lfs and pull all the git lfs files
sudo apt install -y git-lfs;
# Install Python packages
sudo apt install -y python3-pip;
sudo apt install -y python3-tk;
pip3 install --upgrade matplotlib;                  # For liveplot and analysis
pip3 install --upgrade pandas;                      # For post processing
pip3 install --upgrade numpy;                       # For analysis
pip3 install --upgrade h5py;                        # For analysis
pip3 install --upgrade plotly;                      # For analysis
pip3 install --upgrade kaleido;                     # For saving plotly images
pip3 install --upgrade selenium;                    # For cesium visuals
pip3 install --upgrade requests urllib3 chardet;    # For cesium visuals
pip3 install --upgrade streamlit;                   # For Dallin tool
sudo pip3 install --upgrade streamlit;              # For Dallin tool
# Install google chrome for visuals
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb;
sudo dpkg -i google-chrome-stable_current_amd64.deb;
sudo apt -f -y install;
rm google-chrome-stable_current_amd64.deb;
# Update git submodules. If already done this won't do anything.
git submodule update --remote --recursive --init;
# Check out files which were not pulled using git lfs before it was installed
git lfs fetch --all;
git lfs checkout;