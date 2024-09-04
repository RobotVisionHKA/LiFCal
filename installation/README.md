# Introduction

This folder is associated with the LiFCal repository for light field camera calibration. It contains explanations and files to install the dependencies required by the program:
- Nvidia Driver;
- Cuda;
- Ceres Solver;
- COLMAP;
- OpenCV.

This file proposes two methods to install dependencies: a local installation and a Docker installation. The first method allows all dependencies to be installed directly on the system. The second keeps the system unchanged by running the program in a Docker container. Both methods are described below, using Ansible.

# Install Ansible

Ansible is a tool to define a system state in so-called playbooks. These define actions such as software installation. Ansible is build on top of python3. Here is a quick installation guid. For more details, visit <a href="https://docs.ansible.com/ansible/latest/installation_guide/intro_installation.html" target="_blank">Ansible documentation</a>.
```bash
# Install python3
sudo apt update
sudo apt install python3 python3-pip
python3 --version

# Install Ansible
python3 -m pip install --user ansible
ansible --version
activate-global-python-argcomplete --user
```

# Install dependencies

This section explains how to install LiFCal dependencies. Two options are available: local installation and Docker installation. Both are described below.
For both methods, navigate to the installation folder.
```bash
cd installation
```

## Option 1: Local installation

First, navigate to the folder containing the ansible playbooks.
```bash
cd playbooks
```

Run Ansible dependency installation scripts.
```bash
ansible-playbook -i localhost, LiFCal_install_NvidiaDriver_535.yaml --connection=local --ask-become
ansible-playbook -i localhost, LiFCal_install_Cuda_12_2_2.yaml --connection=local --ask-become
ansible-playbook -i localhost, LiFCal_install_CeresSolver_2_1_0.yaml --connection=local --ask-become
ansible-playbook -i localhost, LiFCal_install_COLMAP.yaml --connection=local --ask-become
ansible-playbook -i localhost, LiFCal_install_OpenCV.yaml --connection=local --ask-become
```

## Option 2: Docker installation

First, navigate to the folder containing the ansible playbooks.
```bash
cd playbooks
```

To use the Docker method, install Docker, Nvidia drivers and the Nvidia Docker Toolkit.
```bash
# Install Docker
ansible-playbook -i localhost, LiFCal_install_Docker.yaml --connection=local --ask-become

# Install Nvidia
ansible-playbook -i localhost, LiFCal_install_NvidiaDriver_535.yaml --connection=local --ask-become
ansible-playbook -i localhost, LiFCal_install_NvidiaContainer.yaml --connection=local --ask-become
```

Navigate to the folder containing the Dockerfile.
```bash
cd ..
```

The image can now be built from the Dockerfile. Run the following command, replacing the <name_tag> with a custom name (e.g. `LiFCal:1.0.0`).
```bash
docker build . -t <name_tag>
```
The following command enables to test the container by executing it.
```bash
sudo docker run -it --runtime=nvidia --rm --gpus all <name_tag> bash
```

Setup the toolchain according to your IDE or in the console with the following Docker options.
```bash
-it
-v </path/to/DATA>:/data        # mount a local directory to /data of the container
--runtime=nvidia                # enable CUDA
--rm                            # delete all files if the container dies
--gpus all                      # enable GUP support
```

