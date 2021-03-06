# Virtual Machine

1. Download the virtual machine using this link [https://memmo-data.laas.fr/media/memmo-student.zip](https://memmo-data.laas.fr/media/memmo-student.zip).
1. If you don't already have a virtualization system, download [https://www.virtualbox.org/](https://www.virtualbox.org/).
1. Unzip and load the virtual machine into your virtualization system (the virtual machine is in an open format and should be read by any kind of virtualization system).
1. Start the VirtualBox Image. If you start the VirtualBox for the first time, everything should be already running with the test notebook ~/summer-school/setup/load_talos.ipynb. Say Hi to Talos!
1. Otherwise, you can start by running ```jupyter notebook``` in ~/summer-school. Notebooks for the tutorials are in ~/summer-school/tutorials.

## Update the notebooks

The VirtualBox or Docker images might not contain the latest version of the notebooks so please pull the latest modifications before the tutorials. To do that :
1. If running in one of the terminals, kill jupyter notebook using ctrl+C.
1. Pull changes from the repository:
```bash
cd ~/summer-school
git pull
git submodule update --init
```
1. restart ```jupyter notebook```






