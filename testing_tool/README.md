### Systemtap installation

```
sudo apt install libelf-dev libdw-dev
cd /tmp && wget https://sourceware.org/systemtap/ftp/releases/systemtap-4.4.tar.gz
tar -xf systemtap-4.4.tar.gz && cd systemtap-4.4
./configure
make && sudo make install
```

### Debug symbols installation

```
printf "deb http://ddebs.ubuntu.com %s main restricted universe multiverse\n" $(lsb_release -cs){,-updates,-security,-proposed} | \
sudo tee -a /etc/apt/sources.list.d/ddebs.list
wget -O - http://ddebs.ubuntu.com/dbgsym-release-key.asc | sudo apt-key add -
sudo apt update
sudo apt install linux-image-$(uname -r)-dbgsym
```

If there is no dbgsym package and stap command doesn't build kernel module, then rebuild linux kernel with [config](https://wiki.archlinux.org/index.php/SystemTap#Kernel_rebuild).

### Building kernel module

```
sudo stap copying.stp -m copying.ko -p4
```
