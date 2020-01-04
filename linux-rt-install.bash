wget https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.4.12.tar.xz
wget https://www.kernel.org/pub/linux/kernel/projects/rt/4.4/patch-4.4.12-rt19.patch.xz
xz -cd linux-4.4.12.tar.xz | tar xvf -
cd linux-4.4.12
xzcat ../patch-4.4.12-rt19.patch.xz | patch -p1
sudo make menuconfig
make && make modules
sudo make install && sudo make modules_install
grub-mkconfig -o /boot/grub/grub.cfg
