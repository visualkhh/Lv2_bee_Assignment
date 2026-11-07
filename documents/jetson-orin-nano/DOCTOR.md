# DOCTOR

## check
```bash
echo '--- kernel / arch ---'
uname -a
uname -m
dpkg --print-architecture

echo '--- Jetson L4T / JetPack ---'
cat /etc/nv_tegra_release 2>/dev/null || echo 'no nv_tegra_release'
dpkg-query -W -f='${Package} ${Version}\n' nvidia-l4t-core 2>/dev/null || true

echo '--- glibc / Python ---'
getconf GNU_LIBC_VERSION
python3 --version

echo '--- Jetson model ---'
cat /proc/device-tree/model 2>/dev/null; echo
--- Ubuntu ---
PRETTY_NAME="Ubuntu 24.04.4 LTS"
NAME="Ubuntu"
VERSION_ID="24.04"
VERSION="24.04.4 LTS (Noble Numbat)"
VERSION_CODENAME=noble
ID=ubuntu
ID_LIKE=debian
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
UBUNTU_CODENAME=noble
LOGO=ubuntu-logo
--- kernel / arch ---
Linux bee 6.8.12-1021-tegra #1 SMP PREEMPT Thu Aug  6 21:56:04 PDT 2026 aarch64 aarch64 aarch64 GNU/Linux
aarch64
arm64
--- Jetson L4T / JetPack ---
# R39 (release), REVISION: 2.1, GCID: 46758480, BOARD: generic, EABI: aarch64, DATE: Fri Aug  7 05:54:22 AM UTC 2026
# KERNEL_VARIANT: oot
TARGET_USERSPACE_LIB_DIR=nvidia
TARGET_USERSPACE_LIB_DIR_PATH=usr/lib/aarch64-linux-gnu/nvidia
nvidia-l4t-core 39.2.1-20260806224157
--- glibc / Python ---
glibc 2.39
Python 3.12.3
--- Jetson model ---
NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super
bee@bee:~$ 

```