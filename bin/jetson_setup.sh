#!/usr/bin/env bash

# Setup script for Nvidia Jetson Nano boards.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# September 2, 2023

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then set -o xtrace; fi

# Verify that the PWD is the project root directory
CURR_DIR=${PWD##*/}
INIT_DIR=$PWD
REQ_CURR_DIR="turtlebot3-utv"
if [[ $CURR_DIR != "$REQ_CURR_DIR" ]]; then
  echo >&2 "ERROR: Wrong path, this script must run inside $REQ_CURR_DIR"
  return 1
fi

# Disable serial console
echo "Disabling Nvidia serial console..."
sleep 1
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
udevadm trigger

sudo apt-get update

# Remove some unnecessary packages
echo "Removing unnecessary packages..."
sleep 1
sudo apt-get purge -y \
  libreoffice* \
  modemmanager

# Install basic utilities
echo "Installing basic utilities..."
sleep 1
sudo apt-get install -y --no-install-recommends \
  apt-utils \
  automake \
  build-essential \
  cmake \
  curl \
  gcc \
  gdb \
  gedit \
  gparted \
  gtk3-nocsd \
  htop \
  lm-sensors \
  lsb-release \
  make \
  minicom \
  nano \
  neofetch \
  openjdk-11-jdk \
  pigz \
  python3 \
  python3-pip \
  rsync \
  rt-tests \
  screen \
  ssh \
  sshfs \
  valgrind \
  vim \
  wget
sudo -H pip3 install -U setuptools
sudo -H pip3 install -U jetson-stats

# Update system and clean up
echo "Updating current packages and cleaning up..."
sleep 1
sudo apt-get upgrade -y
sudo apt-get autoclean
sudo apt-get autoremove -y

# Install perfect fonts (required to use Zsh from local terminals)
echo "Installing Zsh fonts..."
sleep 1
mkdir -p ~/.local/share/fonts
wget -nc -O "$HOME/.local/share/fonts/MesloLGS\ NF\ Regular.ttf" https://github.com/romkatv/powerlevel10k-media/raw/master/MesloLGS%20NF%20Regular.ttf
wget -nc -O "$HOME/.local/share/fonts/MesloLGS\ NF\ Bold.ttf" https://github.com/romkatv/powerlevel10k-media/raw/master/MesloLGS%20NF%20Bold.ttf
wget -nc -O "$HOME/.local/share/fonts/MesloLGS\ NF\ Italic.ttf" https://github.com/romkatv/powerlevel10k-media/raw/master/MesloLGS%20NF%20Italic.ttf
wget -nc -O "$HOME/.local/share/fonts/MesloLGS\ NF\ Bold\ Italic.ttf" https://github.com/romkatv/powerlevel10k-media/raw/master/MesloLGS%20NF%20Bold%20Italic.ttf
fc-cache -f -v

# Install and configure Zsh and Bash
echo "Installing and configuring Zsh..."
sleep 1
sudo apt-get install -y --no-install-recommends \
  python3-pygments \
  zsh \
  zsh-doc

sudo chsh -s /usr/bin/zsh "$USER"

ZSH=$HOME/.oh-my-zsh
ZSH_CUSTOM=$ZSH/custom
ZSH_THEMES=$ZSH_CUSTOM/themes

wget -qO- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh | zsh || true
git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-syntax-highlighting.git "${ZSH_CUSTOM:-~/.oh-my-zsh/custom}"/plugins/zsh-syntax-highlighting
git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-autosuggestions "${ZSH_CUSTOM:-~/.oh-my-zsh/custom}"/plugins/zsh-autosuggestions
git clone --single-branch --depth 1 https://github.com/romkatv/powerlevel10k.git "$ZSH_THEMES"/powerlevel10k

cp config/zshrc ~/.zshrc
cp config/p10k.zsh ~/.p10k.zsh

echo "Configuring Bash..."
sleep 1
cp config/bashrc ~/.bashrc

echo "Configuring nano and vim..."
sleep 1
cp config/nanorc ~/.nanorc
cp config/vimrc ~/.vimrc

# Install Docker Compose
echo "Installing Docker Compose v2..."
sleep 1
sudo mkdir -p /usr/local/lib/docker/cli-plugins
sudo curl -SL https://github.com/docker/compose/releases/latest/download/docker-compose-linux-aarch64 -o /usr/local/lib/docker/cli-plugins/docker-compose
sudo chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
sudo ln -s /usr/local/lib/docker/cli-plugins/docker-compose /usr/local/bin/docker-compose

# Create docker group and add user to it
echo "Creating new group for Docker users and adding $USER to it..."
sleep 1
sudo groupadd docker
sudo usermod -aG docker "$USER"

# Add current user to hardware access groups
HW_GROUPS=(
  "adm"
  "dialout"
  "plugdev"
  "tty"
  "uucp"
  "video")

echo "Adding current user $USER to hardware access groups..."
sleep 1
for GROUP in "${HW_GROUPS[@]}"; do
  if ! groups | grep -q "$GROUP"; then
    sudo usermod -a -G "$GROUP" "$USER"
    echo "$USER added to $GROUP"
  fi
done

# Configure udev rules for OpenCR board
echo "Adding udev rules for OpenCR board..."
sleep 1
sudo cp config/99-opencr-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

cd "$INIT_DIR" || return 1
echo "Done! Next run 'bin/container_setup.sh' to setup the Docker container on this board."
echo "The system will now be restarted to apply all changes..."
sleep 5
sudo reboot
