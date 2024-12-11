
## Linux

### No GUI
sudo init 3
dorvan
6956
sudo init 5
ctrl + alt + f* : change tty console

tmux

ctrl + b , shift + 2 : split horizotal
ctrl + b, shift + 5 : split verticaly
ctrl + b + arrow : rezize
ctrl + b, arrow change terminal

### Add environment variable persistent
sudo nano /etc/environment
ABC = "/path/to/file"
source /etc/environment
echo $ABC

### Memory usage
nvidia-smi

watch -n 1 free -h

htop


## Git

ssh-keygen -t rsa -b 4096 -C "your_email@example.com"

add rsa content to github

git remote set-url origin git@github.com:user/repository.git


## Python env

source myEnv/bin/activate
which pip
deactivate

source /home/dorvan/Documents/dorvan/Babydoll/Produits/Babybot-01/Informatic-01/venv/jetsonEnv/bin/activate

source /home/dorvan/Documents/dorvan/Babydoll/Produits/Babybot-01/Informatic-01/venv/hostEnv/bin/activate

## Tensorboard

tensorboard --logdir ./ppo_ant_v1/t_logs

