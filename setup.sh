
if [ -n "$($SHELL -c 'echo $ZSH_VERSION')" ]; then
   echo 'source ~/rda_ws/devel/setup.bash' >> ~/.bashrc
elif [ -n "$($SHELL -c 'echo $BASH_VERSION')" ]; then
   echo 'source ~/rda_ws/devel/setup.zsh' >> ~/.zshrc
fi