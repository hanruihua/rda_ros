
if [ -n "$($SHELL -c 'echo $ZSH_VERSION')" ]; then
   echo 'source ~/rda_ws/devel/setup.zsh' >> ~/.zshrc
   zsh -c 'source ~/.zshrc'
elif [ -n "$($SHELL -c 'echo $BASH_VERSION')" ]; then
   echo 'source ~/rda_ws/devel/setup.bash' >> ~/.bashrc
   bash -c 'source ~/.bashrc'
fi