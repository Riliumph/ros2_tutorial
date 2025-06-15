#!/bin/bash

bash_completion_d=$HOME/.config/bash_completion.d
mkdir -p ${bash_completion_d}

if type pip &>/dev/null; then
  pip completion --bash >${bash_completion_d}/pip
  echo "source ${bash_completion_d}/pip" >>$HOME/.bashrc
fi

