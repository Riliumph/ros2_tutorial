#!/bin/bash

bash_completion_d=$HOME/.config/bash_completion.d
mkdir -p ${bash_completion_d}

if type docker &>/dev/null; then
  docker completion bash >${bash_completion_d}/docker
  echo "source ${bash_completion_d}/docker" >>$HOME/.bashrc
fi

if type pip &>/dev/null; then
  pip completion --bash >${bash_completion_d}/pip
  echo "source ${bash_completion_d}/pip" >>$HOME/.bashrc
fi
