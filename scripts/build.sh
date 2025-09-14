#!/usr/bin/env sh

set -e

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <src_dir> <target_dir>"
    exit 1
fi

src_dir=$1
shift

target_dir=$1
shift

echo 'Check pyenv...'
if ! command -v pyenv
then
    echo "pyenv could not be found. install pyenv using https://github.com/pyenv/pyenv#installation"
    exit 1
fi
echo 'success'

cd "$src_dir"
    unset PYTHONPATH
    deactivate 2>/dev/null || true

    python_version=$(cat .python-version)
    echo "ensuring python version $python_version..."
    pyenv install -s "$python_version"
    echo "python version $python_version available"

    echo 'fixing paths...'
    PATH="$(pyenv root)/shims:$PATH"
    export PATH
    echo "python path is $(which python)"
    echo "python version is $(python --version)"
    
    echo 'installing venv...'
    python -m pip install virtualenv
    python -m venv .venv
    . .venv/bin/activate
    python -m pip install .
    echo 'success'
cd -

cp -a "$src_dir/.venv" "$target_dir"