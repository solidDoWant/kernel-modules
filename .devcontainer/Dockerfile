FROM mcr.microsoft.com/devcontainers/base:jammy

RUN apt update && apt install -y git ssh make gcc libssl-dev liblz4-tool expect g++ \
    patchelf chrpath gawk texinfo chrpath diffstat software-properties-common bison \
    flex fakeroot cmake unzip device-tree-compiler libncurses-dev python3-pip \
    python3-pyelftools bc make build-essential libssl-dev zlib1g-dev libbz2-dev \
    libreadline-dev libsqlite3-dev wget curl llvm libncursesw5-dev xz-utils tk-dev \
    libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev python3 rsync gcc-aarch64-linux-gnu \
    python-is-python3 kmod

COPY config.yml /home/vscode/.earthly/config.yml
RUN chown -R 1000:1000 /home/vscode
