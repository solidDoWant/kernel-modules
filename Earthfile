VERSION 0.8

ARG --global KERNEL_REMOTE=https://github.com/torvalds/linux.git
ARG --global KERNEL_GITREF=master
ARG --global MODULE_ARCH

APT_INSTALL:
    FUNCTION
    ARG --required PACKAGES
    RUN apt update && DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y $PACKAGES

SET_MODULE_ARCH:
    FUNCTION
    IF [ -z "$MODULE_ARCH" ]
        ENV MODULE_ARCH = $(uname -m)
    END

# TODO support armel/armhf
DETERMINE_TRIPLET:
    FUNCTION
    DO +SET_MODULE_ARCH

    ENV TRIPLET_OS = "linux-gnu"
    IF [ "$MODULE_ARCH" = "arm64" ]
        ENV TRIPLET_MACHINE = "aarch64"
    ELSE IF [ "$MODULE_ARCH" = "amd64" ]
        ENV TRIPLET_MACHINE = "x86-64"
    ELSE
        ENV TRIPLET_MACHINE = $(echo "$MODULE_ARCH" | sed 's/_/-/')
    END

SET_KERNEL_MAKE_ARGS:
    FUNCTION
    DO +DETERMINE_TRIPLET

    # If cross compiling
    LET KERNEL_MAKE_ARGS = "--jobs=$(nproc) ARCH=$MODULE_ARCH"
    IF [ "$MODULE_ARCH" != "$(uname -m)" ]
        SET KERNEL_MAKE_ARGS = "$KERNEL_MAKE_ARGS CROSS_COMPILE=$TRIPLET_MACHINE-$TRIPLET_OS-"
    END
    ENV KERNEL_MAKE_ARGS = $KERNEL_MAKE_ARGS

# TODO support armel/armhf
kernel-build-image:
    FROM ubuntu:22.04

    DO +DETERMINE_TRIPLET

    DO +APT_INSTALL --PACKAGES="git ssh make gcc libssl-dev liblz4-tool expect g++ \
        patchelf chrpath gawk texinfo chrpath diffstat software-properties-common bison \
        flex fakeroot cmake unzip device-tree-compiler libncurses-dev python3-pip \
        python3-pyelftools bc make build-essential libssl-dev zlib1g-dev libbz2-dev \
        libreadline-dev libsqlite3-dev wget curl llvm libncursesw5-dev xz-utils tk-dev \
        libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev python3 rsync python-is-python3 \
        libelf-dev"
    DO +APT_INSTALL --PACKAGES "gcc-$TRIPLET_MACHINE-$TRIPLET_OS"

download-kernel:
    FROM +kernel-build-image
    WORKDIR /kernel

    RUN git clone --single-branch --branch $KERNEL_GITREF --depth 1 $KERNEL_REMOTE .
    SAVE ARTIFACT /kernel AS LOCAL ./kernel/$KERNEL_GITREF

setup-kernel:
    FROM +download-kernel

    DO +SET_KERNEL_MAKE_ARGS
    RUN make $KERNEL_MAKE_ARGS clean && make $KERNEL_MAKE_ARGS mrproper

    COPY --if-exists .config .
    IF [ ! -f .config ]
        COPY --if-exists "$MODULE_ARCH.config" .config
    END

    IF [ ! -f .config ]
        RUN make $KERNEL_MAKE_ARGS defconfig

        COPY --if-exists config.overrides .
        IF [ -f config.overrides ]
            RUN cat "config.overrides" >> ".config"
        END
    END

    RUN make $KERNEL_MAKE_ARGS oldconfig

    SAVE ARTIFACT /kernel AS LOCAL ./kernel/$KERNEL_GITREF

menuconfig:
    FROM +setup-kernel

    DO +SET_KERNEL_MAKE_ARGS
    RUN --interactive make $KERNEL_MAKE_ARGS menuconfig

    SAVE ARTIFACT ./.config AS LOCAL "./$MODULE_ARCH.config"

build-kernel:
    FROM +setup-kernel

    DO +SET_KERNEL_MAKE_ARGS
    RUN make $KERNEL_MAKE_ARGS

    SAVE ARTIFACT . AS LOCAL ./kernel/$KERNEL_GITREF

build-module:
    FROM +build-kernel

    WORKDIR /src
    COPY . .

    LET MODULE_MAKE_ARGS = "$KERNEL_MAKE_ARGS -C /kernel M=$(echo $PWD) KERNEL_SRC_DIR=/kernel"

    RUN make $MODULE_MAKE_ARGS clean && make $MODULE_MAKE_ARGS modules
    SAVE ARTIFACT *.ko AS LOCAL ./built-modules/
