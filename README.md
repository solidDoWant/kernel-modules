## What is this?
This repo contains a list of kernel modules that I've written that have not yet been upstreamed to mainline Linux.

## Current modules
| Name | Status | Features |
|-|-|-|
| pcie-dw-ep-rockchip | Untested | RK3588/RK2588s: eDMA support, MSI/MSIX support, bus master support, WAKE# and PERST# |


## Build instructions
1. Download and install [Earthly](https://earthly.dev), or launch a dev container with the config in .devcontainer.
2. Run `earthly +build-module --MODULE_ARCH=<your target arch, probably arm64>`.
3. The output is in the created `built-modules` directory.
