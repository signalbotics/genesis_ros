# Debian packaging (Ubuntu 24.04 / ROS 2 Jazzy)

Builds three `.deb`s from a Genesis engine checkout plus this repo:

| Package | Arch | Contents |
|---|---|---|
| `genesis-world` | `amd64` | Vendored Python 3.12 venv at `/opt/genesis/venv` with Genesis + `torch` (CUDA 13). Wrapper at `/usr/bin/genesis` launches the venv's interpreter. |
| `genesis-world-assets` | `all` | `genesis/assets/` (URDFs, meshes, textures) under `/opt/genesis/assets/`. |
| `ros-jazzy-genesis-ros` | `amd64` | The `genesis_ros` ROS 2 package (generated via `bloom`). `Depends: genesis-world`. |

Version: `<genesis-pyproject-version>+git<YYYYMMDD>.<short-sha>-1`, where the
sha is from *this* repo's HEAD. Genesis's own version comes from its
`pyproject.toml`.

## Prerequisites

- Docker on the build host.
- A Genesis engine checkout somewhere. By default we look for one at
  `../Genesis` (sibling of this repo). Override with `GENESIS_SRC=...`.

## Build everything (Docker-based, recommended)

```bash
./build-in-docker.sh            # builds all 3 .debs → ./out/
./build-in-docker.sh genesis-world       # just one package
./build-in-docker.sh ros                 # just the ROS bridge
```

First run builds the `genesis-deb-builder:noble` image (~2 min, cached
after). Subsequent runs reuse `./​.pip-cache/` so wheels don't re-download.

Non-default Genesis location:

```bash
GENESIS_SRC=/path/to/your/Genesis ./build-in-docker.sh
```

## Native build (no Docker)

Ubuntu 24.04 Noble only:

```bash
sudo apt install -y devscripts debhelper dh-python python3-bloom \
    python3-pip python3.12-venv python3-all fakeroot build-essential git \
    libdistro-info-perl ros-jazzy-ros-base ros-jazzy-ament-package \
    python3-rosdep
sudo rosdep init || true
rosdep update
GENESIS_SRC=/path/to/Genesis ./build-all.sh
```

## Installing the built .debs

```bash
sudo apt install ./out/genesis-world_*.deb \
                 ./out/genesis-world-assets_*.deb \
                 ./out/ros-jazzy-genesis-ros_*.deb
```

`apt install ./file.deb` (not `dpkg -i`) pulls runtime dependencies
automatically.

## Publishing to a local apt repo

```bash
sudo apt install -y reprepro
cd apt-repo
reprepro includedeb noble ../out/*.deb
# Serve ./ over nginx / caddy as e.g. https://apt.example.com/
```

Clients:

```bash
echo "deb [trusted=yes] https://apt.example.com noble main" | \
    sudo tee /etc/apt/sources.list.d/genesis.list
sudo apt update
sudo apt install ros-jazzy-genesis-ros genesis-world-assets
```

## Layout

```
packaging/
├── build-in-docker.sh          # driver for the Docker build
├── build-all.sh                # driver for the native build
├── Dockerfile.noble            # builder image definition
├── apt-repo/conf/distributions # reprepro config
├── genesis-world/              # engine + vendored venv
│   ├── debian/{control,rules,postinst,prerm,copyright}
│   ├── wrapper/genesis         # /usr/bin/genesis
│   └── build.sh
├── genesis-world-assets/       # URDF + meshes
│   ├── debian/{control,rules,copyright}
│   └── build.sh
├── ros-jazzy-genesis-ros/      # ROS 2 bridge via bloom
│   ├── build.sh
│   └── README.md
└── out/                        # .deb artifacts land here
```
