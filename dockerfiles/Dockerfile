# https://hub.docker.com/_/ros
# https://github.com/osrf/docker_images/blob/master/ros/jazzy/ubuntu/noble/perception/Dockerfile
FROM ros:jazzy-ros-base-noble

# 是否开发模式，如果是开发模式，会安装「开发环境」中的软件
ARG CLEAN_APT_CACHE=1

# 「基本系统」中需要的软件列表
ARG APT_INSTALL_LIST=""
ARG APT_INSTALL_LIST="$APT_INSTALL_LIST ca-certificates locales sudo"
ARG APT_INSTALL_LIST="$APT_INSTALL_LIST vim nano tree pciutils gedit"
ARG APT_INSTALL_LIST="$APT_INSTALL_LIST git openssh-client openssh-server"
ARG APT_INSTALL_LIST="$APT_INSTALL_LIST build-essential cmake"
ARG APT_INSTALL_LIST="$APT_INSTALL_LIST python3-pip python3-venv"
# ARG APT_INSTALL_LIST="$APT_INSTALL_LIST ros-jazzy-desktop-full=0.11.0-1*"
ARG APT_INSTALL_LIST="$APT_INSTALL_LIST ros-jazzy-desktop-full"
ARG APT_INSTALL_LIST="$APT_INSTALL_LIST x11-xserver-utils x11-apps net-tools"
# ARG APT_INSTALL_LIST="$APT_INSTALL_LIST libxcb-keysyms1-dev libxcb-image0-dev \
#     libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync0-dev libxcb-xfixes0-dev \
#     libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev \
#     libfontconfig1-dev libfreetype6-dev libx11-dev libxext-dev libxfixes-dev \
#     libxi-dev libxrender-dev libxcb1-dev libx11-xcb-dev libxcb-glx0-dev x11vnc \
#     xauth build-essential mesa-common-dev libglu1-mesa-dev libxkbcommon-dev \
#     libxcb-xkb-dev libxslt1-dev libxkbcommon-x11-0"

# 自定义镜像源 (tuna 在科学网络下不可访问)
ARG MIRRORS_URL="mirrors.tuna.tsinghua.edu.cn"
# ARG MIRRORS_URL="mirrors.ustc.edu.cn"
ENV DEBIAN_FRONTEND=noninteractive

# 添加 ROS2 的 key
COPY ./.cache/ros.key /usr/share/keyrings/ros-archive-keyring.gpg

# set -x 选项会使得每个命令在执行前都会被打印出来，报错时会显示是哪个命令出错
RUN set -x && \
    sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/g' ${HOME}/.bashrc && \
    # 修改 apt 源
    sed -i -e "s#//.*archive.ubuntu.com#//${MIRRORS_URL}#g" -e "s#//ports.ubuntu.com#//${MIRRORS_URL}#g" /etc/apt/sources.list.d/ubuntu.sources && \
    # 移除镜像中自带的 ROS2 源
    cp /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2-latest.list.bak && \
    rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    # 添加 ROS2 源
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://${MIRRORS_URL}/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    # apt-get upgrade -y && \ # 升级会导致 ros-desktop-full 无法安装
    # 开发模式下「基本系统」和「部署环境」的软件列表合并
    apt-get install -y --no-install-recommends ${APT_INSTALL_LIST} && \
    apt-get autoremove -y && \
    # 非开发模式下清理 apt 缓存
    if [ $CLEAN_APT_CACHE -ne 1 ]; then { apt-get clean all; rm -rf /var/lib/apt/lists/*; } fi && \
    localedef -i zh_CN -c -f UTF-8 -A /usr/share/locale/locale.alias zh_CN.UTF-8 


ENV PATH=$PATH:/opt/ros/jazzy/bin
# ENV LANG=zh_CN.utf8
ENV LANG=US_en.utf8

# 系统时间错误可能导致: `tls: failed to verify certificate: x509: certificate signed by unknown authority`
# 修改时区 (date -R 检查当前时间)
RUN ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime

ARG USER_NAME="root"
ARG USER_HOME="/${USER_NAME}"

# 激活
RUN [ -d "/opt/ros/jazzy/" ] && echo "source /opt/ros/jazzy/setup.bash" >> $USER_HOME/.bashrc

# ============================================================
#   Python 环境
# ============================================================
# # PEP 668 – Python base environments Python 增强提案 (PEP)：https://realpython.com/python-virtual-environments-a-primer/?ref=yaolong.net
# # https://www.adamsdesk.com/posts/resolve-pip-externally-managed-environment
# # 临时解决方案  --ignoreExternallyManagedEnvironment
ADD . $USER_HOME/tmp
ENV PYTHON_VENV_DIR=$USER_HOME/.venv
ENV PIP_MIRROR="https://mirrors.ustc.edu.cn/pypi/simple"
RUN set -x && \
    python3 -m venv ${PYTHON_VENV_DIR} --clear && \
    echo "source ${PYTHON_VENV_DIR}/bin/activate" >> $USER_HOME/.bashrc && \
    VENV_PYTHON=${PYTHON_VENV_DIR}/bin/python3 && \
    $VENV_PYTHON -m pip install -i $PIP_MIRROR -U --upgrade pip setuptools wheel && \
    $VENV_PYTHON -m pip config set global.index-url $PIP_MIRROR && \
    [ -f $USER_HOME/tmp/requirements.txt ] && $VENV_PYTHON -m pip install -r ${USER_HOME}/tmp/requirements.txt || echo "requirements.txt not found, skip install" && \
    sudo rm -rf $USER_HOME/tmp && \
    # 清理缓存，缓存目录： - [Linux]: ~/.cache/pip and ~/.cache/pipenv  - [macOS]: ~/Library/Caches/pip and ~/.cache/pipenv  - [Windows]: %LocalAppData%/pip/Cache
    $VENV_PYTHON -m pip cache purge

# 提升虚拟环境 python 的优先级
ENV PATH=${PYTHON_VENV_DIR}/bin:$PATH
