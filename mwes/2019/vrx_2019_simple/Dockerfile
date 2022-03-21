FROM ros:melodic-ros-base

# Set Ubuntu release
ARG RELEASE=bionic

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN [ -z "$(apt-get indextargets)" ]
RUN echo 'Etc/UTC' > /etc/timezone \
 && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
 && apt update \
 && apt install -q -y tzdata \
 && rm -rf /var/lib/apt/lists/*

# Required utilities 
RUN apt update \
 && apt install -y --no-install-recommends \
        ros-melodic-ros-base=1.4.1-0* \
        ros-melodic-ros-core=1.4.1-0* \
        python-rosdep \
        python-rosinstall \
        python-vcstools \
 && rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh run_my_system.bash /

CMD ["/bin/bash"]
ENTRYPOINT ["/ros_entrypoint.sh"]

RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros1-latest.list' \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && apt update \
 && apt install -q -y \
        dirmngr \
        gnupg2 \
        lsb-release \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /run/systemd && echo 'docker' > /run/systemd/container
RUN set -xe \
 && echo '#!/bin/sh' > /usr/sbin/policy-rc.d \
 && echo 'exit 101' >> /usr/sbin/policy-rc.d \
 && chmod +x /usr/sbin/policy-rc.d \
 && dpkg-divert --local --rename --add /sbin/initctl \
 && cp -a /usr/sbin/policy-rc.d /sbin/initctl \
 && sed -i 's/^exit.*/exit 0/' /sbin/initctl \
 && /bin/sh -c 'echo "force-unsafe-io" > /etc/dpkg/dpkg.cfg.d/docker-apt-speedup' \
 && /bin/sh -c 'echo "DPkg::Post-Invoke { \"rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true\"; };" > /etc/apt/apt.conf.d/docker-clean' \
 && /bin/sh -c 'echo "APT::Update::Post-Invoke { \"rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true\"; };" >> /etc/apt/apt.conf.d/docker-clean' \
 && /bin/sh -c 'echo "Dir::Cache::pkgcache \"\"; Dir::Cache::srcpkgcache \"\";" >> /etc/apt/apt.conf.d/docker-clean' \
 && /bin/sh -c 'echo "Acquire::Languages \"none\";" > /etc/apt/apt.conf.d/docker-no-languages' \
 && /bin/sh -c 'echo "Acquire::GzipIndexes \"true\"; Acquire::CompressionTypes::Order:: \"gz\";" > /etc/apt/apt.conf.d/docker-gzip-indexes' \
 && /bin/sh -c 'echo "Apt::AutoRemove::SuggestsImportant \"false\";" > /etc/apt/apt.conf.d/docker-autoremove-suggests'
