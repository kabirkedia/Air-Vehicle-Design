FROM ros:humble-perception-jammy

# setup timezone
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends \
    tzdata dirmngr gnupg2 emacs ros-humble-desktop ros-humble-rosbridge-suite  && \
    rm -rf /var/lib/apt/lists/*