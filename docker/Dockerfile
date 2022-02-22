# Hu Zhu, ROMA Lab, SIR, MEE, SUSTech
# email: zhuhu00@foxmail.com

FROM zhuhu/cuda102-ros-ubuntu18:0.1

RUN python --version

ENV DEBIAN_FRONTEND noninteractive

# # Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

WORKDIR /root