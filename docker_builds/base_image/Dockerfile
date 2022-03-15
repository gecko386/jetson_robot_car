ARG BASE_IMAGE="ros:galactic-pytorch-l4t-r32.6.1"
FROM ${BASE_IMAGE}

# generate entrypoint
COPY ./update_entrypoint.sh /tmp/.
RUN chmod 755 /tmp/update_entrypoint.sh
RUN /tmp/update_entrypoint.sh
RUN rm /tmp/update_entrypoint.sh

ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]
WORKDIR /