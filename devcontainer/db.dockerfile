FROM postgres:15-alpine

ENV POSTGRES_USER=postgres
ENV POSTGRES_PASSWPRD=postgres
ENV POSTGRES_HOST_AUTH_METHOD=trust

RUN echo "PS1='\u@\h(\$(hostname -i)):\w \\$ '" >> ~/.bashrc
