ARG BUILD_FROM=python:3.11-alpine
FROM $BUILD_FROM

RUN pip3 install --no-cache-dir pyserial paho-mqtt

COPY jbd_bms_mqtt.py /jbd_bms_mqtt.py
COPY run.sh /run.sh
RUN chmod +x /run.sh

CMD ["/run.sh"]
