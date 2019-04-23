docker run -d -p 1883:1883 -p 9001:9001 --restart always \
       --log-driver json-file --log-opt max-size=10m --name mosquitto toke/mosquitto
       
docker run -d -p 8888:8888 --restart always --name chronograf chronograf

docker run -d -p 8083:8083 -p 8086:8086 --expose 8090 --expose 8099 \
        --log-opt max-size=10m --restart always --name influxdb influxdb

docker run -d -v $(pwd)/telegraf.conf:/etc/telegraf/telegraf.conf:ro \
        --log-opt max-size=10m --restart always --name telegraf telegraf

docker run -d -p 3000:3000 --link influxdb:influxdb \
    --log-opt max-size=10m --restart always --name grafana grafana/grafana
