to add read_serial daemon

cp read_serial.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable read_serial.service
