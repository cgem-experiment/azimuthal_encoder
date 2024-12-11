When first installing cgem_az_encoder.py running as a service run the following commands:

- Create the folder `/az_encoder`
``sudo mkdir -p /az_encoder``

-Give it the appropiate permissions:

``sudo chmod 755 /az_encoder``

-Place in the folder above the `config.json` file. 

-Place the `cgem_az_encoder.service` file on the `/etc/systemd/system` folder and activate the system:

``sudo systemctl daemon-reload``

``sudo systemctl start cgem_az_encoder.service``

``sudo systemctl enable cgem_az_encoder.service``

``sudo systemctl status cgem_az_encoder.service``



