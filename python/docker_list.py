#!/usr/bin/python

from docker import Client
cli = Client(base_url='unix://var/run/docker.sock')

print("Containers:")
print(cli.containers())
print("Images:")
print(cli.images())
