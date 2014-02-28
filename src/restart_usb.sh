#!/bin/bash

echo -n "0000:00:12.2" | sudo tee /sys/bus/pci/drivers/ehci-pci/unbind
echo
echo -n "0000:00:13.2" | sudo tee /sys/bus/pci/drivers/ehci-pci/unbind
echo
echo -n "0000:00:16.2" | sudo tee /sys/bus/pci/drivers/ehci-pci/unbind
echo
echo -n "0000:00:12.2" | sudo tee /sys/bus/pci/drivers/ehci-pci/bind
echo
echo -n "0000:00:13.2" | sudo tee /sys/bus/pci/drivers/ehci-pci/bind
echo
echo -n "0000:00:16.2" | sudo tee /sys/bus/pci/drivers/ehci-pci/bind
echo
