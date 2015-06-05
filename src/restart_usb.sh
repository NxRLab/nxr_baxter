#!/bin/bash

echo -n "0000:00:12.2" | tee /sys/bus/pci/drivers/ehci-pci/unbind
echo
echo -n "0000:00:13.2" | tee /sys/bus/pci/drivers/ehci-pci/unbind
echo
echo -n "0000:00:16.2" | tee /sys/bus/pci/drivers/ehci-pci/unbind
echo
# echo -n "0000:02:00.0" | tee /sys/bus/pci/drivers/xhci_hcd/unbind
# echo
echo -n "0000:00:12.2" | tee /sys/bus/pci/drivers/ehci-pci/bind
echo
echo -n "0000:00:13.2" | tee /sys/bus/pci/drivers/ehci-pci/bind
echo
echo -n "0000:00:16.2" | tee /sys/bus/pci/drivers/ehci-pci/bind
echo
# echo -n "0000:02:00.0" | tee /sys/bus/pci/drivers/xhci_hcd/bind
# echo
