#!/bin/bash
sudo sysctl -w net.core.rmem_max=12582912
sudo sysctl -w net.core.wmem_max=12582912
