#! /bin/sh
### BEGIN INIT INFO
# Provides:          rtnet
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: rtnet
# Description:       rtnet
### END INIT INFO

RTNET_MOD_DIR=/usr/local/rtnet/modules
RTNET_BIN_DIR=/usr/local/rtnet/sbin

ETH_IFACE=eth1

#ETH_DRV=e100
#RT_ETH_DRV=rt_eepro100
ETH_DRV=e1000e
RT_ETH_DRV=rt_$ETH_DRV

# PCI addresses of RT-NICs to claim (format: 0000:00:00.0)
#   If both Linux and RTnet drivers for the same hardware are loaded, this
#   list instructs the start script to rebind the given PCI devices, detaching
#   from their Linux driver, attaching it to the RT driver above. Example:
#   REBIND_RT_NICS="0000:00:19.0 0000:01:1d.1"
#REBIND_RT_NICS=$(ethtool -i $ETH_IFACE | grep bus-info | cut -d ' ' -f 2)
REBIND_RT_NICS="0000:02:00.0"
#REBIND_RT_NICS=""
echo "Rebind pci " $REBIND_RT_NICS

SUBNET_IP=169.254.89
#SUBNET_IP=192.168.1
LOCAL_IP=$SUBNET_IP.1
NETMASK=255.255.255.0

BASE_IP_ADDR=20
NUM_OF_REMOTE_BOARDS=200

. /lib/lsb/init-functions

case "$1" in
  start)
    log_daemon_msg "Loading rtnet modules ... " 
    export PATH=$PATH:$RTNET_BIN_DIR
    if [ ! -e /dev/rtnet ]; then    
        mknod /dev/rtnet c 10 240
        chgrp xenomai /dev/rtheap
    fi
    insmod $RTNET_MOD_DIR/rtnet.ko
    insmod $RTNET_MOD_DIR/rtipv4.ko
    insmod $RTNET_MOD_DIR/rtudp.ko
    insmod $RTNET_MOD_DIR/rttcp.ko
    insmod $RTNET_MOD_DIR/rtpacket.ko
    insmod $RTNET_MOD_DIR/rt_loopback.ko
    insmod $RTNET_MOD_DIR/$RT_ETH_DRV.ko
    ifdown $ETH_IFACE 
    for dev in $REBIND_RT_NICS; do
        if [ -d /sys/bus/pci/devices/$dev/driver ]; then
            echo $dev > /sys/bus/pci/devices/$dev/driver/unbind
        fi
        echo $dev > /sys/bus/pci/drivers/$RT_ETH_DRV/bind
    done 
    
    $RTNET_BIN_DIR/rtifconfig rtlo up 127.0.0.1
    $RTNET_BIN_DIR/rtifconfig rteth0 up $LOCAL_IP netmask $NETMASK
    #sleep 1
    $RTNET_BIN_DIR/rtroute del $SUBNET_IP.255
    $RTNET_BIN_DIR/rtroute add 255.255.255.255 ff:ff:ff:ff:ff:ff dev rteth0
    echo
    echo "Scan subnet ...."
    for i in $(seq $BASE_IP_ADDR $(($BASE_IP_ADDR+$NUM_OF_REMOTE_BOARDS))); do
      $RTNET_BIN_DIR/rtroute solicit $SUBNET_IP.$i dev rteth0
      sleep 0.1      
      rtroute | grep "$SUBNET_IP.$i"
    done
    echo
    echo "Dump RTnet routing table"
    cat /proc/rtnet/ipv4/host_route
    log_end_msg $?
  ;;

  stop)
    log_daemon_msg "Unloading rtnet modules ... "
    $RTNET_BIN_DIR/rtifconfig rtlo down 2> /dev/null
    $RTNET_BIN_DIR/rtifconfig rteth0 down 2> /dev/null
    rmmod $RT_ETH_DRV 2> /dev/null
    rmmod rt_loopback 2> /dev/null
    rmmod rtpacket 2> /dev/null
    rmmod rttcp 2> /dev/null
    rmmod rtudp 2> /dev/null
    rmmod rtipv4 2> /dev/null
    rmmod rtnet 2> /dev/null
    for dev in $REBIND_RT_NICS; do
        #echo $dev > /sys/bus/pci/devices/$dev/driver/unbind
        #echo 1 > /sys/bus/pci/devices/$dev/remove
        echo $dev > /sys/bus/pci/drivers/$ETH_DRV/bind
    done
    ifup $ETH_IFACE
    log_end_msg $?
  ;;
  restart|force-reload)
    $0 stop
    sleep 1
    $0 start
  ;;
  status)
    lsmod
  ;;
  *)
    echo "Usage: $0 {start|stop|status}"
    exit 1
  ;;
esac

exit 0
