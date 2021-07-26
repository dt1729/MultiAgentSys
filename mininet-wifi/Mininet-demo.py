from mininet.node import Controller
from mininet.log import setLogLevel, info
from mn_wifi.link import wmediumd, mesh
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mn_wifi.wmediumdConnector import interference


if __name__ == "__main__":
    print("Check comments to run 2 Access point configuration")
    net = Mininet_wifi(controller=Controller,link=wmediumd, wmediumd_mode=interference)
    sta1 = net.addStation('sta1')
    sta2 = net.addStation('sta2')
    ap1 = net.addAccessPoint('ap1', ssid='AP')
   #ap2 = net.addAccessPoint('ap2', ssid='AP1') uncomment this to start access point2
    c0 = net.addController('c0')


    #setting a propagation model for wifi simulation, locations to APs and STAs can also be addded for better understanding of this. 
    net.setPropagationModel(model="logDistance", exp=4.5)

    net.configureWifiNodes()

    net.addLink(sta1,ap1)
    net.addLink(sta2,ap1)

    #uncomment this to establish connections between stations and accesspoints
    #net.addLink(sta1,ap2)
    #net.addLink(sta1,ap2)

    #starting the network
    net.build()
    c0.start()
    ap1.start([c0])
    #ap2.start([c0])
   
    #starting command line to visualise the network
    CLI(net)

    net.stop()

