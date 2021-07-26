"""Mininet-WiFi: A simple networking testbed for Wireless OpenFlow/SDWN!
   author: Ramon Fontes (ramonrf@dca.fee.unicamp.br)"""


class AssociationControl(object):
    "Mechanisms that optimize the use of the APs"

    changeAP = False

    def __init__(self, intf, ap_intf, ac):
        if ac in dir(self):
            self.__getattribute__(ac)(intf, ap_intf)

    def llf(self, intf, ap_intf):
        'LLF: Least Loaded First'
        apref = intf.associatedTo
        if apref:
            ref_llf = len(apref.associatedStations)
            if len(ap_intf.associatedStations) + 2 < ref_llf:
                intf.disconnect_pexec(apref)
                self.changeAP = True
        else:
            self.changeAP = True
        return self.changeAP

    def ssf(self, intf, ap_intf):
        'SSF: Strongest Signal First'
        dist = intf.node.get_distance_to(intf.associatedTo.node)
        rssi = intf.get_rssi(ap_intf, dist)
        ref_dist = intf.node.get_distance_to(ap_intf.node)
        ref_rssi = intf.get_rssi(ap_intf, ref_dist)
        if float(ref_rssi) > float(rssi + 0.1):
            intf.disconnect_pexec(ap_intf)
            self.changeAP = True
        return self.changeAP
