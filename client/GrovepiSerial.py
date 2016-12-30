#get Raspberry pi serial number
def getserial():
        #Extract serial from CPUinfo file
        global cpuserial
        try:
                #Read cpu file
                f = open('/proc/cpuinfo','r')
                for line in f:
                        #find out the serial info
                        if line[0:6] == 'Serial':
                                cpuserial = line[10:26]
                f.close()
        except IOError:
                cpuserial = "ERROR00000000000"
        return cpuserial
