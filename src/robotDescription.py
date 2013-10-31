import roslib; roslib.load_manifest('george')
import rospy

###############################################################################################       
###############################################################################################       
class RobotDescription():
###############################################################################################       
###############################################################################################       

    ###########################################################       
    def __init__(self):
    ###########################################################       
        self.name = ""
        self.NAppendages = 0
        self.max_nservos = 0
        self.total_servos = 0
        self.appendages = []


    ###########################################################       
    def Print(self):
    ###########################################################       
        print "name: %s" % self.name
        print "NAppendages: %d" % self.NAppendages
        print "max_nservos: %d" % self.max_nservos
        print "total_servos: %d" % self.total_servos
        for i in range(self.NAppendages):
            self.appendages[i].Print()
    ###########################################################
    def str(self):       
    ###########################################################       
        str = "{\'name\' :%s " % self.name
        str = "%s, \'NAppendages\' : %d" % (str, self.NAppendages)
        str = "%s, \'max_nservos\' : %d" % (str, self.max_nservos)
        str = "%s, \'total_servos\' : %d" % (str, self.total_servos)
        str = "%s, \'Appendages\' : ["
        for i in range(self.NAppendages):
            str = "%s, {%s}" % (str, self.appendages[i].str())
        str = "%s], \'max_nservos\' : %d, \'total_servos\' : %d" % (str, self.max_nservos, self.total_servos)
        return str

    ###########################################################       
    def ReadParameters(self):        
    ###########################################################       
        self.name = rospy.get_param('robot_name')
        self.NAppendages = rospy.get_param('NAppendages')
        for i in range(self.NAppendages): 
            rospy.loginfo('Getting appendage #' + str(i))
            appendage_name = rospy.get_param('appendage_name' + str(i))
            appendage_nservos = rospy.get_param('nservos' + str(i))
            appendage_firstservo = rospy.get_param('firstservo' + str(i))
            jointnames_str = rospy.get_param('joints' + str(i))
            jointnames = jointnames_str.split(',')
            self.AddAppendage(appendage_name, appendage_nservos, appendage_firstservo, jointnames)
        rospy.loginfo("robot description:")
        rospy.loginfo(self.str())
    ###########################################################       
    def AddAppendage(self, appendage_name="", nservos=0, firstservo=0, jointnames=[]):
    ###########################################################       
        self.appendages.append(Appendage(appendage_name, nservos, firstservo, jointnames))
        self.total_servos += nservos
        if nservos > self.max_nservos:
            self.max_nservos = nservos
            
    
###############################################################################################       
###############################################################################################       
class Appendage():
###############################################################################################       
###############################################################################################       

    ###########################################################       
    def __init__(self, name="", nservos=0, firstservo=0, jointnames=[]):
    ###########################################################       
        self.name = name;
        self.nservos = nservos;
        self.firstservo = firstservo;
        self.jointnames = jointnames;
        
    ###########################################################       
    def Print(self):
    ###########################################################       
        print "appenage name: %s" % self.name
        print "  nservos: %d" % self.nservos
        print "  firstservo: %d" % self.firstservo
        print "  jointnames = %s" % self.jointnames

    ###########################################################       
    def str(self):
    ###########################################################       
        str = "appendage_name:%s" % self.name
        str = "%s, nservos:%d" % (str, self.nservos)
        str = "%s, firstservo:%d" % (str, self.firstservo)
        str = "%s, jointnames:%s" % (str, self.jointnames)
        
        return str