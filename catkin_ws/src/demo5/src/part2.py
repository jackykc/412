
class Task3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go'])
        self.twist = Twist()
    def execute(self, data):
        
        return 'go'

# follower = Follower()
sm = smach.StateMachine(outcomes=['finish'])
with sm:
    # Add states to the container
    smach.StateMachine.add('GO', Go(), 
                 transitions={'stop':'STOP'})
    smach.StateMachine.add('STOP', Stop(), 
                 transitions={'go':'GO', 'task1': 'TASK1', 'task2': 'TASK2', 'task3': 'TASK3'})
    smach.StateMachine.add('TASK1', Task1(), 
                 transitions={'go':'GO'})
    smach.StateMachine.add('TASK2', Task2(), 
                 transitions={'go':'GO'})
    smach.StateMachine.add('TASK3', Task3(), 
                 transitions={'go':'GO'})

# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

outcome = sm.execute()
rospy.spin()
sis.stop()