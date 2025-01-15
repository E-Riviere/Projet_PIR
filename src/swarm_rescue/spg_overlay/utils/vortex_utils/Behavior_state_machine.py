import math
from statemachine import State, StateMachine

class BehaviorSelector(StateMachine):

    follower = State('Follower', initial = False)
    explorator = State('Explorator', initial = True)

    become_explorator = follower.to(explorator)
    become_follower = explorator.to(follower)

    def __init__(self):
        self.first_determination = False
        super(BehaviorSelector, self).__init__()

    def on_enter_follower(self):
        print("je deviens follower")
    def on_exit_follower(self):
        print("je ne suis plus follower")
    def on_enter_explorator(self):
        print("je deviens explorator")
    def on_exit_explorator(self):
        print("je ne suis plus explorator")

    def initial_state_determination(self, semantic_data):
        if not self.first_determination:
            if len(semantic_data) > 0:
                angle_data = semantic_data[2]
                detection = False
                for data in angle_data:
                    if data > -math.pi * 1/2 and data< math.pi * 1/2:
                        detection = True
                if not detection:
                    self.explorator.to.itself()
                    self.first_determination = True
                if detection and not self.current_state_value == 'follower':
                    self.become_follower()
                    self.first_determination = True

                else:
                    self.follower.to.itself()
                    self.first_determination = True

            else:
                self.explorator.to.itself()
                self.first_determination = True
        else:
            pass
    
    def msg_state_determination(self, msg_data):
        if msg_data is not None:
            if 'red' in msg_data and not self.current_state_value == 'explorator':
                self.become_explorator()
            elif None in msg_data:
                self.first_determination = False
            else:
                pass
        else:
            pass
        
    def situation_state_determination(self, situation):
        if situation == 'dead_end':
            if self.current_state_value == 'explorator':
                self.become_follower()
            else:
                pass
        else:
            pass