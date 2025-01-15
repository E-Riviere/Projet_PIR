from statemachine import State, StateMachine

class BehaviorSelector(StateMachine):

    follower = State('Follower', initial = False)
    explorator = State('Explorator', initial = True)

    become_explorator = follower.to(explorator)
    become_follower = explorator.to(follower)

    def __init__(self):
        self.first_determination = False
        super(BehaviorSelector, self).__init__()

    def on_transition(self, event, state):
        print(f"On '{event}', on the '{state.id}' state.")

    def initial_state_determination(self, semantic_data):
        if not self.first_determination:
            if len(semantic_data) > 0:
                if not self.current_state_value == 'follower':
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
        if 'red' in msg_data and not self.current_state_value == 'explorator':
            self.become_explorator()
        elif None in msg_data:
            self.first_determination = False
        else:
            pass
    
    def detection_state_determination(self, detection):
        if detection:
            if self.current_state_value == 'explorator':
                self.become_follower()
            else:
                pass
        else:
            pass