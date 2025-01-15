from statemachine import State, StateMachine

class ControllerSelector(StateMachine):

    stop = State('Stop', initial = True)
    follow_the_gap =State('Follow', initial = False)
    alignement_with_the_gap = State('Aligne', initial = False)
    centering_in_the_intersection = State('Centering', initial = False)
    rushing_in_the_gap = State('Rushing', initial = False)
    follow_the_explorator = State('FollowExplorator', initial = False)

    is_in_intersection = follow_the_gap.to(centering_in_the_intersection, cond = "Intersection_Detected")
    is_centered = centering_in_the_intersection.to(alignement_with_the_gap, cond = "Is_Centered")
    is_aligned = alignement_with_the_gap.to(rushing_in_the_gap, cond = "Is_Aligned")
    turn_around = follow_the_gap.to(alignement_with_the_gap, cond = "Dead_end_Detected")
    change_follow_explorator = follow_the_gap.to(follow_the_explorator)
    change_follow_follower = follow_the_explorator.to(follow_the_gap)
    move_follower = stop.to(follow_the_explorator)
    move_explorator = stop.to(follow_the_gap)
    STOP = stop.from_(stop, follow_the_gap, alignement_with_the_gap, centering_in_the_intersection, rushing_in_the_gap, follow_the_explorator)

    intersection_cycle = (is_in_intersection|
                          is_centered|
                          is_aligned
                          )
    
    dead_end_cycle = (turn_around|
                      is_aligned
                      )
    
    def __init__(self):
        self.visual_connectivity = None
        self.critical_visual_connectivity = None
        self.behavior = None
        self.gap_analysis = None
        self.negative_gap_analysis = None
        self.situation = None
        self.gap_selected = None
        super(ControllerSelector, self).__init__()

    # def before_transition(self, event, state):
    #     print(f"Before '{event}', on the '{state.id}' state.")

    # def on_transition(self, event, state):
    #     print(f"On '{event}', on the '{state.id}' state.")

    def after_transition(self, event, state):
        print(f"After '{event}', on the '{state.id}' state.")


    def Is_Centered(self):
        Obst_dist_ray = self.negative_gap_analysis[1]
        passage = False
        if Obst_dist_ray[-1][1] - Obst_dist_ray[1][1] < -0.5:
            passage = True
        elif Obst_dist_ray[-1][1] - Obst_dist_ray[1][1] > 0.5:
            passage = True

        if Obst_dist_ray[0][1] - Obst_dist_ray[-1][1] < - 0.5:
            passage = True
        elif Obst_dist_ray[0][1] - Obst_dist_ray[-1][1] < 0.5:
            passage = True
            
        if not passage:
            return True
        else:
            return False

    def Is_Aligned(self):
        direction = self.gap_analysis[1][self.gap_selected]
        if direction < 0.05 and direction > -0.05:
            return True
        else:
            return False
    
    def Intersection_Detected(self):
        if self.situation == 'intersection':
            return True
        else:
            return False
    
    def Dead_end_Detected(self):
        if self.situation == 'dead_end':
            return True
        else:
            return False
        
    def controller_selection(self):
        if self.situation == 'intersection':
            try:
                self.intersection_cycle()
            except Exception as e:
                pass


        elif self.situation == 'dead_end':
            try:
                self.dead_end_cycle()
            except Exception as e:
                pass

        if self.behavior == 'explorator' and self.critical_visual_connectivity == 'True' :
            self.STOP()
        elif self.behavior == 'follower':
            self.current_state_value = 'follow_the_explorator'
        elif self.situation == 'corridor':
            self.current_state_value = 'follow_the_gap'
        elif self.situation == 'open_space':
            self.current_state_value = 'follow_the_gap'
        elif self.situation == 'curve':
            self.current_state_value = 'follow_the_gap'


        
        


    