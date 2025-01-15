import math

from statemachine import State, StateMachine

class SituationSelector(StateMachine):

    open_space = State('Open-space', initial = True)
    corridor = State('Corridor', initial = False)
    intersection = State('Intersection', initial = False)
    curve = State('Curve', initial = False)
    dead_end = State('Dead-end', initial = False)

    new_situation_open_space = open_space.from_(corridor, intersection, curve, dead_end)
    new_situation_corridor = corridor.from_(open_space, intersection, curve, dead_end)
    new_situation_intersection = intersection.from_(open_space, corridor, curve ,dead_end)
    new_situation_curve = curve.from_(open_space, corridor, intersection, dead_end)
    new_situation_dead_end = dead_end.from_(open_space, corridor, intersection)

    def __init__(self):
        self.gap_analysis = None
        self.gap_number = None
        super(SituationSelector, self).__init__()

    # def before_transition(self, event, state):
    #     print(f"Before '{event}', on the '{state.id}' state.")

    # def on_transition(self, event, state):
    #     print(f"On '{event}', on the '{state.id}' state.")

    def after_transition(self, event, state):
        print(f"After '{event}', on the '{state.id}' state.")

    def AgentSituation(self):
        GAP_size = self.gap_analysis[2]
        Inter_gap = self.gap_analysis[3]

        if self.gap_number == 1:
            if GAP_size[0] > math.pi * 2/3:
                if not self.current_state_value == 'open_space':
                    self.new_situation_open_space()
            else:
                if not self.current_state_value == 'dead_end':
                    self.new_situation_dead_end()
        if self.gap_number == 2:
            if GAP_size[0] > math.pi * 2/3 or GAP_size[1] > math.pi * 2/3:
                if not self.current_state_value == 'open_space':
                    self.new_situation_open_space()
            elif Inter_gap[0] < math.pi * 3/4:
                if not self.current_state_value == 'curve':
                    self.new_situation_curve()
            else:
                if not self.current_state_value == 'corridor':
                    self.new_situation_corridor()
        if self.gap_number > 2:
            if not self.current_state_value == 'intersection':
                self.new_situation_intersection()
    
