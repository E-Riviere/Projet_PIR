from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule

class Situation(BrainModule):
    # InStock = State("Sotck", initial=True)
    # AtRoot = State()
    # # InIntersection = State()
    # # InDeadEnd = State()
    # # AtRootAndIntersection = State()
    # # InBetweenTwoSituation = State()

    # Leader_Start_Explo = InStock.to(AtRoot)
    # # A=InBetweenTwoSituation.to(InIntersection)
    # # B=InBetweenTwoSituation.to(InDeadEnd)
    # # C=InBetweenTwoSituation.to(AtRoot)
    # # D=InBetweenTwoSituation.to(AtRootAndIntersection)
    # # E=AtRoot.to(InBetweenTwoSituation)
    # # F=InIntersection.to(InBetweenTwoSituation)
    # # G=InDeadEnd.to(InBetweenTwoSituation)
    # # H=AtRootAndIntersection.to(InBetweenTwoSituation)
    # # I=InStock.to(InBetweenTwoSituation)
    

    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier

        self.drone_situation = {
        "Stock" : True,
        "Root": False,
        "Intersection" : False,
        "Dead-end" : False,
        "Open space" : False,
        "Curve" : False,
        "Exploration possible" : False,
        "Exploration co;pleted" : False,
        "Visual connectivity" : False,
        "Critical visual connectivity" : False,
        "Too close" : False,
        "All branch explored" : False,
        "Collision" : False
        }



    def read_request(self):
        last_request = list(self.recieved_requests.items())[-1]
        if last_request[1] == "situation":
            self.request(self.signature, "module manager", "Need sensors analyze")
    
    def read_msg(self):
        last_msg = list(self.recieved_msgs.items())[-1]
        if last_msg[1][0] == "analyzed data":
            self.situation_determination(last_msg)
            self.send(self.signature, "module manager", ["drone situation", self.drone_situation])

    def situation_determination(self, analyzed_data):
        pass










    # def brain_tickle_situation_for_control(self):
    #     if self.current_state.id == "InStock":
    #         self.brain.situation_tickle_brain_to_procedure("in stock") 
    
    # def brain_tickle_situation_for_procedure(self, procedure_request):
    #     if procedure_request == "change situation to root":
    #         self.Leader_Start_Explo()



    # def on_enter_InStock(self):
    #     print("in stock")
    # def on_enter_AtRoot(self):
    #     print(self.identifier, "at root")

    




    