from statemachine import State
from statemachine import StateMachine
from abc import abstractmethod


class BrainModule():

    '''
    Les "request" sont utilisé par les modules pour demander quelque chose avec attente d'une réponse. 
    Quand une "request" est prise par le destinataire, le module emmetteur de la "request" ce met en attente de la réponse.
    Les modules peuvent aussi envoyer des messages sans attente de retour, soit parce qu'un autre module leur a fait une demande d'infos soit parce qu'il considère 
    qu'une action est à réaliser au vu des infos qu'il possède.
    
    '''

    def __init__(self, signature):
        super().__init__()
        self.signature = signature
        self.subscribers = {}
        self.recipient = []
        self.recieved_msgs = {}
        self.recieved_requests = {}
        self.possible_state = ()
        self.actual_state = None

    def create_link_with(self, module):
        self.subscribers[module.signature] = module
        module.subscribers[self.signature] = self
        # print(module.identifier, module.signature, module.subscribers)

    def send(self, author, recipient, title, *args):
        if recipient in self.subscribers:
            module = self.subscribers[recipient]
            msg = (author, *args)
            module.recieved_msgs[title] = msg
            # if self.identifier == 0:
            #     print(msg)
            #     print(self.identifier, author, "msgs", recipient, title, module.recieved_msgs)
            module.read_msg(title)

    def request(self, author, recipient, request):
        if recipient in self.subscribers:
            module = self.subscribers[recipient]
            module.recieved_requests[request] = author
            # if self.identifier == 0:
            #     print(self.identifier, author, "requests", recipient, request, module.recieved_requests)
            module.read_request(request)

    @abstractmethod
    def read_request(self):
        '''
        Methode à définir pour chaque module, elle explicite la manière dont sera lu la request par le module.
        '''
        pass

    @abstractmethod
    def read_msg(self):
        '''
        Methode à définir pour chaque module, elle explicite la manière dont sera lu le msg par le module.
        '''
        pass
        






















        # def take_recieved_msg(self, author, recipient, msg_data):
    #     '''Le msg a bien été récupéré'''
    #     self.msg_sent_to_me()
    #     if recipient == self.signature :
    #         self.read_recieved_msg(author, msg_data)
    #     elif self.signature == "module manager":
    #         self.transfer_msg(author, recipient, msg_data)

    # def take_recieved_request(self, author, request):
    #     pass


    # def subscription_request(self, author, target):
    #     '''Je fais une demande de souscription'''
    #     self.ask_for_something()
    #     target.take_recieved_msg(author, "subscription request")
    

    

        
    


    
        
 

        





    # def situation_tickle_brain_to_procedure(self, situation_name):
    #     if situation_name == "in stock":
    #         self.behavior.brain_tickle_procedure_for_situation(situation_name)


    # def procedure_tickle_brain_to_action(self, action_name):
    #     if action_name == "take the root":
    #         self.action.agent_entrance()
    #     if action_name == "stop at root":
    #         self.action.agent_stop_at_root()

    # def procedure_tickle_brain_to_role(self, procedure_request):
    #     if procedure_request == "change role to leader":
    #         self.role.First_drone()


    # def procedure_tickle_brain_to_situation(self, procedure_request):
    #     if procedure_request == "change situation to root":
    #         self.situation.brain_tickle_situation_for_procedure(procedure_request)

    
    # def action_tickle_brain_to_procedure(self, action_done):
    #     if action_done == "take root done":
    #         self.behavior.brain_tickle_procedure_for_action()

    # def control_tickle_brain(self, gps, root):

    #     self.situation.brain_tickle_situation_for_control()
    #     self.action.brain_tickle_action_for_control(gps, root)

    #     forwrard = self.action.forward
    #     lateral = self.action.lateral
    #     rotation = self.action.rotation
    #     return ((forwrard, lateral, rotation))






