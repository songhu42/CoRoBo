class ServiceDao():
    def __init__(self):
        self.srv_seq = 0
        self.srv_id = 0
        self.srv_nm = "" 
        self.srv_desc = ""
        self.srv_call = ""
        # dur : mili seconds 
        self.srv_dur = 0
        self.pre_dur = 0
        self.aft_dur = 0
        self.map_tp = ""
        self.parm = []

    def get_parm_val(self, parm_id): 
        for parm in self.parm:
            if parm.parm_id == parm_id:
                return parm.parm_val
        
        return "Not Exist"
    