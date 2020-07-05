### Solution to 1.8
class SumOfCostFreeFlyer:
    def __init__(self,rmodel,rdata,costs,weights):
        self.rmodel = rmodel
        self.costs = costs
        self.weights = np.array(weights)
    def calc(self,q):
        q = pin.normalize(self.rmodel,q)
        ### Alternatively, add this extra term to the sum
        extra = sum( (q-pin.normalize(rmodel,q))**2 )*100
        return sum(self.weights*[ cost.calc(q) for cost in self.costs] ) #+extra
    def callback(self,q):
        q = pin.normalize(self.rmodel,q)
        for c in self.costs:
            if hasattr(c,'callback'): c.callback(q)
    
mycost = SumOfCostFreeFlyer(rmodel,rdata,
                            [Cost3d(rmodel,rdata,ptarget = np.array([2,2,-1]), viz=viz),
                             CostPosture(rmodel,rdata)],[1,1e-3])
qopt = fmin_bfgs(mycost.calc,robot.q0,callback=mycost.callback)    
