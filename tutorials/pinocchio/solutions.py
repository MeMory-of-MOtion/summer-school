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

### Solution to 2.2
cost6d = Cost6d(rmodel,rdata)
g6 = numdiff(cost6d.calc,q)
assert(norm(g6-cost6d.calcDiff(q))<1e-5)

### Solution to 2.3
cost3d = Cost3d(rmodel,rdata)
g3 = numdiff(cost3d.calc,q)
assert(norm(g3-cost3d.calcDiff(q))<1e-5)

### Solution to 2.4
costq = CostPostureDiff(rmodel,rdata)
gq = numdiff(costq.calc,q)
assert(norm(gq-costq.calcDiff(q))<1e-5)

### Solution to 2.5a
costg = CostGravity(rmodel,rdata)
gg = numdiff(costg.calc,q)
assert(norm(gg-costg.calcDiff(q))/norm(costg.calc(q))<1e-5)

### Solution to 2.5b
costMg = CostWeightedGravity(rmodel,rdata)
gMg = numdiff(costMg.calc,q)
assert(norm(gMg-costMg.calcDiff(q))/norm(costMg.calc(q))<1e-5)

### Solution to 2.6
class SumOfCostFreeFlyer:
    def __init__(self,rmodel,rdata,costs,weights):
        self.rmodel = rmodel
        self.costs = costs
        self.weights = np.array(weights)
    def calc(self,q):
        q = pin.normalize(self.rmodel,q)
        return sum(self.weights*[ cost.calc(q) for cost in self.costs] )
    def callback(self,q):
        q = pin.normalize(self.rmodel,q)
        for c in self.costs:
            if hasattr(c,'callback'): c.callback(q)
    def calcDiff(self,q):
        q = pin.normalize(self.rmodel,q)
        Tqc = sum([ w*cost.calcDiff(q) for (w,cost) in zip(self.weights,self.costs)] )
        Q = dExpQ_inv(self.rmodel,q)
        return Tqc@Q

costsum = SumOfCostFreeFlyer(rmodel,rdata,[ cost3d,costMg ],[1,1e-5])
gsum = numdiff(costsum.calc,q)
assert(norm(gsum-costsum.calcDiff(q))<1e-5)

