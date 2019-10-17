/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Benders;

import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloRange;
import ilog.cplex.IloCplex;
import assignement2.UnitCommitmentProblem;
import ilog.concert.IloIntVar;

/**
 *
 * @author vtodd
 */
// We have chosen the u_gt's to be the complicating constraints.
// If these are fixed we will either have a linear problem with an optimal solution or no feasible solution.
public class MasterProblem {
    
    private final IloRange upTimeConstr[][];
    private final IloRange downTimeConstr[][];
    // These two constraints ((1c) and (1d)) we bring to the masterproblem as they are somewhat dependent
    // We need to ask Emma or G about why it makes so perfect sense to bring exactly these two to the MP
    // Maybe it's because everywhere else u_gt is there is also another variable
    // and therefore the cons goes to the SP - Victor
    private final IloCplex model;
    private final IloIntVar u[][];
    // Our only decesion variable in the MP (plus phi)
    private final IloNumVar phi;
    
    private final UnitCommitmentProblem UCP;
    
     public MasterProblem(UnitCommitmentProblem ucp) throws IloException {
        this.UCP = ucp;
        
        // 1. Every model needs an IloCplex object
        this.model = new IloCplex();
        
        // 2. Creates the decision variables
        // we have g generators and t hours, thus we create a matrix
        // of length nGenerators times nHours which will contain objects of type IloIntVar
        this.u = new IloIntVar[UCP.getnGenerators()][UCP.getnHours()];
        
        // Assigns to each position in the matrix an IloIntVar object which can take value 0 and 1.
        for(int g = 1; g<= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                u[g-1][t-1] = model.intVar(0,1 , "u"+g+t);
            }
        }
        
        // Here we create our phi needed for Bender's decomposition
        // Note here that 0 is the smallest value phi can take
        // as all the decision variables and all parameters of our original problem are non-negative.
        this.phi = model.numVar(0, Double.POSITIVE_INFINITY,"phi");
        
        // 3. Creates the objective function
        // Creates an empty linear numerical expression 
        IloLinearNumExpr obj = model.linearNumExpr();
        
        // Adds terms to the equation
        for(int i = 1; i <= UCP.getnGenerators(); i++){
            for(int j = 1; j <=UCP.getnHours(); j++){
            obj.addTerm(UCP.getCommitmentCost(i), u[i-1][j-1]);
            }
        }
        obj.addTerm(1, phi);
        
        // Tells cplex to minimize the objective function
        model.addMinimize(obj);
        
        // We now add (1c) to the MP. Note we have g times t constraints.
        upTimeConstr = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                // We loop over t'
                for(int tt = t; tt <= UCP.getMinOnTimeAtT(g, t); tt++){
                    lhs.addTerm(1, u[g-1][tt-1]);
                    lhs.addTerm(-1, u[g-1][t-1]);
                    if(t > 1){
                        lhs.addTerm(1, u[g-1][t-2]);
                    }
                }
                // Here we assign every constraint into our constraint matrix (IloRange)
                upTimeConstr[g-1][t-1] = model.addGe(lhs,0);
            }
        }
        // We now add (1d) to the MP.
        downTimeConstr = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                // We loop over t'
                // We add one every time we loop. We save this sum in a constant
                int constant = 0;
                for(int tt = t; tt <= UCP.getMinOffTimeAtT(g, t); tt++){
                    constant++;
                    lhs.addTerm(-1, u[g-1][tt-1]);
                    lhs.addTerm(1, u[g-1][t-1]);
                    if(t > 1){
                        lhs.addTerm(-1, u[g-1][t-2]);
                    }
                }
                // We make this right hand side as we cannot add a series of 1's in our lhs.addTerm
                // Note, lads, that I have changed this as in the UnitCommitmentModel.
                // Still think it should be -constant tho -Victor
                downTimeConstr[g-1][t-1] = model.addGe(lhs,constant);
            }
        }
        
        
    }
    public void solve() throws IloException{
        // Setting the output to null we suppress logs
        // regarding the (dual) simplex iterations
        model.use(new Callback());
        
        // Solves the problem
        model.solve();
    }
    private class Callback extends IloCplex.LazyConstraintCallback{
        
        public Callback() {
        }
        
        @Override
        protected void main() throws IloException {
            double[][] U = getU();
            double Phi = getPhi();
        
                //We create an instance of the feasibility subproblem
        FeasSubProblem fsp = new FeasSubProblem(UCP,U);
            fsp.solve();
                //We store the result of the feasibility subproblem
            double fspObjective = fsp.getObjective();
        
        
        System.out.println("FSP "+fspObjective);
            //We check if the feasibility problem is positive. If so we make a feasibility cut.
        if(fspObjective >= 0+1e-10){
            System.out.println("Generating feasibility cut!");
            //Here we store the dual variables of the feasibility subproblem
            double constant = fsp.generateFeasCutConstant();
            IloLinearNumExpr linearTerm = fsp.getCutLinearTerm(u);
            //Here we add the feasibility cut to our Master Problem
            add(model.le(linearTerm,-constant));
            
            //If the feasibility subproblem is zero we move on to solve the optimality subproblem
        }else{
            
            //We now create an instance of the optimality subproblem.
            OptSubProblem osp = new OptSubProblem(UCP,U);
                osp.solve();
                //Here we store the objective value of optimality subproblem
                double ospObjective = osp.getObjective();
                
            System.out.println("Phi "+Phi+ " OSP "+ospObjective );
                if(Phi >= ospObjective - 1e-9){
                    
                    //If the above inequality holds, we have found the optimal solution to the current node.
                    System.out.println("The current node is optimal");
                }else{
                    
                    //If the inequality does not hold we create an optimality cut.
                   
                    System.out.println("Generating optimality cut");
                    // We get the constant and the linear term from
                    // the optimality suproblem 
                    double cutConstant = osp.generateObtCutConstant();
                    IloLinearNumExpr cutTerm = osp.getCutLinearTerm(u);
                    cutTerm.addTerm(1, phi);
                    // and generate and add a cut. 
                    add(model.ge(cutTerm, cutConstant));
                }
        }
    }
        
                // getU stores the solution of the u_gts of our master problem
    public double[][] getU() throws IloException{
        double U[][] = new double[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g<= UCP.getnGenerators() ;g++){
            for(int t = 1; t <= UCP.getnHours() ; t++){
                U[g-1][t-1] = model.getValue(u[g-1][t-1]);
            }
        }
        return U;
    }
                // getPhi stores the solution of phi of our master problem
    public double getPhi() throws IloException{
        return model.getValue(phi);
        }
    
    }
    
            //This will give us the objective value of our master problem.
            //Our master problem includes phi. This is not a problem as we want to minimize phi in our MP
            //so the value of phi will correspond to our value of our optimality subproblem.
            //Therefore the solution will correspond to the optimal value of the original problem.
    public double getObjective() throws IloException{
        return model.getObjValue();
    }
    
        //This gives the price of having all the generators running during the day.
    public double getFirstStageCost() throws IloException{
        double cost = 0;
        for(int i = 1; i <= UCP.getnGenerators(); i++){
            for(int j = 1; j <= UCP.getnHours(); j++){
            cost += UCP.getCommitmentCost(i)* model.getValue(u[i-1][j-1]);
            }
        }    
        return cost;
    }
    
     public void printSolution() throws IloException{
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
            System.out.println("U_"+g+"_"+t+" = "+model.getValue((u[g-1][t-1])));    
            }
        }
    }  
    
    
    public void end(){
        model.end();
    }
    
}
