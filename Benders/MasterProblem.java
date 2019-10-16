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
public class MasterProblem {
    
    private final IloRange upTimeConstr[][];
    private final IloRange downTimeConstr[][];
    private final IloCplex model;
    private final IloIntVar u[][];
    private final IloNumVar phi;
    
    private final UnitCommitmentProblem UCP;
    
     public MasterProblem(UnitCommitmentProblem ucp) throws IloException {
        this.UCP = ucp;
        
        // 1. Every model needs an IloCplex object
        this.model = new IloCplex();
        
        // 2. Creates the decision variables
        // we have three plants, thus we create an array
        // of length nFacilities which will contain objects of type IloNumVar
        this.u = new IloIntVar[UCP.getnGenerators()][UCP.getnHours()];
        
        // Assigns to each position in the array an IloNumVar object 
        // i.e., a decisio variable in Cplex's language
        for(int g = 1; g<= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                u[g-1][t-1] = model.intVar(0,1 , "u"+g+t);
            }
        }
        
        this.phi = model.numVar(0, Double.POSITIVE_INFINITY,"phi");
        
        // 3. Creates the objective function
        // Creates an empty linear numerical expression (linear equation)
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
                upTimeConstr[g-1][t-1] = model.addGe(lhs,0);
            }
        }
        
        downTimeConstr = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                // We loop over t'
                for(int tt = t; tt <= UCP.getMinOffTimeAtT(g, t); tt++){
                    lhs.addTerm(-1, u[g-1][tt-1]);
                    lhs.addTerm(+1, u[g-1][t-1]);
                    if(t > 1){
                        lhs.addTerm(-1, u[g-1][t-2]);
                    }
                }
                // We make this right hand side as we cannot at a series of 1's in our lhs.addTerm
                downTimeConstr[g-1][t-1] = model.addGe(lhs,-UCP.getMinOffTimeAtT(g,t)+t-1);
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
        
        
        FeasSubProblem fsp = new FeasSubProblem(UCP,U);
            fsp.solve();
            double fspObjective = fsp.getObjective();
        
        
        System.out.println("FSP "+fspObjective);
        if(fspObjective >= 0+1e-10){
            System.out.println("Generating feasibility cut!");
            double constant = fsp.generateFeasCutConstant();
            IloLinearNumExpr linearTerm = fsp.getCutLinearTerm(u);
            add(model.le(linearTerm,-constant));
        }else{
            OptSubProblem osp = new OptSubProblem(UCP,U);
                osp.solve();
                double ospObjective = osp.getObjective();
                
            System.out.println("Phi "+Phi+ " OSP "+ospObjective );
                if(Phi >= ospObjective - 1e-9){
                    // 3.3. In this case the problem at the current node
                    // is optimal.
                    System.out.println("The current node is optimal");
                }else{
                    System.out.println("Generating optimality cut");
                    // We get the constant and the linear term from
                    // the optimality suproblem 
                    double cutConstant = osp.generateObtCutConstant();
                    IloLinearNumExpr cutTerm = osp.getCutLinearTerm(u);
                    cutTerm.addTerm(-1, phi);
                    // and generate and add a cut. 
                    add(model.le(cutTerm, -cutConstant));
                }
        }
    }
   
    public double[][] getU() throws IloException{
        double U[][] = new double[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g<= UCP.getnGenerators() ;g++){
            for(int t = 1; t <= UCP.getnHours() ; t++){
                U[g-1][t-1] = model.getValue(u[g-1][t-1]);
            }
        }
        return U;
    }
   
    public double getPhi() throws IloException{
        return model.getValue(phi);
        }
    
    }
    
    public double getObjective() throws IloException{
        return model.getObjValue();
    }
    /**
     * Returns the cost of the first stage part of the solution only.
     * @return the cost of the x solution.
     * @throws IloException 
     */
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
            System.out.println("U_"+g+t+" = "+model.getValue((u[g-1][t-1])));    
            }
        }
    }  
    
    
    public void end(){
        model.end();
    }
    
}
