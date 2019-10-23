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

// We have chosen the u_gt's to be the complicating variables.
// If these are fixed we will either have a linear problem (such as a well known Transportation Problem)
// or several single-variable optimization problems, with an optimal solution or no feasible solution.
public class MasterProblem{
    
    private final IloRange upTimeConstr[][];
    private final IloRange downTimeConstr[][];
    // These two constraints ((1c) and (1d)) we bring to the masterproblem as they are they are the only
    // ones depending only on u.
    // Note that we could also have included constraint 1b in the Master Problem, but it doesn't make a
    // difference if we put it here or in the subproblems (as this is only there to active startup costs).
    // This because it is the only constraint containing the variable c.
    
    private final IloCplex model;
    private final IloIntVar u[][];
    // Our only decision variable in the MP (plus phi)
    private final IloNumVar phi;
    
    //We need an instance of the UnitCommitmentProblem
    private final UnitCommitmentProblem UCP;
    
    public MasterProblem(UnitCommitmentProblem ucp) throws IloException {
        this.UCP = ucp;
        
        
        this.model = new IloCplex();
        
        // Creates the decision variables
        // we have g generators and t hours, thus we create a matrix
        // of length nGenerators times nHours which will contain objects of type IloIntVar
        // (we could also have used booleans here as the u's are binary variables)
        this.u = new IloIntVar[UCP.getnGenerators()][UCP.getnHours()];
        
        // Assigns to each position in the matrix an IloIntVar object which can take value 0 and 1.
        for(int g = 1; g<= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                u[g-1][t-1] = model.intVar(0,1 , "u"+g+"_"+t);
            }
        }
        
        // Here we create our phi needed for Bender's decomposition
        // Note here that 0 is the smallest value phi can take
        // as all the decision variables and all parameters of our original problem are non-negative.
        this.phi = model.numVar(0, Double.POSITIVE_INFINITY,"phi");
        
        // Here we create the objective function
        IloLinearNumExpr obj = model.linearNumExpr();
        
        // We add terms to the equation
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <=UCP.getnHours(); t++){
                //We add the complicating variable times its coefficient for each g and t
                obj.addTerm(UCP.getCommitmentCost(g), u[g-1][t-1]);
            }
        }
        // We also add phi to compensate for the rest of the decision variables
        obj.addTerm(1, phi);
        
        // Tells cplex to minimize the objective function
        model.addMinimize(obj);
        
        
        // Now we add constraints to the MP:
        
        // We add (1c) to the MP. Note we have g times t constraints. We save them all to keep track of
        // them later when we solve the dual problem
        upTimeConstr = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                // We loop over t'
                for(int tt = t; tt <= UCP.getMinOnTimeAtT(g, t); tt++){
                    lhs.addTerm(1, u[g-1][tt-1]);
                    lhs.addTerm(-1, u[g-1][t-1]);
                    //we neglect u_{g,0} as we assume that it is 0. We add u_{g,t-2} for t>1.
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
                // We add one every time we loop over t'. We save this sum in a constant
                // Note that we reset the constant to 0 every time we loop over t and avoid a large
                // constant for a large t.
                int constant = 0;
                for(int tt = t; tt <= UCP.getMinOffTimeAtT(g, t); tt++){
                    constant++;
                    lhs.addTerm(-1, u[g-1][tt-1]);
                    lhs.addTerm(1, u[g-1][t-1]);
                    // Same as 1c
                    if(t > 1){
                        lhs.addTerm(-1, u[g-1][t-2]);
                    }
                }
                // In each expression we move the constant term to the right-hand-side and therefore get
                // a minus.
                downTimeConstr[g-1][t-1] = model.addGe(lhs,-constant);
            }
        }
    }
    
// We want to solve the MP
    public void solve() throws IloException{
        // We solve the model using the below defined Callback
        model.use(new Callback());
        
        // Solves the problem
        model.solve();
    }
    
    
    // Create the Callback
    private class Callback extends IloCplex.LazyConstraintCallback{
        
        public Callback() {
        }
        
        // Here we define what should be done every time we reach a new integer node in the B&B tree. 
        @Override
        protected void main() throws IloException {
            // We obtain the solution of our variables in the current node
            double[][] U = getU();
            double Phi = getPhi();
        
            //We create an instance of the feasibility subproblem and solve it
            FeasSubProblem fsp = new FeasSubProblem(UCP,U);
            fsp.solve();
            //We store the result of the objective function of the feasibility subproblem
            double fspObjective = fsp.getObjective();
            System.out.println("FSP = "+fspObjective);
 
            //We check if the objective function value of the feasibility problem is positive.
            //If so we make a feasibility cut.
        if(fspObjective >= 0+1e-6){
            System.out.println("Generating feasibility cut!");
            // We make the feasibility cut by getting the constant and linear term from the subproblem
            double constant = fsp.generateFeasCutConstant();
            IloLinearNumExpr linearTerm = fsp.getCutLinearTerm(u);
            //Here we add the feasibility cut to our Master Problem
            IloRange cut = model.le(linearTerm, -constant);
            add(cut);
            
            //If the feasibility subproblem is zero we move on to solve the optimality subproblem
        }
        else{
            //We now create an instance of the optimality subproblem and solves it
            OptSubProblem osp = new OptSubProblem(UCP,U);
            osp.solve();
            
            //Here we store the objective value of the optimality subproblem
            double ospObjective = osp.getObjective();
            
            //We print the values of Phi and the objective value
            System.out.println("Phi = "+Phi+ " and "+ " OSP "+ospObjective );
            
            //If the below inequality holds, we have found the optimal solution to the current node.
            if(Phi >= ospObjective - 1e-9){
                System.out.println("The current node is optimal");
                
                // We also print the generators that are turned on/off during the hours
                System.out.println("These generators are turned on during these hours:");
                for(int g = 1; g <= UCP.getnGenerators(); g++){
                    System.out.print(UCP.getGeneratorName(g));
                    for(int t =1; t <= UCP.getnHours(); t++){
                        System.out.print(String.format(" %4.0f ", U[g-1][t-1]));
                    }
                System.out.println("");
                }
                // We print the amount of energy each generator produces in each hour
                System.out.println("Our generators produce this amount of energy during these hours:");
                for(int g = 1; g <= UCP.getnGenerators(); g++){
                    System.out.print(UCP.getGeneratorName(g));
                    for(int t =1; t <= UCP.getnHours(); t++){
                        System.out.print(String.format(" %4.0f ", osp.getPSolution()[g-1][t-1]));
                    }
                System.out.println("");
                }
                
                //If the inequality does not hold we create an optimality cut.
                }else{
                    
                    System.out.println("Generating optimality cut");
                    // We get the constant and the linear term from
                    // the optimality subproblem 
                    double cutConstant = osp.generateObtCutConstant();
                    IloLinearNumExpr cutTerm = osp.getCutLinearTerm(u);
                    //We isolate the constant bringing phi to the other side of the inequality
                    cutTerm.addTerm(-1, phi);
                    // and generate and add a cut. 
                    IloRange cut = model.le(cutTerm,-cutConstant);
                    add(cut);
                    
                }
        }
    }
        
    // getU stores the solution of the u_gts of our master problem at the current integer node
    public double[][] getU() throws IloException{
    double U[][] = new double[UCP.getnGenerators()][UCP.getnHours()];
    for(int g = 1; g<= UCP.getnGenerators() ;g++){
        for(int t = 1; t <= UCP.getnHours() ; t++){
            U[g-1][t-1] = getValue(u[g-1][t-1]);
        }
    }
    return U;
    }

    // getPhi stores the solution of phi of our master problem at the current integer node
    public double getPhi() throws IloException{
        return getValue(phi);
    }
    }
//This will give us the objective value of our master problem.
//Our master problem includes phi. This is not a problem as we want to minimize phi in our MP
//so the value of phi will correspond to our value of our optimality subproblem.
//Therefore the solution will correspond to the optimal value of the original problem.
public double getObjective() throws IloException{
    return model.getObjValue();
}
    
     
//We also print the solution of the u's:
public void printSolution() throws IloException{
    System.out.println("These generators are turned on during these hours:");
    for(int g = 1; g <= UCP.getnGenerators(); g++){
        System.out.print(UCP.getGeneratorName(g));
        for(int t =1; t <= UCP.getnHours(); t++){
            System.out.print(String.format(" %4.0f ", model.getValue(u[g-1][t-1])));
            }
            System.out.println("");
        }
        

    }
}