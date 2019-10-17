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
public class OptSubProblem {
 
    
    
    private final IloCplex model;
    private final IloNumVar c[][];
    private final IloNumVar p[][];
    private final IloNumVar l[];
    private final UnitCommitmentProblem UCP;
    private final IloRange startUpConstraints[][];
    private final IloRange demandConstraints[];
    private final IloRange lowProdConstraints[][];
    private final IloRange upProdConstraints[][];
    private final IloRange upRampConstraints[][];
    private final IloRange lowRampConstraints[][];
    
    public OptSubProblem(UnitCommitmentProblem cpp, double U[][]) throws IloException {
        this.UCP = cpp;
        
        
        // 1. Every model needs an IloCplex object
        this.model = new IloCplex();
        
        // 2. Creates the decision variables and how many we need of each
        c = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        p = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        l = new IloNumVar[UCP.getnHours()];
        
        // Here we tell java that our variables should be numeric variables and should be non-negative.
        for(int j = 1; j <= UCP.getnHours(); j++){
                for(int i = 1; i<= UCP.getnGenerators(); i++){
                c[i-1][j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                p[i-1][j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
            }
        l[j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
        }
        
        
        // 3. Creates the objective function
        // Creates an empty linear numerical expression (linear equation)
        IloLinearNumExpr obj = model.linearNumExpr();
        // Adds terms to the equation
        
           for(int t = 1; t <= UCP.getnHours(); t++){
                for(int g = 1; g <= UCP.getnGenerators(); g++){
                    obj.addTerm(1, c[g-1][t-1]);    
                    obj.addTerm(UCP.getProductionCost(g), p[g-1][t-1]);
            }
            // 3. Adds the shedding costs
            // This is outside the g loop as it has nothing to do with the generators.
            obj.addTerm(UCP.getSheddingCost(t), l[t-1]);
        }
        
        // 4. Adds the objective function to the model
        model.addMinimize(obj);
 
        
        
        // We now make the constraints for the optimality sub problem
        // These are all the constraints from the original problem that are not in the master problem.
        // This is (1b)
        this.startUpConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, c[g-1][t-1]);
                // We bring the right-hand-side to the lhs, changing sign
                                //This will save our code from crashing
                if(t > 1){
                    startUpConstraints[g-1][t-1] = model.addGe(lhs, UCP.getStartUpCost(g)*(U[g-1][t-1]-U[g-1][t-2]));
                }else{
                    startUpConstraints[g-1][t-1] = model.addGe(lhs , UCP.getStartUpCost(g)*U[g-1][t-1]);
                }
                
            }
        }
        
        // This is (1e)
        this.demandConstraints = new IloRange[UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            IloLinearNumExpr lhs = model.linearNumExpr();
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                lhs.addTerm(1, p[g-1][t-1]);
            }
            lhs.addTerm(1, l[t-1]);
            demandConstraints[t-1] = model.addEq(lhs, UCP.getDemand(t));
        }
        
        // This is (1f)
        this.lowProdConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1,p[g-1][t-1]);
                lowProdConstraints[g-1][t-1] = model.addGe(lhs, UCP.getMinProduction(g)*U[g-1][t-1]);
                
            
            }
        }
        
        // This is (1g)
        this.upProdConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1,p[g-1][t-1]);
                upProdConstraints[g-1][t-1] = model.addLe(lhs,UCP.getMaxProduction(g)*U[g-1][t-1]);
            }
        }
        
        // This is (1h)
        this.upRampConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1,p[g-1][t-1]);
                if(t > 1){
                    lhs.addTerm(-1,p[g-1][t-2]);
                }
                upRampConstraints[g-1][t-1] = model.addLe(lhs,UCP.getRampUp(g));
            }
        }
        
        // This is (1i)
        this.lowRampConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1,p[g-1][t-1]);
                if(t > 1){
                    lhs.addTerm(-1,p[g-1][t-2]);
                }
                lowRampConstraints[g-1][t-1] = model.addLe(lhs,UCP.getRampUp(g));
            }
        }
        
        
    }
    
    // Here we reset the problem and solve it
    public void solve() throws IloException{
        model.setOut(null);
        model.solve();
    }
    
    // If our solution is optimal we are done in this node. If not we need to create an optimality
    //  from the dual variables of the optimality subproblem
    // We create the dual constant from the constraints that do not include u_gt.
    public double generateObtCutConstant() throws IloException{
        // Generates the constant part of the cut
        double constant = 0;
        for(int t = 1; t <= UCP.getnHours(); t++){
            constant = constant + model.getDual(demandConstraints[t-1])*UCP.getDemand(t);
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                constant = constant + model.getDual(upRampConstraints[g-1][t-1])*UCP.getRampUp(g) + model.getDual(lowRampConstraints[g-1][t-1])*UCP.getRampDown(g);
            }
        }
    return constant;    
    }
    
    // We now create the LinearTerm from the constraints 
    // in the subproblem(second problem? secondary problem?) that include u_gt.
    public IloLinearNumExpr getCutLinearTerm(IloIntVar u[][]) throws IloException{
        IloLinearNumExpr cutTerm = model.linearNumExpr();
        // Generates the term in u
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                cutTerm.addTerm(model.getDual(startUpConstraints[g-1][t-1])*UCP.getStartUpCost(g), u[g-1][t-1]);
                cutTerm.addTerm(model.getDual(lowProdConstraints[g-1][t-1])*UCP.getMinProduction(g), u[g-1][t-1]);
                cutTerm.addTerm(model.getDual(upProdConstraints[g-1][t-1])*UCP.getMaxProduction(g), u[g-1][t-1]);
            if(g > 1){
                cutTerm.addTerm(model.getDual(startUpConstraints[g-1][t-1])*UCP.getStartUpCost(g), u[g-1][t-2]);
            }
            }
        }
        return cutTerm;
    }
    
    
    // Here we get the objective function value from the obtimality subproblem to check if our solution is optimal
    public double getObjective() throws IloException{
        return model.getObjValue();
    }
    
    // Here we store the solution. This will only be needed if we find an optimal solution and want to print it
    // ... Or if we find a nonoptimal solution and we want to print that...
    public double[][] getCSolution() throws IloException{
        double C[][] = new double[UCP.getnGenerators()][UCP.getnHours()];
        for(int j = 1; j <= UCP.getnHours(); j++){
            for(int i = 1; i <= UCP.getnGenerators(); i++){
                C[i-1][j-1] = model.getValue(c[i-1][j-1]);   
            }      
        }
        return C;
    }
    
    public double[][] getPSolution() throws IloException{
        double P[][] = new double[UCP.getnGenerators()][UCP.getnHours()];
        for(int j = 1; j <= UCP.getnHours(); j++){
            for(int i = 1; i <= UCP.getnGenerators(); i++){       
                P[i-1][j-1] = model.getValue(p[i-1][j-1]);
            }
        }
        return P;
    }
    
    public double[] getLSolution() throws IloException{
        double L[] = new double[UCP.getnHours()];
        for(int j = 1; j <= UCP.getnHours(); j++){
                L[j-1] = model.getValue(l[j-1]);
        }
        return L;
    }
    
    // This will release all variables from the IloCplex.
    public void end(){
        model.end();
    }
    
}