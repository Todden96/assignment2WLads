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
    private final IloRange commitmentConstraints[][];
    private final IloRange demandConstraints[];
    private final IloRange lowProdConstraints[][];
    private final IloRange upProdConstraints[][];
    private final IloRange upRampConstraints[][];
    private final IloRange lowRampConstraints[][];
    /**
     * Creates the LP model for the optimality subproblem
     * @param cpp
     * @param U a solution to MP
     * @throws IloException 
     */
    public OptSubProblem(UnitCommitmentProblem cpp, double U[][]) throws IloException {
        this.UCP = cpp;
        
        
        // 1. Every model needs an IloCplex object
        this.model = new IloCplex();
        
        // 2. Creates the decision variables
        // and we have 5 cities to serve
        c = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        p = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        l = new IloNumVar[UCP.getnHours()];
        
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
 
        
        
        
        this.commitmentConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, c[g-1][t-1]);
                // We bring the right-hand-side to the lhs, changing sign
                                //This will save our code from crashing
                if(t > 1){
                    commitmentConstraints[g-1][t-1] = model.addGe(lhs, UCP.getStartUpCost(g)*(U[g-1][t-1]-U[g-1][t-2]));
                }else{
                    commitmentConstraints[g-1][t-1] = model.addGe(lhs , UCP.getStartUpCost(g)*U[g-1][t-1]);
                }
                
            }
        }
        this.demandConstraints = new IloRange[UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            IloLinearNumExpr lhs = model.linearNumExpr();
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                lhs.addTerm(1, p[g-1][t-1]);
            }
            lhs.addTerm(1, l[t-1]);
            demandConstraints[t-1] = model.addEq(lhs, UCP.getDemand(t));
        }
        
        
        
        // 4. Constraints (1f)
        this.lowProdConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1,p[g-1][t-1]);
                lowProdConstraints[g-1][t-1] = model.addGe(lhs, UCP.getMinProduction(g)*U[g-1][t-1]);
                
            
            }
        }
        
        // 5. Constraints (1g)
        this.upProdConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1,p[g-1][t-1]);
                upProdConstraints[g-1][t-1] = model.addLe(lhs,UCP.getMaxProduction(g)*U[g-1][t-1]);
            }
        }
        
        // 6. Constraints (1h)
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
    
    public void solve() throws IloException{
        model.setOut(null);
        model.solve();
    }
    
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
    
    public IloLinearNumExpr getCutLinearTerm(IloIntVar u[][]) throws IloException{
        IloLinearNumExpr cutTerm = model.linearNumExpr();
        // Generates the term in x
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                cutTerm.addTerm(model.getDual(commitmentConstraints[g-1][t-1])*UCP.getStartUpCost(g), u[g-1][t-1]);
                cutTerm.addTerm(model.getDual(lowProdConstraints[g-1][t-1])*UCP.getMinProduction(g), u[g-1][t-1]);
                cutTerm.addTerm(model.getDual(upProdConstraints[g-1][t-1])*UCP.getMaxProduction(g), u[g-1][t-1]);
            if(g > 1){
                cutTerm.addTerm(model.getDual(commitmentConstraints[g-1][t-1])*UCP.getStartUpCost(g), u[g-1][t-2]);
            }
            }
        }
        return cutTerm;
    }
    
    /**
     * Returns the objective value
     * @return the objective value
     * @throws IloException 
     */
    public double getObjective() throws IloException{
        return model.getObjValue();
    }
    /**
     * Returns the solution to the problem.
     * @return the solution y for each i-j.
     * @throws IloException 
     */
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
   
    public void end(){
        model.end();
    }
    
}