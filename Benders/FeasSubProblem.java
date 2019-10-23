/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Benders;

import ilog.concert.IloException;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloIntVar;
import ilog.concert.IloRange;
import ilog.cplex.IloCplex;
import assignement2.UnitCommitmentProblem;

// Here we bring in all decision variables and constraints not in the MP.
// Note that for every constraint we add an auxiliary variable (two for (1e) as it is an equality).
public class FeasSubProblem {
    private final IloCplex model;
    //Decision variables
    private final IloNumVar c[][]; //costs
    private final IloNumVar p[][]; //production
    private final IloNumVar l[];   //shed
    //Auxiliary variables
    private final IloNumVar v1[][]; //(1b)
    private final IloNumVar v2plus[];   //(1e)
    private final IloNumVar v2minus[];   //(1e)
    private final IloNumVar v3[][]; //(1f)
    private final IloNumVar v4[][]; //(1g)
    private final IloNumVar v5[][]; //(1h)
    private final IloNumVar v6[][]; //(1i)
    //An instance of the Unit Commitment Problem
    private final UnitCommitmentProblem UCP;
    //Constraints
    private final IloRange startUpConstraints[][];
    private final IloRange demandConstraints[];
    private final IloRange lowProdConstraints[][];
    private final IloRange upProdConstraints[][];
    private final IloRange upRampConstraints[][];
    private final IloRange lowRampConstraints[][];
    
    public FeasSubProblem(UnitCommitmentProblem ucp, double U[][]) throws IloException{
        //Note the U[][] argument is the fixed complicating variables from the MP.
        this.UCP = ucp;
        this.model = new IloCplex();
        
        //We create decision variables. c and p need g*t and l only need t.
        c = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        p = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        l = new IloNumVar[UCP.getnHours()];
        // Our auxiliary variables need to be the same dimension as the constraints to which they belong.
        this.v1 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        this.v2plus = new IloNumVar[UCP.getnHours()];
        this.v2minus = new IloNumVar[UCP.getnHours()];
        this.v3 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        this.v4 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        this.v5 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        this.v6 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        
        // Now we just make sure that all variables are non-negative.
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g<= UCP.getnGenerators(); g++){
                c[g-1][t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                p[g-1][t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v1[g-1][t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v3[g-1][t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v4[g-1][t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v5[g-1][t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v6[g-1][t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                }
            // the variables of dimension t is outside the g loop
            l[t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
            v2plus[t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
            v2minus[t-1] = model.numVar(0, Double.POSITIVE_INFINITY);
        }
        
        //Here we create the objective function
        IloLinearNumExpr obj = model.linearNumExpr();
        
        // The objective consists of minimizing the sum of the auxiliary variables. 
        // Add terms to the objective function
        for(int t = 1; t<= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                obj.addTerm(1, v1[g-1][t-1]);
                obj.addTerm(1, v3[g-1][t-1]);
                obj.addTerm(1, v4[g-1][t-1]);
                obj.addTerm(1, v5[g-1][t-1]);
                obj.addTerm(1, v6[g-1][t-1]);
            }
            obj.addTerm(1, v2plus[t-1]);
            obj.addTerm(1, v2minus[t-1]);
        }
        
        // Tells cplex to minimize the objective function. This means that we want to obtain no
        // contribution from the auxiliary variables.
        model.addMinimize(obj);
        
        // Here we add (1b) to our feasible subproblem where we bring in the v1-auxiliary variable
        // with positive contribution. We save the constraints in a matrix to get the duals later.
        this.startUpConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                //Note that we make a constraint for each g and t
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, c[g-1][t-1]);
                lhs.addTerm(1, v1[g-1][t-1]);
                //We assume u_{g,0} is zero and only subtract u_{g,t-2} for t>1.
                if(t == 1){
                    startUpConstraints[g-1][t-1] = model.addGe(lhs, UCP.getStartUpCost(g)*(U[g-1][t-1]));
                }else{
                startUpConstraints[g-1][t-1] = model.addGe(lhs, UCP.getStartUpCost(g)*(U[g-1][t-1]-U[g-1][t-2]));
            }
            }
        }
        
        // Here we add (1e) where we bring in the v2plus (positive contribution and v2minus (negative contribution)
        // as (1e) is an equality constraint.
        this.demandConstraints = new IloRange[UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours() ; t++){
            //We make a constraint for each t
            IloLinearNumExpr lhs = model.linearNumExpr();
            for(int g = 1; g <= UCP.getnGenerators(); g++){
            lhs.addTerm(p[g-1][t-1], 1);
            }
            lhs.addTerm(l[t-1], 1);
            lhs.addTerm(v2plus[t-1], 1);
            lhs.addTerm(v2minus[t-1], -1);
            demandConstraints[t-1] = model.addEq(lhs, UCP.getDemand(t));
        }
        
        // Here we add (1f) where we bring in v3. 
        this.lowProdConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                //Again we make a constraint for each t and g.
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, p[g-1][t-1]);
                lhs.addTerm(1, v3[g-1][t-1]);
                lowProdConstraints[g-1][t-1] = model.addGe(lhs, UCP.getMinProduction(g)*U[g-1][t-1]);
            }
        }  
        
        // Here we add (1g) where we bring in v4. 
        // Note that this is negative as our decision variable has an upper bound
        this.upProdConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, p[g-1][t-1]);
                lhs.addTerm(-1, v4[g-1][t-1]);
                upProdConstraints[g-1][t-1] = model.addLe(lhs, UCP.getMaxProduction(g)*U[g-1][t-1]);
            }
        }
        
        // Here we add (1h) where we bring in v5.
        this.upRampConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, p[g-1][t-1]);
                lhs.addTerm(-1, v5[g-1][t-1]);
                //Again we assume p_{g,0}=0 therefore subtract p_{g,t-2} from t>1 (remember 0-indexed)
                if(t > 1){
                    lhs.addTerm(-1, p[g-1][t-2]);
                }
                upRampConstraints[g-1][t-1] = model.addLe(lhs, UCP.getRampUp(g));
            }
        }
        
        // Here we add (1i) where we bring in v6.
        this.lowRampConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int g = 1; g <= UCP.getnGenerators(); g++){
            for(int t = 1; t <= UCP.getnHours(); t++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(-1, p[g-1][t-1]);
                lhs.addTerm(-1, v6[g-1][t-1]);
                //The same conditions as for above - we subtract p_{g,t-2} from t>1.
                if(t > 1){
                    lhs.addTerm(1, p[g-1][t-2]);
                }
                lowRampConstraints[g-1][t-1] = model.addLe(lhs, UCP.getRampDown(g));
            }
        }
        
    }
    
    //Here we create a method to get the amount of shedding that is needed to give feasibility for each t.
     public double[] getL() throws IloException{
        double L[] = new double[UCP.getnHours()];
        for(int t = 1; t <= UCP.getnHours() ; t++){
            L[t-1] = model.getValue(l[t-1]);
        }
        return L;
    }
    //Solves the feasibility sub problem
    public void solve() throws IloException{
        model.solve();
        
    }
    
    // We get our objective function value for the feasibility problem. 
    // This is needed to check whether it is strictly positive or zero (we want the latter).
    public double getObjective() throws IloException{
        return model.getObjValue();
        
    }
    
    //If the objective value is strictly positive, we use the dual variables to create a feasibility cut.
    // We create the dual constant from the constraints that do not include u_gt.
    public double generateFeasCutConstant() throws IloException{
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
    // in the subproblem that includes u_gt.
    public IloLinearNumExpr getCutLinearTerm(IloIntVar u[][]) throws IloException{
        IloLinearNumExpr cutTerm = model.linearNumExpr();
        // Generates the term in u
        for(int t = 1; t <= UCP.getnHours(); t++){
            for(int g = 1; g <= UCP.getnGenerators(); g++){
                cutTerm.addTerm(model.getDual(startUpConstraints[g-1][t-1])*UCP.getStartUpCost(g), u[g-1][t-1]);
                cutTerm.addTerm(model.getDual(lowProdConstraints[g-1][t-1])*UCP.getMinProduction(g), u[g-1][t-1]);
                cutTerm.addTerm(model.getDual(upProdConstraints[g-1][t-1])*UCP.getMaxProduction(g), u[g-1][t-1]);
            if(t > 1){
                cutTerm.addTerm(-model.getDual(startUpConstraints[g-1][t-1])*UCP.getStartUpCost(g), u[g-1][t-2]);
            }
            }
        }
        return cutTerm;
    }
}
