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

/**
 *
 * @author vtodd
 */
// Here we bring in all decision variables and constraints not in the MP.
// Not that for every constraint we add an auxiliary variable (two for (1e) as it is an equality).
public class FeasSubProblem {
    private final IloCplex model;
    private final IloNumVar c[][]; //costs
    private final IloNumVar p[][]; //production
    private final IloNumVar l[];   //shed
    private final IloNumVar v1[][]; //(1b)
    private final IloNumVar v2plus[];   //(1e)
    private final IloNumVar v2minus[];   //(1e)
    private final IloNumVar v3[][]; //(1f)
    private final IloNumVar v4[][]; //(1g)
    private final IloNumVar v5[][]; //(1h)
    private final IloNumVar v6[][]; //(1i)
    private final UnitCommitmentProblem UCP;
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
        
        //We create decesion variables. c and p need g*t and l only need t.
        c = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        p = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        l = new IloNumVar[UCP.getnHours()];
        // Our auxiliary variables need to be same dimension as the constrain to which they belong.
        this.v1 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        this.v2plus = new IloNumVar[UCP.getnHours()];
        this.v2minus = new IloNumVar[UCP.getnHours()];
        this.v3 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        this.v4 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        this.v5 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        this.v6 = new IloNumVar[UCP.getnGenerators()][UCP.getnHours()];
        
        // Now we just make sure that all variables are non-negative.
        for(int j = 1; j <= UCP.getnHours(); j++){
                for(int i = 1; i<= UCP.getnGenerators(); i++){
                c[i-1][j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                p[i-1][j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v1[i-1][j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v3[i-1][j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v4[i-1][j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v5[i-1][j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
                v6[i-1][j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
            }
        l[j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
        v2plus[j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
        v2minus[j-1] = model.numVar(0, Double.POSITIVE_INFINITY);
        }
        
        IloLinearNumExpr obj = model.linearNumExpr();
        
        // The objective consists of minimizing the sum of the slack and surplus variables. 
        // Adds terms to the equation
        for(int j = 1; j<= UCP.getnHours(); j++){
            for(int i = 1; i <= UCP.getnGenerators(); i++){
            obj.addTerm(1, v1[i-1][j-1]);
            obj.addTerm(1, v3[i-1][j-1]);
            obj.addTerm(1, v4[i-1][j-1]);
            obj.addTerm(1, v5[i-1][j-1]);
            obj.addTerm(1, v6[i-1][j-1]);
        }
            obj.addTerm(1, v2plus[j-1]);
            obj.addTerm(1, v2minus[j-1]);
        }
        
        // Tells cplex to minimize the objective function
        model.addMinimize(obj);
        
        // Here we add (1b) to our feassub where we bring in the v1-slack as positive
        this.startUpConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int i = 1; i <= UCP.getnGenerators(); i++){
            for(int j = 1; j <= UCP.getnHours(); j++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, c[i-1][j-1]);
                lhs.addTerm(1, v1[i-1][j-1]);
                if(j == 1){
                    startUpConstraints[i-1][j-1] = model.addGe(lhs, UCP.getStartUpCost(i)*(U[i-1][j-1]));
                }else{
                startUpConstraints[i-1][j-1] = model.addGe(lhs, UCP.getStartUpCost(i)*(U[i-1][j-1]-U[i-1][j-2]));
            }
            }
        }
        
        // Here we add (1e) where we bring in the v2plus and v2minus as (1e) is an equality constraint.
        this.demandConstraints = new IloRange[UCP.getnHours()];
        for(int j = 1; j <= UCP.getnHours() ; j++){
            IloLinearNumExpr lhs = model.linearNumExpr();
            for(int i = 1; i <= UCP.getnGenerators(); i++){
            lhs.addTerm(p[i-1][j-1], 1);
            }
            lhs.addTerm(l[j-1], 1);
            lhs.addTerm(v2plus[j-1], 1);
            lhs.addTerm(v2minus[j-1], -1);
            demandConstraints[j-1] = model.addEq(lhs, UCP.getDemand(j));
        }
        
        // Here we add (1f) where we bring in v3. 
        this.lowProdConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int i = 1; i <= UCP.getnGenerators(); i++){
            for(int j = 1; j <= UCP.getnHours(); j++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, p[i-1][j-1]);
                lhs.addTerm(1, v3[i-1][j-1]);
                lowProdConstraints[i-1][j-1] = model.addGe(lhs, UCP.getMinProduction(i)*U[i-1][j-1]);
            }
        }  
        
        // Here we add (1g) where we bring in v4. Note that this is negative as our decision variable has an upper bound
        this.upProdConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int i = 1; i <= UCP.getnGenerators(); i++){
            for(int j = 1; j <= UCP.getnHours(); j++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, p[i-1][j-1]);
                lhs.addTerm(-1, v4[i-1][j-1]);
                upProdConstraints[i-1][j-1] = model.addLe(lhs, UCP.getMaxProduction(i)*U[i-1][j-1]);
            }
        }
        
        // Here we add (1h) where we bring in v5.
        this.upRampConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int i = 1; i <= UCP.getnGenerators(); i++){
            for(int j = 1; j <= UCP.getnHours(); j++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, p[i-1][j-1]);
                lhs.addTerm(-1, v5[i-1][j-1]);
                if(j > 1){
                    lhs.addTerm(-1, p[i-1][j-2]);
                }
                upRampConstraints[i-1][j-1] = model.addLe(lhs, UCP.getRampUp(i));
            }
        }
        
        // Here we add (1i) where we bring in v6.
        this.lowRampConstraints = new IloRange[UCP.getnGenerators()][UCP.getnHours()];
        for(int i = 1; i <= UCP.getnGenerators(); i++){
            for(int j = 1; j <= UCP.getnHours(); j++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(-1, p[i-1][j-1]);
                lhs.addTerm(-1, v6[i-1][j-1]);
                if(j > 1){
                    lhs.addTerm(1, p[i-1][j-2]);
                }
                lowRampConstraints[i-1][j-1] = model.addLe(lhs, UCP.getRampDown(i));
            }
        }
        
    }
    
    //Resets the model and solves it
    public void solve() throws IloException{
        model.setOut(null);
        model.solve();
    }
    
    // We get our objective function value for the feasibility problem. 
    // This is needed to check if it is non-positive.
    public double getObjective() throws IloException{
        return model.getObjValue();
    }
    
    //If this is not the case, we use the dual variables to create a feasibility cut.
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
    
     //Releases all the objects retained by the IloCplex object. 
    public void end(){
        model.end();
    }
}
