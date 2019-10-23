/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package assignement2;

import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;


public class UnitCommitmentModel {
    
    private final IloCplex model;
    private final IloIntVar u[][];
    private final IloNumVar c[][];
    private final IloNumVar l[];
    private final IloNumVar p[][];
    private final UnitCommitmentProblem ucp;
    
    //This model will describe everything in the UnitCommitmentProblem such a variables, 
    //constraints, objective function. Note that limits on decision variables are 
    //taken care of in the range in which they can live.
    public UnitCommitmentModel(UnitCommitmentProblem ucp) throws IloException {
        this.ucp = ucp;
        this.model = new IloCplex();
        
        // Creates the decision variables 
        
        // 1. The u_gt variables
        //Decides if generator g is turned on at time t
        u = new IloIntVar[ucp.getnGenerators()][ucp.getnHours()];
        for(int g = 1; g <= ucp.getnGenerators(); g++){
            for(int t = 1; t <= ucp.getnHours(); t++){
                u[g-1][t-1] = model.intVar(0,1,"u_"+g+"_"+t);
            }
        }
        
        // 2. The c_gt variables
        // Adds the start up cost for generator g to start up at time t.
        // Note that the range of the variable holds constraint (1j) accounted for.
        c = new IloNumVar[ucp.getnGenerators()][ucp.getnHours()];
        for(int g = 1; g <= ucp.getnGenerators(); g++){
            for(int t = 1; t <= ucp.getnHours(); t++){
                c[g-1][t-1] = model.numVar(0,Double.POSITIVE_INFINITY,"c_"+g+"_"+t);
            }
        }
        
        // 3. The l_t variables
        // This variable is the shed variable if we at time t miss power from our own generation.
        // Note that the range of the variable holds constraint (1m) accounted for. 
        // This means that we cannot produce extra power and sell it at shed cost.
        l = new IloNumVar[ucp.getnHours()];
        for(int t = 1; t <= ucp.getnHours(); t++){
            l[t-1] = model.numVar(0,Double.POSITIVE_INFINITY,"l_"+t);
        }
        
        // 4. The p_gt variables
        // This variable tells us how much power generator g is producing at time t.
        // Note that the range of the variable holds constraint (1l) accounted for.
        p = new IloNumVar[ucp.getnGenerators()][ucp.getnHours()];
        for(int g = 1; g <= ucp.getnGenerators(); g++){
            for(int t = 1; t <= ucp.getnHours(); t++){
                p[g-1][t-1] = model.numVar(0,Double.POSITIVE_INFINITY,"p_"+g+"_"+t);
            }
        }
        
        
        // Creates the objective function
        
        // 1. Creates an empty linear expression
        IloLinearNumExpr obj = model.linearNumExpr();
        
        for(int t = 1; t <= ucp.getnHours(); t++){
            for(int g = 1; g <= ucp.getnGenerators(); g++){
            // 2. Adds the start-up costs, commitment costs and production costs
                obj.addTerm(1, c[g-1][t-1]);    
                obj.addTerm(ucp.getCommitmentCost(g), u[g-1][t-1]);
            // Note that the getter takes variable g whereas the variable u is assigned at [g-1][t-1]
            // This is because we already specified in the UCP class that 'getting' int g means
            // returning position [g-1] in the CommitmentCost vector.
            // This is the case for all getters.
                obj.addTerm(ucp.getProductionCost(g), p[g-1][t-1]);
            }
            // 3. Adds the shedding costs
            // Note this is outside the g loop as it has nothing to do with the generators.
            obj.addTerm(ucp.getSheddingCost(t), l[t-1]);
        }
        
        // 4. Adds the objective function to the model
        model.addMinimize(obj);
        
        
        // Creates the constraints
        
        // 1. Constraints (1b)
        // We have one (1b) constraint for each hour and generator
        for(int t = 1; t <= ucp.getnHours(); t++){
            for(int g = 1; g <= ucp.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1, c[g-1][t-1]);
                // We bring the right-hand-side to the lhs, changing sign
                lhs.addTerm(-ucp.getStartUpCost(g), u[g-1][t-1]);
                if(t > 1){
                    lhs.addTerm(+ucp.getStartUpCost(g), u[g-1][t-2]); 
                // Note that in order to get u_g,t-1 we need to access u[g-1][t-2] (notice the -2)
                // This will save our code from crashing
                }
                // Finally we add the constraint to our model
                model.addGe(lhs, 0);
            }
        }
        
        // 2. Constraints (1c)
        for(int t = 1; t <= ucp.getnHours(); t++){
            for(int g = 1; g <= ucp.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                // We loop over t'
                for(int tt = t; tt <= ucp.getMinOnTimeAtT(g, t); tt++){
                    lhs.addTerm(1, u[g-1][tt-1]);
                    lhs.addTerm(-1, u[g-1][t-1]);
                    if(t > 1){
                        lhs.addTerm(1, u[g-1][t-2]);
                    }
                }
                model.addGe(lhs,0);
            }
        }
        
        // 2. Constraints (1d)
        for(int t = 1; t <= ucp.getnHours(); t++){
            for(int g = 1; g <= ucp.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                // We loop over t'
                // We add one every time we loop. We save this sum in a constant
                int constant = 0;
                for(int tt = t; tt <= ucp.getMinOffTimeAtT(g, t); tt++){
                    constant++;
                    lhs.addTerm(-1, u[g-1][tt-1]);
                    lhs.addTerm(1, u[g-1][t-1]);
                    if(t > 1){
                        lhs.addTerm(-1, u[g-1][t-2]);
                    }
                }
                //We have changed this to "-constant" as we move the constant to the RHS of the inequality.                
                model.addGe(lhs,-constant);
            }
        }
        
        // 3. Constraints (1e)
        for(int t = 1; t <= ucp.getnHours(); t++){
            IloLinearNumExpr lhs = model.linearNumExpr();
            //Note that this is created first, so we can create a sum inside all t constraint.
            for(int g = 1; g <= ucp.getnGenerators(); g++){
                lhs.addTerm(1, p[g-1][t-1]);
            }
            lhs.addTerm(1, l[t-1]);
            model.addEq(lhs, ucp.getDemand(t));
        }
        
        // 4. Constraints (1f)
        for(int t = 1; t <= ucp.getnHours(); t++){
            for(int g = 1; g <= ucp.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1,p[g-1][t-1]);
                lhs.addTerm(-ucp.getMinProduction(g), u[g-1][t-1]);
                model.addGe(lhs,0);
            }
        }
        
        // 5. Constraints (1g)
        for(int t = 1; t <= ucp.getnHours(); t++){
            for(int g = 1; g <= ucp.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1,p[g-1][t-1]);
                lhs.addTerm(-ucp.getMaxProduction(g), u[g-1][t-1]);
                model.addLe(lhs,0);
            }
        }
        
        // 6. Constraints (1h)
        for(int t = 1; t <= ucp.getnHours(); t++){
            for(int g = 1; g <= ucp.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                lhs.addTerm(1,p[g-1][t-1]);
                if(t > 1){
                    lhs.addTerm(-1,p[g-1][t-2]);
                }
                model.addLe(lhs,ucp.getRampUp(g));
            }
        }
        
        // 7. Constraints (1i)
        for(int t = 1; t <= ucp.getnHours(); t++){
            for(int g = 1; g <= ucp.getnGenerators(); g++){
                IloLinearNumExpr lhs = model.linearNumExpr();
                if(t > 1){
                    lhs.addTerm(1,p[g-1][t-2]);
                }
                lhs.addTerm(-1,p[g-1][t-1]);
                model.addLe(lhs,ucp.getRampDown(g));
            }
        }
        
        
    }
    
    public void solve() throws IloException{
        model.solve();
        System.out.println("Optimal objectve "+model.getObjValue());
    }
    
    // For every generator we can see 1's at the times they are on and zero otherwise.
    public void printSolution() throws IloException{
        System.out.println("Commitment");
        for(int g = 1; g <= ucp.getnGenerators(); g++){
            System.out.print(ucp.getGeneratorName(g));
            for(int t =1; t <= ucp.getnHours(); t++){
                System.out.print(String.format(" %4.0f ", model.getValue(u[g-1][t-1])));
            }
            System.out.println("");
        }
        
        // For every generator we can see the start up costs whenever they start up and zero otherwise
        System.out.println("Startup costs");
        for(int g = 1; g <= ucp.getnGenerators(); g++){
            System.out.print(ucp.getGeneratorName(g));
            for(int t =1; t <= ucp.getnHours(); t++){
                System.out.print(String.format(" %4.2f ", model.getValue(c[g-1][t-1])));
            }
            System.out.println("");
        }
        
        // For every generator g we can see how much they produce at all time t
        System.out.println("Production");
        for(int g = 1; g <= ucp.getnGenerators(); g++){
            System.out.print(ucp.getGeneratorName(g));
            for(int t =1; t <= ucp.getnHours(); t++){
                System.out.print(String.format(" %4.2f ", model.getValue(p[g-1][t-1])));
            }
            System.out.println("");
        }
        
    }
    public void end(){
        model.end();
    }
    
    
    
}
