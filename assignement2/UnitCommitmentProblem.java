/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package assignement2;

/**
 *
 * @author vtodd
 */
public class UnitCommitmentProblem {
    
       // The state contains the data of the problem
    private final int nHours;
    private final int nGenerators;
    private final String[] generatorName;
    private final double[] commitmentCost;
    private final double[] productionCost;
    private final double[] sheddingCost;
    private final double[] startUpCost;
    private final int[] minOnTime;
    private final int[] minOffTime;
    private final int[][] minOnTimeAtT;
    private final int[][] minOffTimeAtT;
    private final double[] minProduction;
    private final double[] maxProduction;
    private final double[] rampUp;
    private final double[] rampDown;
    private final double[] demand;

    public UnitCommitmentProblem(int nHours, int nGenerators, String[] generatorName, 
            double[] commitmentCost, double[] productionCost, double[] sheddingCost, 
            double[] startUpCost, int[] minOnTime, int[] minOffTime, int[][] minOnTimeAtT, int[][] minOffTimeAtT, double[] minProduction, double[] maxProduction, double[] rampUp, double[] rampDown, double[] demand) {
        this.nHours = nHours;
        this.nGenerators = nGenerators;
        this.generatorName = generatorName;
        this.commitmentCost = commitmentCost;
        this.productionCost = productionCost;
        this.sheddingCost = sheddingCost;
        this.startUpCost = startUpCost;
        this.minOnTime = minOnTime;
        this.minOffTime = minOffTime;
        this.minOnTimeAtT = minOnTimeAtT;
        this.minOffTimeAtT = minOffTimeAtT;
        this.minProduction = minProduction;
        this.maxProduction = maxProduction;
        this.rampUp = rampUp;
        this.rampDown = rampDown;
        this.demand = demand;
    }

    public int getnHours() {
        return nHours;
    }

    public int getnGenerators() {
        return nGenerators;
    }
    /**
     * Returns the commitment cost for generator number g.
     * Note that we need to pass the generator number in 1...nGenerators,
     * (This is an arbitrary choice), and it retrieves the cost as
     * commitmentCost[i-1]. The same convention applies to the remaining methods.
     * @param g
     * @return 
     */
    public double getCommitmentCost(int g) {
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return commitmentCost[g-1];
    }

    public double getProductionCost(int g) {
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return productionCost[g-1];
    }
    
    /**
     * Returns the shedding cost for a given hour.
     * @param h
     * @return 
     */
    public double getSheddingCost(int h) {
        System.out.println("h = "+h+ " "+nHours);
        if(h < 1 || h > nHours){
            throw new IllegalArgumentException("The hour number must be in [1,"+nHours+"].");
        }
        return sheddingCost[h-1];
    }

    public String getGeneratorName(int g){
        //Gives the name of generator g on the list of generators
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return generatorName[g-1];
    }
    public double getStartUpCost(int g) {
        //Gives the start up cost of generator g on the list of generatorts
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return startUpCost[g-1];
    }

    public int getMinOnTime(int g) {
        //Gives the minimum on-time if we choose to turn on generator g
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return minOnTime[g-1];
    }

    public int getMinOffTime(int g) {
        //Gives the minimum off-time if we choose to turn on generator g
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return minOffTime[g-1];
    }
   
    public int getMinOnTimeAtT(int g, int h) {
        //Gives the minimum on-time (remaining on-time) for generator g at time h
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        if(h < 1 || h > nGenerators){
            throw new IllegalArgumentException("The hour number must be in [1,"+nHours+"].");
        }
        return minOnTimeAtT[g-1][h-1];
    }

    public int getMinOffTimeAtT(int g, int h) {
        //Gives the minimum off-time (remaining off-time) for generator g at time h
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        if(h < 1 || h > nGenerators){
            throw new IllegalArgumentException("The hour number must be in [1,"+nHours+"].");
        }
        return minOffTimeAtT[g-1][h-1];
    }
    
    public double getMinProduction(int g) {
        //The minimum a generator can generate if it is turned on
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return minProduction[g-1];
    }

    public double getMaxProduction(int g) {
        //The maximum a generator can generate if it is turned on
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return maxProduction[g-1];
    }

    public double getRampUp(int g) {
        //The maximum the generator g can ramp up during one time period
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return rampUp[g-1];
    }

    public double getRampDown(int g) {
        //The maximum the generator g can ramp down during one time period
        if(g < 1 || g > nGenerators){
            throw new IllegalArgumentException("The generator number must be in [1,"+nGenerators+"].");
        }
        return rampDown[g-1];
    }

    public double getDemand(int h) {
        //Returns the amount of power (in MW) needed at time h
        if(h < 1 || h > nGenerators){
            throw new IllegalArgumentException("The hour number must be in [1,"+nGenerators+"].");
        }
        return demand[h-1];
    }
    
    
    
    
    
}
