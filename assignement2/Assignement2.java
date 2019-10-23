/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package assignement2;

import ilog.concert.IloException;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import Benders.MasterProblem;
import java.util.Locale;

/**
 *
 * @author vtodd
 */
public class Assignement2 {
/**
     * @param args the command line arguments
     * @throws java.io.FileNotFoundException
     * @throws ilog.concert.IloException
     */
    public static void main(String[] args) throws FileNotFoundException, IloException{
   
        // 1. READS THE GENERATORS DATA
        // The generators data are stored in the generators.txt file
        // Note that the generators.txt file must be in project's specific NetBeans 
        // directory. If the generators.txt file is elsewhere you will need to provide
        // an absolute path such as "C://Users/Documents/MyFolder/generators.txt"
        File generatorsFile = new File("generators.txt");
        Scanner scanner = new Scanner(generatorsFile).useLocale(Locale.US);
        
        // We skip the first two lines of the file since they are a header
        // That is, we move the cursor past the first two lines (at the beginning of the third)
        scanner.nextLine();
        scanner.nextLine();
        
        // We create the arrays for storing the data we read.
        // These arrays will then be used to create an instance of 
        // the UnitCommitmentProblem class.
        
        // We have 31 generators in the file
        int nGenerators = 31;
        String[] names = new String[nGenerators];
        double[] minProduction = new double[nGenerators];
        double[] maxProduction = new double[nGenerators];
        double[] startUpCosts = new double[nGenerators];
        double[] commitmentCosts = new double[nGenerators];
        double[] rampUp = new double[nGenerators];
        double[] rampDown = new double[nGenerators];
        int[] minUpTime = new int[nGenerators];
        int[] minDownTime = new int[nGenerators];
        double[] productionCost = new double[nGenerators];
        
        for(int g = 1; g <= nGenerators; g++){
            names[g-1] = scanner.next();
            minProduction[g-1] = scanner.nextDouble();
            maxProduction[g-1] = scanner.nextDouble();
            startUpCosts[g-1] = scanner.nextDouble();
            commitmentCosts[g-1] = scanner.nextDouble();
            rampUp[g-1] = scanner.nextDouble();
            // RampDown is equal to RampUp
            rampDown[g-1] = rampUp[g-1];
            minUpTime[g-1] = scanner.nextInt();
            minDownTime[g-1] = scanner.nextInt();
            productionCost[g-1] = scanner.nextDouble();
        }
        
        
        // 2. READS THE LOADS DATA
        // We have 24 hours
        int nHours = 24;
        double[] demand = new double[nHours];
        
        // *************
        File loadsFile = new File("loads.txt");
        Scanner scanner2 = new Scanner(loadsFile);
        
        // We skip the first line as this is only a description
        scanner2.nextLine();
        for(int i = 1; i <= nHours; i++){
            //Fill in the demands for each hour
            demand[i-1] = scanner2.nextDouble();
        }
        
        // We calculate min up time and down time for each t
        int[][] minUpTimeAtT = new int[nGenerators][nHours];
        int[][] minDownTimeAtT = new int[nGenerators][nHours];
        for(int g = 1; g <= nGenerators; g++){
            for(int t = 1; t <= nHours; t++){
                minUpTimeAtT[g-1][t-1] = Math.min(t+minUpTime[g-1]-1, nHours);
                minUpTimeAtT[g-1][t-1] = Math.min(t+minDownTime[g-1]-1, nHours);
            }
        }
        
        //Here we define the shedding costs for every hour of the day
        double[] sheddingCosts = new double[nHours];
        double constant = 0;
        for(int t = 1; t <=nHours; t++){
            for(int g = 1; g <= nGenerators; g++){
                //We find the maximum of all the production costs of the generators.
            if(constant <= productionCost[g-1]){
                //if the constant is less than one of the previous production costs then we make the 
                //constant equal this production cost
                constant = productionCost[g-1];
                        }
            }
            //The constant will therefore attain the highest production cost of all the generators.
            //Here we assign the shedding costs as they were given in the assignment.
            sheddingCosts[t-1] = constant*2;
        }
        
        // 3. CREATES AN INSTANCE OF THE UNIT COMMITMENT PROBLEM
        UnitCommitmentProblem ucp 
                = new UnitCommitmentProblem(nHours,nGenerators,
                        names,commitmentCosts,productionCost,
                        sheddingCosts, startUpCosts,
                        minUpTime,minDownTime,
                        minUpTimeAtT,minDownTimeAtT,
                        minProduction,maxProduction,
                        rampUp,rampDown,
                        demand
        );
        
        // 4. CREATES THE MATHEMATICAL MODEL AND SOLVES IT (WITHOUT DECOMPOSITION)
        UnitCommitmentModel ucpm = new UnitCommitmentModel(ucp);
        ucpm.solve();
        ucpm.printSolution();
        
        
        // 5. SOLVES THE PROBLEM USING BENDERS DECOMPOSITION
        // ***************
        //We solve the problem using the Master Problem.
        MasterProblem mp = new MasterProblem(ucp);
        mp.solve();
        System.out.println("Best value with integer Benders = "+mp.getObjective());
        mp.printSolution();
    }
    
}
